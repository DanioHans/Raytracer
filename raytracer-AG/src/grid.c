#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include "grid.h"
#include "scene.h"
#include "bbox.h"
#include "intersection.h"
#include "constants.h"

uniform_grid *g_grid = NULL;
int use_grid = 0;

// map 3D cell indices to flat index
static inline int grid_index(const uniform_grid *g, int ix, int iy, int iz) {
    return ix + g->nx * (iy + g->ny * iz);
}

static void grid_cell_add(grid_cell *cell, int tri_index) {
    if (cell->count == cell->capacity) {
        int new_cap = cell->capacity ? 2 * cell->capacity : 8;
        cell->tri_indices = (int *)realloc(cell->tri_indices, new_cap * sizeof(int));
        cell->capacity = new_cap;
    }
    cell->tri_indices[cell->count++] = tri_index;
}

void grid_build(void) {
    if (g_grid) {
        grid_free();
    }

    if (scene_num_triangles == 0) return;

    uniform_grid *g = (uniform_grid *)calloc(1, sizeof(uniform_grid));

    g->bbox = bbox_create();
    for (int i = 0; i < scene_num_vertices; i++) {
        bbox_update(&g->bbox, scene_vertices[i]);
    }

    g->bbox.min.x -= C_EPSILON; g->bbox.min.y -= C_EPSILON; g->bbox.min.z -= C_EPSILON;
    g->bbox.max.x += C_EPSILON; g->bbox.max.y += C_EPSILON; g->bbox.max.z += C_EPSILON;

    vec3 size = v3_subtract(g->bbox.max, g->bbox.min);
    float dx = size.x, dy = size.y, dz = size.z;

    // 2. Choose resolution based on triangle count and scene size
    // Simple heuristic: aim for ~N cells where N ~ num_triangles
    float volume = fmaxf(dx * dy * dz, 1e-6f);
    float target_cells = (float)scene_num_triangles;
    float cell_vol = powf(volume / target_cells, 1.0f / 3.0f);

    if (cell_vol <= 0.0f || !isfinite(cell_vol)) {
        cell_vol = 1.0f;
    }

    g->nx = (int)fmaxf(1.0f, floorf(dx / cell_vol));
    g->ny = (int)fmaxf(1.0f, floorf(dy / cell_vol));
    g->nz = (int)fmaxf(1.0f, floorf(dz / cell_vol));

    const int MAX_RES = 64;
    if (g->nx > MAX_RES) g->nx = MAX_RES;
    if (g->ny > MAX_RES) g->ny = MAX_RES;
    if (g->nz > MAX_RES) g->nz = MAX_RES;

    g->cell_size = v3_create(
        dx / (float)g->nx,
        dy / (float)g->ny,
        dz / (float)g->nz
    );

    int total_cells = g->nx * g->ny * g->nz;
    g->cells = (grid_cell *)calloc(total_cells, sizeof(grid_cell));

    for (int t = 0; t < scene_num_triangles; t++) {
        triangle tri = scene_triangles[t];

        boundingbox tb = bbox_create();
        for (int i = 0; i < 3; i++) {
            bbox_update(&tb, scene_vertices[tri.v[i]]);
        }

        vec3 minp = tb.min;
        vec3 maxp = tb.max;

        int ix_min = (int)floorf((minp.x - g->bbox.min.x) / g->cell_size.x);
        int iy_min = (int)floorf((minp.y - g->bbox.min.y) / g->cell_size.y);
        int iz_min = (int)floorf((minp.z - g->bbox.min.z) / g->cell_size.z);

        int ix_max = (int)floorf((maxp.x - g->bbox.min.x) / g->cell_size.x);
        int iy_max = (int)floorf((maxp.y - g->bbox.min.y) / g->cell_size.y);
        int iz_max = (int)floorf((maxp.z - g->bbox.min.z) / g->cell_size.z);

        if (ix_min < 0) ix_min = 0;
        if (iy_min < 0) iy_min = 0;
        if (iz_min < 0) iz_min = 0;
        if (ix_max >= g->nx) ix_max = g->nx - 1;
        if (iy_max >= g->ny) iy_max = g->ny - 1;
        if (iz_max >= g->nz) iz_max = g->nz - 1;

        for (int iz = iz_min; iz <= iz_max; iz++) {
            for (int iy = iy_min; iy <= iy_max; iy++) {
                for (int ix = ix_min; ix <= ix_max; ix++) {
                    int idx = grid_index(g, ix, iy, iz);
                    grid_cell_add(&g->cells[idx], t);
                }
            }
        }
    }

    g_grid = g;

    printf("grid_build(): %d triangles, grid %dx%dx%d (%d cells)\n",
           scene_num_triangles, g->nx, g->ny, g->nz, total_cells);
}

void grid_free(void) {
    if (!g_grid) return;
    int total_cells = g_grid->nx * g_grid->ny * g_grid->nz;
    for (int i = 0; i < total_cells; i++) {
        free(g_grid->cells[i].tri_indices);
    }
    free(g_grid->cells);
    free(g_grid);
    g_grid = NULL;
}

int grid_first_intersection(intersection_point* ip,
    vec3 ray_origin, vec3 ray_direction)
{
    if (!g_grid) return 0;

    float t_min, t_max;

    if (!bbox_intersect(&t_min, &t_max, g_grid->bbox,
        ray_origin, ray_direction,
        0.0f, C_INFINITY))
    {
        return 0;
    }

    if (t_min < 0.0f) t_min = 0.0f;

    vec3 pos = v3_add(ray_origin, v3_multiply(ray_direction, t_min));

    float gx = (pos.x - g_grid->bbox.min.x) / g_grid->cell_size.x;
    float gy = (pos.y - g_grid->bbox.min.y) / g_grid->cell_size.y;
    float gz = (pos.z - g_grid->bbox.min.z) / g_grid->cell_size.z;

    int ix = (int)floorf(gx);
    int iy = (int)floorf(gy);
    int iz = (int)floorf(gz);

    if (ix < 0) ix = 0;
    if (iy < 0) iy = 0;
    if (iz < 0) iz = 0;
    if (ix >= g_grid->nx) ix = g_grid->nx - 1;
    if (iy >= g_grid->ny) iy = g_grid->ny - 1;
    if (iz >= g_grid->nz) iz = g_grid->nz - 1;

    int   stepX, stepY, stepZ;
    float tMaxX, tMaxY, tMaxZ;
    float tDeltaX, tDeltaY, tDeltaZ;

    if (ray_direction.x > 0.0f) {
        stepX = 1;
        float nextPlaneX = g_grid->bbox.min.x + (ix + 1) * g_grid->cell_size.x;
        tMaxX = t_min + (nextPlaneX - pos.x) / ray_direction.x;
        tDeltaX = g_grid->cell_size.x / ray_direction.x;  // > 0
    }
    else if (ray_direction.x < 0.0f) {
        stepX = -1;
        float nextPlaneX = g_grid->bbox.min.x + ix * g_grid->cell_size.x;
        tMaxX = t_min + (nextPlaneX - pos.x) / ray_direction.x;
        tDeltaX = -g_grid->cell_size.x / ray_direction.x; // make > 0
    }
    else {
        stepX = 0;
        tMaxX = C_INFINITY;
        tDeltaX = C_INFINITY;
    }

    if (ray_direction.y > 0.0f) {
        stepY = 1;
        float nextPlaneY = g_grid->bbox.min.y + (iy + 1) * g_grid->cell_size.y;
        tMaxY = t_min + (nextPlaneY - pos.y) / ray_direction.y;
        tDeltaY = g_grid->cell_size.y / ray_direction.y;
    }
    else if (ray_direction.y < 0.0f) {
        stepY = -1;
        float nextPlaneY = g_grid->bbox.min.y + iy * g_grid->cell_size.y;
        tMaxY = t_min + (nextPlaneY - pos.y) / ray_direction.y;
        tDeltaY = -g_grid->cell_size.y / ray_direction.y;
    }
    else {
        stepY = 0;
        tMaxY = C_INFINITY;
        tDeltaY = C_INFINITY;
    }

    if (ray_direction.z > 0.0f) {
        stepZ = 1;
        float nextPlaneZ = g_grid->bbox.min.z + (iz + 1) * g_grid->cell_size.z;
        tMaxZ = t_min + (nextPlaneZ - pos.z) / ray_direction.z;
        tDeltaZ = g_grid->cell_size.z / ray_direction.z;
    }
    else if (ray_direction.z < 0.0f) {
        stepZ = -1;
        float nextPlaneZ = g_grid->bbox.min.z + iz * g_grid->cell_size.z;
        tMaxZ = t_min + (nextPlaneZ - pos.z) / ray_direction.z;
        tDeltaZ = -g_grid->cell_size.z / ray_direction.z;
    }
    else {
        stepZ = 0;
        tMaxZ = C_INFINITY;
        tDeltaZ = C_INFINITY;
    }

    int hit = 0;
    float best_t = t_max;
    intersection_point best_ip = { 0 };

    float t = t_min;

    while (t <= t_max) {
        if (ix < 0 || ix >= g_grid->nx ||
            iy < 0 || iy >= g_grid->ny ||
            iz < 0 || iz >= g_grid->nz) {
            break;
        }

        stats_count_grid_step();

        int cell_idx = grid_index(g_grid, ix, iy, iz);
        grid_cell* cell = &g_grid->cells[cell_idx];

        for (int i = 0; i < cell->count; i++) {
            int tri_index = cell->tri_indices[i];
            triangle tri = scene_triangles[tri_index];
            intersection_point tmp;

            if (ray_intersects_triangle(&tmp, tri, ray_origin, ray_direction)) {
                if (tmp.t >= t_min && tmp.t < best_t) {
                    best_t = tmp.t;
                    best_ip = tmp;
                    hit = 1;
                }
            }
        }

        if (tMaxX < tMaxY) {
            if (tMaxX < tMaxZ) {
                ix += stepX;
                t = tMaxX;
                tMaxX += tDeltaX;
            }
            else {
                iz += stepZ;
                t = tMaxZ;
                tMaxZ += tDeltaZ;
            }
        }
        else {
            if (tMaxY < tMaxZ) {
                iy += stepY;
                t = tMaxY;
                tMaxY += tDeltaY;
            }
            else {
                iz += stepZ;
                t = tMaxZ;
                tMaxZ += tDeltaZ;
            }
        }
    }

    if (hit) {
        *ip = best_ip;
        return 1;
    }

    return 0;
}

int grid_shadow_check(vec3 ray_origin, vec3 ray_direction)
{
    intersection_point dummy;
    return grid_first_intersection(&dummy, ray_origin, ray_direction);
}

