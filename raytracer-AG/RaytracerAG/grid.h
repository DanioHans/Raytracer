#pragma once

#include "bbox.h"
#include "v3math.h"
#include "intersection.h"

// One cell in the uniform grid: dynamic list of triangle indices
typedef struct grid_cell {
    int* tri_indices;
    int count;
    int capacity;
} grid_cell;

// The grid itself
typedef struct uniform_grid {
    boundingbox bbox;   // world-space bounds of the whole grid
    int nx, ny, nz;     // number of cells in each dimension
    vec3 cell_size;     // size of a cell in x/y/z
    grid_cell* cells;   // nx * ny * nz cells, in a flat array
} uniform_grid;

// Global grid and toggle (similar to use_bvh)
extern uniform_grid* g_grid;
extern int use_grid;

void grid_build(void);
void grid_free(void);

// Ray queries
int grid_first_intersection(intersection_point* ip, vec3 ray_origin, vec3 ray_direction);
int grid_shadow_check(vec3 ray_origin, vec3 ray_direction);
