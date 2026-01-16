#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "octree.h"
#include "scene.h"
#include "bbox.h"
#include "constants.h"
#include "intersection.h"
#include "constants.h"

octree_node *g_octree_root = NULL;
int use_octree = 0;

// Tunable parameters
#define OCTREE_MAX_DEPTH          10
#define OCTREE_MAX_TRIS_PER_LEAF  16

static octree_node *octree_create_node(const boundingbox *bbox)
{
    octree_node *node = (octree_node *)calloc(1, sizeof(octree_node));
    node->bbox = *bbox;
    node->is_leaf = 1;
    node->tri_indices = NULL;
    node->tri_count = 0;
    node->tri_capacity = 0;
    for (int i = 0; i < 8; ++i) {
        node->children[i] = NULL;
    }
    return node;
}

static void octree_add_tri(octree_node *node, int tri_index)
{
    if (node->tri_count == node->tri_capacity) {
        int new_cap = node->tri_capacity ? node->tri_capacity * 2 : 16;
        node->tri_indices = (int *)realloc(node->tri_indices, new_cap * sizeof(int));
        node->tri_capacity = new_cap;
    }
    node->tri_indices[node->tri_count++] = tri_index;
}

// Compute AABB for a triangle
static void tri_bbox(int tri_index, boundingbox* out)
{
    triangle tri = scene_triangles[tri_index];
    *out = bbox_create();

    for (int i = 0; i < 3; ++i) {
        vec3 v = scene_vertices[tri.v[i]];
        bbox_update(out, v);
    }
}

static int bbox_overlap(const boundingbox *a, const boundingbox *b)
{
    if (a->max.x < b->min.x || a->min.x > b->max.x) return 0;
    if (a->max.y < b->min.y || a->min.y > b->max.y) return 0;
    if (a->max.z < b->min.z || a->min.z > b->max.z) return 0;
    return 1;
}

// Subdivide node into 8 children and distribute triangles among them
static void octree_subdivide(octree_node *node, int depth)
{
    if (depth >= OCTREE_MAX_DEPTH) return;
    if (node->tri_count <= OCTREE_MAX_TRIS_PER_LEAF) return;

    vec3 min = node->bbox.min;
    vec3 max = node->bbox.max;
    vec3 mid = v3_create(0.5f * (min.x + max.x),
                         0.5f * (min.y + max.y),
                         0.5f * (min.z + max.z));

    boundingbox child_bbox[8];

    // manually define the 8 octants
    // 0: (min,min,min) - (mid,mid,mid)
    child_bbox[0].min = v3_create(min.x, min.y, min.z);
    child_bbox[0].max = v3_create(mid.x, mid.y, mid.z);

    // 1: (mid,min,min) - (max,mid,mid)
    child_bbox[1].min = v3_create(mid.x, min.y, min.z);
    child_bbox[1].max = v3_create(max.x, mid.y, mid.z);

    // 2: (min,mid,min) - (mid,max,mid)
    child_bbox[2].min = v3_create(min.x, mid.y, min.z);
    child_bbox[2].max = v3_create(mid.x, max.y, mid.z);

    // 3: (mid,mid,min) - (max,max,mid)
    child_bbox[3].min = v3_create(mid.x, mid.y, min.z);
    child_bbox[3].max = v3_create(max.x, max.y, mid.z);

    // 4: (min,min,mid) - (mid,mid,max)
    child_bbox[4].min = v3_create(min.x, min.y, mid.z);
    child_bbox[4].max = v3_create(mid.x, mid.y, max.z);

    // 5: (mid,min,mid) - (max,mid,max)
    child_bbox[5].min = v3_create(mid.x, min.y, mid.z);
    child_bbox[5].max = v3_create(max.x, mid.y, max.z);

    // 6: (min,mid,mid) - (mid,max,max)
    child_bbox[6].min = v3_create(min.x, mid.y, mid.z);
    child_bbox[6].max = v3_create(mid.x, max.y, max.z);

    // 7: (mid,mid,mid) - (max,max,max)
    child_bbox[7].min = v3_create(mid.x, mid.y, mid.z);
    child_bbox[7].max = v3_create(max.x, max.y, max.z);

    for (int i = 0; i < 8; ++i) {
        node->children[i] = octree_create_node(&child_bbox[i]);
    }

    for (int t = 0; t < node->tri_count; ++t) {
        int tri_index = node->tri_indices[t];
        boundingbox tb;
        tri_bbox(tri_index, &tb);

        for (int c = 0; c < 8; ++c) {
            if (bbox_overlap(&tb, &node->children[c]->bbox)) {
                octree_add_tri(node->children[c], tri_index);
            }
        }
    }

    free(node->tri_indices);
    node->tri_indices = NULL;
    node->tri_count = 0;
    node->tri_capacity = 0;
    node->is_leaf = 0;

    for (int c = 0; c < 8; ++c) {
        if (node->children[c]->tri_count > OCTREE_MAX_TRIS_PER_LEAF) {
            octree_subdivide(node->children[c], depth + 1);
        }
    }
}

void octree_build(void)
{
    if (g_octree_root) {
        octree_free(g_octree_root);
        g_octree_root = NULL;
    }

    if (scene_num_triangles == 0) return;

    boundingbox root_bbox = bbox_create();
    for (int i = 0; i < scene_num_vertices; ++i) {
        bbox_update(&root_bbox, scene_vertices[i]);
    }


    root_bbox.min.x -= C_EPSILON; root_bbox.min.y -= C_EPSILON; root_bbox.min.z -= C_EPSILON;
    root_bbox.max.x += C_EPSILON; root_bbox.max.y += C_EPSILON; root_bbox.max.z += C_EPSILON;

    octree_node *root = octree_create_node(&root_bbox);

    root->tri_indices = (int *)malloc(scene_num_triangles * sizeof(int));
    root->tri_capacity = scene_num_triangles;
    root->tri_count = scene_num_triangles;
    for (int t = 0; t < scene_num_triangles; ++t) {
        root->tri_indices[t] = t;
    }

    octree_subdivide(root, 0);

    g_octree_root = root;

    printf("octree_build(): %d triangles\n", scene_num_triangles);
}

void octree_free(octree_node *node)
{
    if (!node) return;

    if (node->tri_indices) {
        free(node->tri_indices);
    }
    for (int i = 0; i < 8; ++i) {
        if (node->children[i]) {
            octree_free(node->children[i]);
        }
    }
    free(node);
}

static int octree_traverse(octree_node *node,
                           vec3 ray_origin,
                           vec3 ray_direction,
                           float *best_t,
                           intersection_point *best_ip)
{
    if (!node) return 0;

    float t0, t1;
    if (!bbox_intersect(&t0, &t1, node->bbox, ray_origin, ray_direction,
                        0.0f, *best_t)) {
        return 0;
    }

    int hit = 0;

    if (node->is_leaf) {
        for (int i = 0; i < node->tri_count; ++i) {
            int tri_index = node->tri_indices[i];
            triangle tri = scene_triangles[tri_index];
            intersection_point tmp;

            if (ray_intersects_triangle(&tmp, tri, ray_origin, ray_direction)) {
                if (tmp.t >= 0.0f && tmp.t < *best_t) {
                    *best_t = tmp.t;
                    *best_ip = tmp;
                    hit = 1;
                }
            }
        }
        return hit;
    }

    float child_t0[8];
    octree_node *child_list[8];
    int child_count = 0;

    for (int i = 0; i < 8; ++i) {
        octree_node *child = node->children[i];
        if (!child) continue;

        float ct0, ct1;
        if (bbox_intersect(&ct0, &ct1, child->bbox, ray_origin, ray_direction,
                           0.0f, *best_t)) {
            child_list[child_count] = child;
            child_t0[child_count] = ct0;
            child_count++;
        }
    }

    for (int i = 1; i < child_count; ++i) {
        float key_t = child_t0[i];
        octree_node *key_c = child_list[i];
        int j = i - 1;
        while (j >= 0 && child_t0[j] > key_t) {
            child_t0[j + 1] = child_t0[j];
            child_list[j + 1] = child_list[j];
            j--;
        }
        child_t0[j + 1] = key_t;
        child_list[j + 1] = key_c;
    }

    for (int i = 0; i < child_count; ++i) {
        if (child_t0[i] > *best_t) {
            // Remaining children start further than current best hit
            break;
        }
        if (octree_traverse(child_list[i], ray_origin, ray_direction, best_t, best_ip)) {
            hit = 1;
        }
    }

    return hit;
}

int octree_first_intersection(intersection_point *ip,
                              vec3 ray_origin,
                              vec3 ray_direction)
{
    if (!g_octree_root) return 0;

    float best_t = C_INFINITY;
    intersection_point best_ip = {0};

    int hit = octree_traverse(g_octree_root, ray_origin, ray_direction, &best_t, &best_ip);
    if (hit) {
        *ip = best_ip;
        return 1;
    }
    return 0;
}

int octree_shadow_check(vec3 ray_origin, vec3 ray_direction)
{
    intersection_point tmp;
    if (octree_first_intersection(&tmp, ray_origin, ray_direction)) {
        return 1;
    }
    return 0;
}
