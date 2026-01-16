#pragma once

#include "bbox.h"
#include "v3math.h"
#include "intersection.h"

// Simple pointer-based octree node
typedef struct octree_node {
    boundingbox bbox;            // bounding box of this node
    int *tri_indices;            // triangles stored in this node (leaf only)
    int tri_count;
    int tri_capacity;
    struct octree_node *children[8]; // NULL if leaf
    int is_leaf;
} octree_node;

extern octree_node *g_octree_root;
extern int use_octree;

void octree_build(void);
void octree_free(octree_node *node);

int octree_first_intersection(intersection_point *ip,
                              vec3 ray_origin,
                              vec3 ray_direction);
int octree_shadow_check(vec3 ray_origin, vec3 ray_direction);
