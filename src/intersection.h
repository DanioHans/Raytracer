#ifndef INTERSECTION_H
#define INTERSECTION_H

#include "types.h"

typedef enum {
    RAY_TYPE_PRIMARY = 0,
    RAY_TYPE_SHADOW  = 1
} ray_type;

extern ray_type current_ray_type;

extern unsigned long long num_rays_shot;
extern unsigned long long num_shadow_rays_shot;
extern unsigned long long num_triangles_tested;
extern unsigned long long num_bboxes_tested;
extern unsigned long long num_grid_steps;

extern unsigned long long num_triangles_tested_primary;
extern unsigned long long num_triangles_tested_shadow;
extern unsigned long long num_bboxes_tested_primary;
extern unsigned long long num_bboxes_tested_shadow;
extern unsigned long long num_grid_steps_primary;
extern unsigned long long num_grid_steps_shadow;

int     find_first_intersection(intersection_point *ip,
            vec3 ray_origin, vec3 ray_direction);

int     shadow_check(vec3 ray_origin, vec3 ray_direction);

int     ray_intersects_triangle(intersection_point* ip, triangle tri,
            vec3 ray_origin, vec3 ray_direction);

void stats_count_triangle_test(void);
void stats_count_bbox_test(void);
void stats_count_grid_step(void);

#endif
