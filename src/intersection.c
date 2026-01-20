#include <math.h>
#include <stdio.h>
#include "intersection.h"
#include "v3math.h"
#include "constants.h"
#include "scene.h"
#include "bvh.h"
#include "grid.h"
#include "octree.h"


// A few counters for gathering statistics on the number and types
// of ray shot

ray_type current_ray_type = RAY_TYPE_PRIMARY;

// The total number of rays
unsigned long long num_rays_shot = 0;

// Number of shadow rays
unsigned long long num_shadow_rays_shot = 0;

// Number of triangles tested for intersection
unsigned long long num_triangles_tested = 0;

// Number of bounding boxes tested for intersection
unsigned long long num_bboxes_tested = 0;

// Number of grid cells stepped through
unsigned long long num_grid_steps = 0;

unsigned long long num_triangles_tested_primary = 0;
unsigned long long num_triangles_tested_shadow  = 0;
unsigned long long num_bboxes_tested_primary    = 0;
unsigned long long num_bboxes_tested_shadow     = 0;
unsigned long long num_grid_steps_primary       = 0;
unsigned long long num_grid_steps_shadow        = 0;

// Forward declarations

static int  find_first_intersected_bvh_triangle(intersection_point* ip,
                vec3 ray_origin, vec3 ray_direction);

// Checks if the given triangle is intersected by ray with given
// origin and direction.
//
// Returns 1 if there is an intersection, or 0 otherwise.
//
// When an intersection is found the fields of 'ip' will be filled in
// with the relevant values.
//
// Note: this routine does NOT return an intersection for triangles
// whose back side faces the ray (by definition a triangle normal
// points to the triangle's front side).
// I.e. we do back-face culling here ...
//
// Code based on Moller & Trumbore, 1997, "Fast, minimum storage
// ray/triangle intersection"

int
ray_intersects_triangle(intersection_point* ip, triangle tri,
    vec3 ray_origin, vec3 ray_direction)
{
    vec3    edge1, edge2;
    vec3    tvec, pvec, qvec;
    double  det, inv_det;
    double  t, u, v;        // u, v are barycentric coordinates
    // t is ray parameter

    stats_count_triangle_test();

    edge1 = v3_subtract(scene_vertices[tri.v[1]], scene_vertices[tri.v[0]]);
    edge2 = v3_subtract(scene_vertices[tri.v[2]], scene_vertices[tri.v[0]]);

    pvec = v3_crossprod(ray_direction, edge2);

    det = v3_dotprod(edge1, pvec);

    if (det < 1.0e-6)
        return 0;

    tvec = v3_subtract(ray_origin, scene_vertices[tri.v[0]]);

    u = v3_dotprod(tvec, pvec);
    if (u < 0.0 || u > det)
        return 0;

    qvec = v3_crossprod(tvec, edge1);

    v = v3_dotprod(ray_direction, qvec);
    if (v < 0.0 || u+v > det)
        return 0;

    t = v3_dotprod(edge2, qvec);

    if (t < 0.0)
        return 0;

    inv_det = 1.0 / det;
    t *= inv_det;
    u *= inv_det;
    v *= inv_det;

    // We have a triangle intersection!
    // Return the relevant intersection values.

    // Compute the actual intersection point
    ip->t = t;
    ip->p = v3_add(ray_origin, v3_multiply(ray_direction, t));

    // Compute an interpolated normal for this intersection point, i.e.
    // we use the barycentric coordinates as weights for the vertex normals
    ip->n = v3_normalize(v3_add(
        v3_add(
            v3_multiply(tri.vn[0], 1.0-u-v),
            v3_multiply(tri.vn[1], u)
        ),
        v3_multiply(tri.vn[2], v)));

    ip->i = v3_normalize(v3_negate(ray_direction));
    ip->material = tri.material;

    return 1;
}

// Check if the given sphere is intersected by the given ray.
// See Shirley et.al., section 10.3.1
// Returns 1 if there is an intersection (and sets the appropriate
// fields of ip), or 0 otherwise.
static int
ray_intersects_sphere(intersection_point* ip, sphere sph,
    vec3 ray_origin, vec3 ray_direction)
{
    float   A, B, C, D;
    vec3    diff;
    float   t_hit;

    A = v3_dotprod(ray_direction, ray_direction);

    diff = v3_subtract(ray_origin, sph.center);
    B = 2.0 * v3_dotprod(diff, ray_direction);
    C = v3_dotprod(diff, diff) - sph.radius * sph.radius;

    D = B*B - 4*A*C;

    if (D < 0.0)
        return 0;

    D = sqrt(D);

    // We're only interested in the first hit, i.e. the one with
    // the smallest t_hit, so we check -B-D first, followed by -B+D

    t_hit = (-B - D)/(2*A);

    if (t_hit < 0.0)
    {
        t_hit = (-B + D)/(2*A);
        if (t_hit < 0.0)
            return 0;
    }

    ip->t = t_hit;
    ip->p = v3_add(ray_origin, v3_multiply(ray_direction, t_hit));
    ip->n = v3_normalize(v3_subtract(ip->p, sph.center));
    ip->i = v3_normalize(v3_negate(ray_direction));
    ip->material = sph.material;

    return 1;
}


// Recursively traverses the Bounding Volume Hierarchy (BVH) to find the closest intersection
// of a ray with any triangle in the scene.

static int traverse_bvh_recursive(bvh_node* node, vec3 ray_origin, vec3 ray_direction, float t0, float* t1, intersection_point* ip)
{
    if (!node) {
        return 0;
    }

    float t_min_bbox = 0.0f;
    float t_max_bbox = 0.0f;

    if (!bbox_intersect(&t_min_bbox, &t_max_bbox, node->bbox, ray_origin, ray_direction, t0, *t1)) {
        return 0;
    }

    // Clamp the intersection interval to [t0, *t1].
    if (t_min_bbox < t0) t_min_bbox = t0;
    if (t_max_bbox > *t1) t_max_bbox = *t1;

    // If the clamped interval is invalid, no intersection occurs within the valid t range.
    if (t_min_bbox > t_max_bbox) {
        return 0;
    }

    if (node->is_leaf) {
        // Leaf node: check all contained triangles for intersection.
        for (int i = 0; i < node->u.leaf.num_triangles; i++) {
            triangle tri = node->u.leaf.triangles[i];
            intersection_point ip_temp;

            // Test ray against the current triangle.
            if (ray_intersects_triangle(&ip_temp, tri, ray_origin, ray_direction)) {
                if (ip_temp.t >= t0 && ip_temp.t < *t1) {
                    *ip = ip_temp;
                    *t1 = ip_temp.t;
                    return 1;
                }
            }
        }
    } else {
        // Inner node: recursively traverse child nodes.
        int hit_left = traverse_bvh_recursive(node->u.inner.left_child, ray_origin, ray_direction, t0, t1, ip);
        int hit_right = traverse_bvh_recursive(node->u.inner.right_child, ray_origin, ray_direction, t0, t1, ip);
        return (hit_left || hit_right); // Return true if any child node reports an intersection.
    }

    // No intersection found in this branch.
    return 0;
}


//Initiates the BVH traversal to find the first triangle intersected by a ray.

static int
find_first_intersected_bvh_triangle(intersection_point *ip,
    vec3 ray_origin, vec3 ray_direction)
{
    float t0 = 0.0f;
    float t1 = C_INFINITY;

    return traverse_bvh_recursive(bvh_root, ray_origin, ray_direction, t0, &t1, ip);
}

// Returns the nearest hit of the given ray with objects in the scene
// (either a sphere or a triangle).
//
// Returns 1 and sets the intersection point values if there
// is an intersection, returns 0 otherwise.
int
find_first_intersection(intersection_point *ip, vec3 ray_origin, vec3 ray_direction)
{
    int     have_hit;
    float   t_nearest;
    intersection_point  ip2;

    num_rays_shot++;

    // We have found no hit yet
    t_nearest = C_INFINITY;
    have_hit = 0;

    // First check against spheres in the scene
    for (int s = 0; s < scene_num_spheres; s++)
    {
        // We need a second set of p and n variables, as there's the
        // possibility that we'll overwrite a closer intersection already
        // found
        if (ray_intersects_sphere(&ip2, scene_spheres[s], ray_origin, ray_direction))
        {
            if (ip2.t < t_nearest)
            {
                *ip = ip2;
                t_nearest = ip2.t;
                have_hit = 1;
            }
        }
    }

    // Then check against triangles in the scene


    if (use_octree) {
        if (octree_first_intersection(&ip2, ray_origin, ray_direction)) {
            if (ip2.t < t_nearest) {
                *ip = ip2;
                t_nearest = ip2.t;
                have_hit = 1;
            }
        }
    }
    else if (use_grid) {
        if (grid_first_intersection(&ip2, ray_origin, ray_direction)) {
            if (ip2.t < t_nearest) {
                *ip = ip2;
                t_nearest = ip2.t;
                have_hit = 1;
            }
        }
    }
    else if (use_bvh)
    {
        // Use the BVH to speed up intersection testing
        if (find_first_intersected_bvh_triangle(&ip2, ray_origin, ray_direction))
        {
            if (ip2.t < t_nearest)
            {
                *ip = ip2;
                t_nearest = ip2.t;
                have_hit = 1;
            }
        }
    }
    else
    {
        // Simply iterate over all the triangles in the scene and check for intersection
        for (int t = 0; t < scene_num_triangles; t++)
        {
            if (ray_intersects_triangle(&ip2, scene_triangles[t], ray_origin, ray_direction))
            {
                if (ip2.t < t_nearest)
                {
                    *ip = ip2;
                    t_nearest = ip2.t;
                    have_hit = 1;
                }
            }
        }
    }

    return have_hit;
}

// Optimized routine for tracing a shadow ray.
//
// This routine doesn't return the nearest intersection, but simply
// checks if there is any intersection.
int
shadow_check(vec3 ray_origin, vec3 ray_direction)
{
    intersection_point  ip;

    ray_type prev_type = current_ray_type;
    current_ray_type = RAY_TYPE_SHADOW;

    num_rays_shot++;
    num_shadow_rays_shot++;

    for (int s = 0; s < scene_num_spheres; s++)
    {
        if (ray_intersects_sphere(&ip, scene_spheres[s], ray_origin, ray_direction)) {
            current_ray_type = prev_type;
            return 1;
        }
    }

    if (use_octree) {
        if (octree_shadow_check(ray_origin, ray_direction)) {
            current_ray_type = prev_type;
            return 1;
        }
    }
    else if (use_grid) {
        if (grid_shadow_check(ray_origin, ray_direction)) {
            current_ray_type = prev_type;
            return 1;
        }
    }
    else if (use_bvh)
    {
        // Use the BVH for speedy intersection testing
        if (find_first_intersected_bvh_triangle(&ip, ray_origin, ray_direction)) {
            current_ray_type = prev_type;
            return 1;
        }
    }
    else
    {
        // Simply iterate over all the triangles in the scene and check for intersection
        for (int t = 0; t < scene_num_triangles; t++)
        {
            if (ray_intersects_triangle(&ip, scene_triangles[t], ray_origin, ray_direction)) {
                current_ray_type = prev_type;
                return 1;
            }
        }
    }
    current_ray_type = prev_type;
    return 0;
}

void
stats_count_triangle_test(void)
{
    num_triangles_tested++;
    if (current_ray_type == RAY_TYPE_SHADOW)
        num_triangles_tested_shadow++;
    else
        num_triangles_tested_primary++;
    return;
}

void
stats_count_bbox_test(void)
{
    num_bboxes_tested++;
    if (current_ray_type == RAY_TYPE_SHADOW)
        num_bboxes_tested_shadow++;
    else
        num_bboxes_tested_primary++;
    return;
}

void
stats_count_grid_step(void)
{
    num_grid_steps++;
    if (current_ray_type == RAY_TYPE_SHADOW)
        num_grid_steps_shadow++;
    else
        num_grid_steps_primary++;
    return;

}