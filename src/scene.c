#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "scene.h"
#include "plymodel.h"
#include "bvh.h"
#include "grid.h"
#include "octree.h"

int         scene_num_triangles, scene_num_vertices;
triangle    *scene_triangles = NULL;
vec3        *scene_vertices  = NULL;

int         scene_num_spheres;
sphere      *scene_spheres   = NULL;

int         scene_num_lights;
light       *scene_lights    = NULL;

float       scene_ambient_light = 0.05f;
vec3        scene_background_color = { 1.0f, 1.0f, 1.0f };

vec3        scene_camera_position, scene_camera_lookat;

int         scene_frost_has_mask = 0;
Texture     scene_frost_mask;

static void scene_frost_mask_reset(void)
{
    tex_free(&scene_frost_mask);
    memset(&scene_frost_mask, 0, sizeof(scene_frost_mask));
    scene_frost_has_mask = 0;
}

void
read_scene(const char *fname)
{
    FILE    *f;
    char    line[256];
    int     i;
    float   x, y, z, r;
    int     cur_material = 0;

    f = fopen(fname, "rt");
    if (!f)
    {
        fprintf(stderr, "\nFATAL: Could not open scene file %s!\n", fname);
        exit(-1);
    }

    printf("Reading scene %s\n", fname);

    /* Reset scene counts */
    scene_num_triangles = scene_num_vertices = 0;
    scene_num_spheres = 0;
    scene_num_lights  = 0;

    /* Reset frosted mask each scene load */
    scene_frost_mask_reset();

    /* Bare bones, but usable, parsing */
    fgets(line, 255, f);

    while (!feof(f))
    {
        if (line[0] == '/' && line[1] == '/')
        {
            /* Comment, skip */
        }
        else
        {
            int  mtype = -1;
            char mask_path[256];

            /*
              Material parsing (simple system):
                material <type>
                material <type> mask <ppm_path>
                material <type> <ppm_path>   (shorthand for mask)
            */
            if (sscanf(line, "material %d mask %255s", &mtype, mask_path) == 2)
            {
                cur_material = mtype;

                if (mtype == 4)
                {
                    scene_frost_mask_reset();
                    scene_frost_has_mask = tex_load_ppm(&scene_frost_mask, mask_path);
                    if (!scene_frost_has_mask)
                        fprintf(stderr, "WARNING: could not load frosted mask '%s'\n", mask_path);
                }
            }
            else if (sscanf(line, "material %d %255s", &mtype, mask_path) == 2)
            {
                cur_material = mtype;

                if (mtype == 4)
                {
                    scene_frost_mask_reset();
                    scene_frost_has_mask = tex_load_ppm(&scene_frost_mask, mask_path);
                    if (!scene_frost_has_mask)
                        fprintf(stderr, "WARNING: could not load frosted mask '%s'\n", mask_path);
                }
            }
            else if (sscanf(line, "material %d", &mtype) == 1)
            {
                cur_material = mtype;

                /* If user sets material 4 with no mask, enforce uniform roughness */
                if (mtype == 4)
                    scene_frost_mask_reset();
            }
            else if (sscanf(line, "light %f %f %f %f\n", &x, &y, &z, &r) == 4)
            {
                scene_lights = realloc(scene_lights, (scene_num_lights + 1) * sizeof(light));
                scene_lights[scene_num_lights].position.x = x;
                scene_lights[scene_num_lights].position.y = y;
                scene_lights[scene_num_lights].position.z = z;
                scene_lights[scene_num_lights].intensity = r;
                scene_num_lights++;
            }
            else if (sscanf(line, "sphere %f %f %f %f\n", &x, &y, &z, &r) == 4)
            {
                scene_spheres = realloc(scene_spheres, (scene_num_spheres + 1) * sizeof(sphere));
                scene_spheres[scene_num_spheres].center.x = x;
                scene_spheres[scene_num_spheres].center.y = y;
                scene_spheres[scene_num_spheres].center.z = z;
                scene_spheres[scene_num_spheres].radius = r;
                scene_spheres[scene_num_spheres].material = cur_material;
                scene_num_spheres++;
            }
            else if (strncmp(line, "ply_file", 8) == 0)
            {
                char  filename[256];
                float sx = 1.0f, sy = 1.0f, sz = 1.0f;
                float tx = 0.0f, ty = 0.0f, tz = 0.0f;

                /*
                  Supported formats:
                    ply_file path
                    ply_file path sx sy sz tx ty tz
                */
                int num_read = sscanf(line, "ply_file %255s %f %f %f %f %f %f",
                                      filename, &sx, &sy, &sz, &tx, &ty, &tz);

                if (num_read >= 1)
                {
                    printf("[%s] (scale %.3f %.3f %.3f, translate %.3f %.3f %.3f)\n",
                           filename, sx, sy, sz, tx, ty, tz);

                    read_ply_model(filename);

                    /* Apply transform to the PLY vertices before copying them */
                    for (i = 0; i < ply_num_vertices; i++)
                    {
                        vec3 p = ply_vertices[i];
                        p.x = sx * p.x + tx;
                        p.y = sy * p.y + ty;
                        p.z = sz * p.z + tz;
                        ply_vertices[i] = p;
                    }

                    scene_vertices = realloc(scene_vertices,
                        (scene_num_vertices + ply_num_vertices) * sizeof(vec3));
                    scene_triangles = realloc(scene_triangles,
                        (scene_num_triangles + ply_num_triangles) * sizeof(triangle));

                    memcpy(scene_vertices + scene_num_vertices,
                        ply_vertices, ply_num_vertices * sizeof(vec3));

                    for (i = 0; i < ply_num_triangles; i++)
                    {
                        scene_triangles[scene_num_triangles + i] = ply_triangles[i];

                        scene_triangles[scene_num_triangles + i].v[0] += scene_num_vertices;
                        scene_triangles[scene_num_triangles + i].v[1] += scene_num_vertices;
                        scene_triangles[scene_num_triangles + i].v[2] += scene_num_vertices;

                        scene_triangles[scene_num_triangles + i].material = cur_material;
                    }

                    scene_num_vertices += ply_num_vertices;
                    scene_num_triangles += ply_num_triangles;
                }
                else
                {
                    fprintf(stderr, "Ignoring malformed ply_file line '%s'\n", line);
                }
            }
        }

        fgets(line, 255, f);
    }

    fclose(f);

    bvh_build();
    grid_build();
    octree_build();
}
