#ifndef SCENE_H
#define SCENE_H

#include "types.h"
#include "texture.h"

extern int      scene_num_triangles, scene_num_vertices;
extern triangle *scene_triangles;
extern vec3     *scene_vertices;

extern int     	scene_num_spheres;
extern sphere   *scene_spheres;

extern int      scene_num_lights;
extern light    *scene_lights;

extern float    scene_ambient_light;
extern vec3    	scene_background_color;

extern vec3     scene_camera_position;
extern vec3     scene_camera_lookat;

extern int      scene_frost_has_mask;
extern Texture  scene_frost_mask;

void		    read_scene(const char *fname);

#endif
