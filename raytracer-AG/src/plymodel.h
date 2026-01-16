#ifndef PLYMODEL_H
#define PLYMODEL_H

#include "types.h"

extern int         ply_num_triangles, ply_num_vertices, ply_num_normals;

extern triangle    *ply_triangles;
extern vec3        *ply_vertices;
extern vec3        *ply_normals;

void        read_ply_model(const char *fname);

#endif
