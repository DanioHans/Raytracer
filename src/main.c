#define _CRT_SECURE_NO_WARNINGS


#ifdef _WIN32
#include <time.h>
#else
#include <sys/time.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifdef __APPLE__
#include <OpenGL/glew.h>
#include <GLUT/glut.h>
#else
#include <GL/glew.h>
#include <GL/glut.h>
#endif

#include "plymodel.h"
#include "v3math.h"
#include "shaders.h"
#include "perlin.h"
#include "intersection.h"
#include "scene.h"
#include "shaders.h"
#include "constants.h"
#include "bvh.h"
#include "grid.h"
#include "octree.h"
#include "texture.h"

// Number of drawable pixels, i.e. x coordinates passed to PutPixel()
// should be in the range [0, framebuffer_width[.  Analogous for y.
// (These values must both be a power of 2)
const int   framebuffer_width = 1024;
const int   framebuffer_height = 1024;

// Camera vertical field-of-view
const float VFOV = 45.0;

byte    *framebuffer;

GLuint vertBuffers[5];
GLuint indBuffers[3];
GLuint normBuffers[2];
GLuint texBuffers[1];
GLuint texCoords[1];

int strips = 12;

int     show_raytraced=0;
int     needs_rerender=1;
int     show_bvh=0;
int     draw_bvh_mode=0;
int     show_normals=0;
int     do_antialiasing=0;

float   camDistance = 6.5;
float   camRotZ = 25.0, camAzimuth = -40.0;
float   saved_camRotZ, saved_camAzimuth, saved_camDistance;
int     mouse_mode = 0;
int     mx, my;

viewpoint   viewpoints[6] =
{
    { -4.8, -13.2, 2.9 },
    { -27.5, -30.5, 2.0 },
    { -73.8, 37.0, 3.8 },
    {  46.2, 0.0, 6.2 },
    { -35.0, -187.2, 2.8 },
    { -80.3, 27.0, 1.1 },
};

GLfloat cubeVert[24] =
{
    -0.5, -0.5, -0.5,
    0.5, -0.5, -0.5,
    0.5, 0.5, -0.5,
    -0.5, 0.5, -0.5,
    -0.5, -0.5, 0.5,
    0.5, -0.5, 0.5,
    0.5, 0.5, 0.5,
    -0.5, 0.5, 0.5
};

GLuint cubeInd[24] = {
    0, 1,
    1, 2,
    2, 3,
    3, 0,
    0, 4,
    1, 5,
    2, 6,
    3, 7,
    4, 5,
    5, 6,
    6, 7,
    7, 4
};

GLfloat quadVert[8] = {
    0.0, 1.0,
    0.0, 0.0,
    1.0, 0.0,
    1.0, 1.0
};

GLfloat quadTex[8] = {
    0.0, 0.0,
    0.0, 1.0,
    1.0, 1.0,
    1.0, 0.0
};

GLuint quadInd[6] = {
    0, 1, 2,
    0, 2, 3
};


#define AA_SPP 512

static inline unsigned int xorshift32(unsigned int* state)
{
    unsigned int x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

static inline float rand01(unsigned int* state)
{
    // [0, 1)
    return (xorshift32(state) & 0x00FFFFFF) / 16777216.0f;
}

/* Do not check for memory leaks when using AddressSanitizer. */
const char *
__asan_default_options(void)
{
    return "detect_leaks=0";
}

void
init_opengl(void)
{
    glewInit();

    // Set the background color
    glClearColor(1, 1, 1, 0);

    // Setup lighting parameters.
    GLfloat light_ambient[] = {0.4,0.4,0.4,0.0};
    GLfloat mat_shininess[] = { 50.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);

    // Generate required buffers.
    glGenBuffers(5, vertBuffers);
    glGenBuffers(2, normBuffers);
    glGenBuffers(3, indBuffers);
    glGenBuffers(1, texCoords);
    glGenTextures(1, texBuffers);

    // Setup buffers for wireframe cubes.
    glBindBuffer(GL_ARRAY_BUFFER, vertBuffers[0]);
    glBufferData(GL_ARRAY_BUFFER, 24 * sizeof(GLfloat), cubeVert, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indBuffers[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 24 * sizeof(GLuint), cubeInd, GL_STATIC_DRAW);

    // Build triangle vertices and normal buffers.
    GLfloat *triVert = malloc(9 * scene_num_triangles * sizeof(GLfloat));
    GLfloat *triNorm = malloc(9 * scene_num_triangles * sizeof(GLfloat));
    GLfloat *normVert = malloc(18 * scene_num_triangles * sizeof(GLfloat));
    triangle tri;
    vec3 addNorm;
    for (int i = 0; i < scene_num_triangles; i++) {
        tri = scene_triangles[i];
        for (int j = 0; j < 3; j++) {
            // Triangle Vertices.
            triVert[9 * i + j * 3] = scene_vertices[tri.v[j]].x;
            triVert[9 * i + j * 3 + 1] = scene_vertices[tri.v[j]].y;
            triVert[9 * i + j * 3 + 2] = scene_vertices[tri.v[j]].z;

            // Triangle Normals.
            triNorm[9 * i + j * 3] = tri.vn[j].x;
            triNorm[9 * i + j * 3 + 1] = tri.vn[j].y;
            triNorm[9 * i + j * 3 + 2] = tri.vn[j].z;

            // For drawing the normals.
            normVert[18 * i + j * 6] = scene_vertices[tri.v[j]].x;
            normVert[18 * i + j * 6 + 1] = scene_vertices[tri.v[j]].y;
            normVert[18 * i + j * 6 + 2] = scene_vertices[tri.v[j]].z;

            addNorm = v3_add(scene_vertices[tri.v[j]], v3_multiply(tri.vn[j], 0.15));

            normVert[18 * i + j * 6 + 3] = addNorm.x;
            normVert[18 * i + j * 6 + 4] = addNorm.y;
            normVert[18 * i + j * 6 + 5] = addNorm.z;
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, vertBuffers[1]);
    glBufferData(GL_ARRAY_BUFFER, 9 * scene_num_triangles * sizeof(GLfloat),
                 triVert, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, normBuffers[0]);
    glBufferData(GL_ARRAY_BUFFER, 9 * scene_num_triangles * sizeof(GLfloat),
                 triNorm, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, vertBuffers[2]);
    glBufferData(GL_ARRAY_BUFFER, 18 * scene_num_triangles * sizeof(GLfloat),
                 normVert, GL_STATIC_DRAW);

    // Buffers are now on the GPU, so they can be freed.
    free(triVert);
    free(triNorm);
    free(normVert);

    // Create sphere buffers.
    float phi;
    float theta;
    int vertNormOffset;
    int indOffset;
    int vertIndex;
    int arrayIndex;
    int vertexStart;
    sphere s;
    // All spheres are in the same buffer.
    GLfloat *sphereVert = malloc(scene_num_spheres * (strips + 1) * strips * 3
                                 * sizeof(GLfloat));
    GLfloat *sphereNorm = malloc(scene_num_spheres * (strips + 1) * strips * 3
                                 * sizeof(GLfloat));
    GLuint *sphereInd = malloc(scene_num_spheres * strips * strips * 6
                               * sizeof(GLuint));
    for (int n = 0; n < scene_num_spheres; n++) {
        s = scene_spheres[n];
        // Determine array starting points for this sphere.
        vertNormOffset = n * (strips + 1) * strips * 3;
        indOffset = n * strips * strips * 6;
        vertIndex = n * (strips + 1) * strips;
        for (int i = 0; i <= strips; i++) {
            phi = i * (M_PI / strips);
            vertexStart = vertNormOffset + i * strips * 3;
            for (int j = 0; j < strips; j++) {
                theta = j * (2 * M_PI / strips);
                // Calculate vertex position.
                sphereVert[vertexStart] = s.radius * cos(theta) * sin(phi) + s.center.x;
                sphereVert[vertexStart + 1] = s.radius * sin(theta) * sin(phi) + s.center.y;
                sphereVert[vertexStart + 2] = s.radius * cos(phi) + s.center.z;

                // Calculate vertex normals.
                sphereNorm[vertexStart] = cos(theta) * sin(phi);
                sphereNorm[vertexStart + 1] = sin(theta) * sin(phi);
                sphereNorm[vertexStart + 2] = cos(phi);
                vertexStart += 3;

                // Create the indices for the various triangles.
                // 2 triangles per new vertex, one to the next vertex and one
                // to the previous vertex.
                if (i > 0) {
                    arrayIndex = indOffset + (i - 1) * strips * 6 + j * 6;
                    // C doesn't do modulo right with negative numbers,
                    // so + strips is used.
                    sphereInd[arrayIndex] = vertIndex + i * strips +
                                           (j - 1 + strips) % strips;
                    sphereInd[arrayIndex + 1] = vertIndex + i * strips + j;
                    sphereInd[arrayIndex + 2] = vertIndex + (i - 1) * strips + j;
                    sphereInd[arrayIndex + 3] = vertIndex + i * strips + j;
                    sphereInd[arrayIndex + 4] = vertIndex + (i - 1) * strips + (j + 1) % strips;
                    sphereInd[arrayIndex + 5] = vertIndex + (i - 1) * strips + j;
                }
            }
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, vertBuffers[3]);
    glBufferData(GL_ARRAY_BUFFER,
                 scene_num_spheres * (strips + 1) * strips * 3 * sizeof(GLfloat),
                 sphereVert, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, normBuffers[1]);
    glBufferData(GL_ARRAY_BUFFER,
                 scene_num_spheres * (strips + 1) * strips * 3 * sizeof(GLfloat),
                 sphereNorm, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indBuffers[1]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 scene_num_spheres * strips * strips * 6 * sizeof(GLuint),
                 sphereInd, GL_STATIC_DRAW);

    // Buffers are on the GPU now so they can be freed.
    free(sphereVert);
    free(sphereNorm);
    free(sphereInd);

    // Raytraced quad.
    quadVert[1] = framebuffer_height;
    quadVert[4] = framebuffer_width;
    quadVert[7] = framebuffer_height;
    quadVert[6] = framebuffer_width;
    glBindBuffer(GL_ARRAY_BUFFER, vertBuffers[4]);
    glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(GLfloat), quadVert, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, texCoords[0]);
    glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(GLfloat), quadTex, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indBuffers[2]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(GLuint), quadInd, GL_STATIC_DRAW);

    // Allocate a framebuffer, to be filled during ray tracing
    framebuffer = (byte*) malloc(framebuffer_width*framebuffer_height*3);

    glBindTexture(GL_TEXTURE_2D, texBuffers[0]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, framebuffer_width, framebuffer_height,
                 0, GL_RGB, GL_UNSIGNED_BYTE, framebuffer);

    // Setup texturing state (as we display the framebuffer
    // using a textured quad)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    // Unbind the buffers.
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void
resize(int w, int h)
{
    if (h == 0)
        h = 1;

    glViewport(0, 0, w, h);
}

void
put_pixel(int x, int y, float r, float g, float b)
{
    if (x < 0 || y < 0 || x >= framebuffer_width || y >= framebuffer_height)
    {
        printf("put_pixel(): x, y coordinates (%d, %d) outside of visible area!\n",
                x, y);
        return;
    }

    // The pixels in framebuffer[] are layed out sequentially,
    // with the R, G and B values one after the each, e.g
    // RGBRGBRGB...

    framebuffer[3*(framebuffer_width*y+x)] = (int)(255*r);
    framebuffer[3*(framebuffer_width*y+x)+1] = (int)(255*g);
    framebuffer[3*(framebuffer_width*y+x)+2] = (int)(255*b);
}

void
setup_camera(void)
{
    float	cx, cy, cz;
    float	t;
    float 	beta, gamma;

    // degrees -> radians
    beta = camAzimuth / 180.0 * M_PI;
    gamma = camRotZ / 180.0 * M_PI;

    cx = camDistance;
    cy = cz = 0.0;

    // Rotate around Y
    t = cx;
    cx = cx * cos(beta) + cz * sin(beta);
	// cy remains unchanged
    cz = -t * sin(beta) + cz * cos(beta);

    // Rotate around Z
    t = cx;
    cx = cx * cos(gamma) - cy * sin(gamma);
    cy = t * sin(gamma) + cy * cos(gamma);
	// cz remains unchanged

    scene_camera_position.x = cx;
    scene_camera_position.y = cy;
    scene_camera_position.z = cz;

    scene_camera_lookat.x = 0.0;
    scene_camera_lookat.y = 0.0;
    scene_camera_lookat.z = 0.5;

    // Assumes matrix mode is model-view
    glLoadIdentity();
    gluLookAt (cx, cy, cz,  0.0, 0.0, 0.5,  0.0, 0.0, 1.0);
}


vec3 calc_color_vec(vec3 right, vec3 up, vec3 forward, float x_offset, float y_offset) {

    vec3 forward_move = v3_add(scene_camera_position, forward);
    vec3 right_offset = v3_multiply(right, x_offset);
    vec3 up_offset = v3_multiply(up, y_offset);

    vec3 pixel_center = v3_add(v3_add(forward_move, right_offset), up_offset);

    vec3 ray_direction = v3_normalize(v3_subtract(pixel_center, scene_camera_position));

    return ray_color(0, scene_camera_position, ray_direction);
}

void
ray_trace(void)
{
    vec3    forward_vector, right_vector, up_vector;
    int     i, j;
    float   image_plane_width, image_plane_height;
    vec3    color;
    char    buf[128];

    float           time_taken;

    fprintf(stderr, "Ray tracing ...");
    clock_t t0 = clock();

    num_rays_shot = 0;
    num_shadow_rays_shot = 0;
    num_triangles_tested = 0;
    num_bboxes_tested = 0;
    num_grid_steps = 0;

    num_triangles_tested_primary = 0;
    num_triangles_tested_shadow  = 0;
    num_bboxes_tested_primary    = 0;
    num_bboxes_tested_shadow     = 0;
    num_grid_steps_primary       = 0;
    num_grid_steps_shadow        = 0;

    current_ray_type = RAY_TYPE_PRIMARY;

    // Compute camera coordinate system from camera position
    // and look-at point
    up_vector = v3_create(0, 0, 1);
    forward_vector = v3_normalize(v3_subtract(scene_camera_lookat, scene_camera_position));
    right_vector = v3_normalize(v3_crossprod(forward_vector, up_vector));
    up_vector = v3_crossprod(forward_vector, right_vector);

    // Compute size of image plane from the chosen field-of-view
    // and image aspect ratio. This is the size of the plane at distance
    // of one unit from the camera position.
    image_plane_height = 2.0 * tan(0.5*VFOV/180*M_PI);
    image_plane_width = image_plane_height * (1.0 * framebuffer_width / framebuffer_height);

    // ...
    float height_factor = image_plane_height / framebuffer_height;
    float width_factor = image_plane_width / framebuffer_width;
    // ...

    // Loop over all pixels in the framebuffer
    for (j = 0; j < framebuffer_height; j++)
    {
        for (i = 0; i < framebuffer_width; i++)
        {
            // Check if anti-aliasing is needed and execute appropriate code
            if (do_antialiasing) {
                // TEMP: high-spp jittered AA for perf testing
                unsigned int rng = (unsigned int)(i * 1973u + j * 9277u + 89173u); // simple per-pixel seed

                color = v3_create(0.0f, 0.0f, 0.0f);

                for (int s = 0; s < AA_SPP; s++) {
                    // jitter inside pixel: [-0.5, +0.5)
                    float jx = rand01(&rng) - 0.5f;
                    float jy = rand01(&rng) - 0.5f;

                    float x_offset = (i + 0.5f + jx - framebuffer_width / 2.0f) * width_factor;
                    float y_offset = (j + 0.5f + jy - framebuffer_height / 2.0f) * height_factor;

                    color = v3_add(color, calc_color_vec(right_vector, up_vector, forward_vector, x_offset, y_offset));
                }

                color = v3_multiply(color, 1.0f / (float)AA_SPP);
            } else {
                // Compute pixel center in image plane
                float x_offset = (i + 0.5f - framebuffer_width / 2.0f)*width_factor;
                float y_offset = (j + 0.5f - framebuffer_height / 2.0f)*height_factor;

                color = calc_color_vec(right_vector, up_vector, forward_vector, x_offset, y_offset);
            }

            // Output pixel color
            put_pixel(i, j, color.x, color.y, color.z);
        }

        sprintf(buf, "Ray-tracing ::: %.0f%% done", 100.0*j/framebuffer_height);
        glutSetWindowTitle(buf);
    }


    // Done!
	clock_t t1 = clock();

    glutSetWindowTitle("Ray-tracing ::: done");

    // Output some statistics
    time_taken = (double)(t1-t0) / CLOCKS_PER_SEC;

    fprintf(stderr, " done in %.1f seconds\n", time_taken);
    fprintf(stderr, "... %lld total rays shot, of which %d camera rays and "
            "%lld shadow rays\n", num_rays_shot,
            do_antialiasing ? 4*framebuffer_width*framebuffer_height :
                              framebuffer_width*framebuffer_height,
            num_shadow_rays_shot);
    fprintf(stderr, "... %lld triangles intersection tested "
            "(avg %.1f tri/ray)\n",
        num_triangles_tested, 1.0*num_triangles_tested/num_rays_shot);
    fprintf(stderr, "... %lld bboxes intersection tested (avg %.1f bbox/ray)\n",
        num_bboxes_tested, 1.0*num_bboxes_tested/num_rays_shot);
    fprintf(stderr, "... %lld grid cells stepped through (avg %.1f cells/ray)\n",
        num_grid_steps, 1.0*num_grid_steps/num_rays_shot);

    unsigned long long num_primary_rays = num_rays_shot - num_shadow_rays_shot;

    fprintf(stderr, "--- breakdown primary vs shadow ---\n");
    fprintf(stderr, "primary rays: %llu, shadow rays: %llu\n",
            num_primary_rays, num_shadow_rays_shot);

    if (num_primary_rays > 0) {
        fprintf(stderr, "primary: %llu triangle tests (avg %.1f tri/primary ray)\n",
                num_triangles_tested_primary,
                (double)num_triangles_tested_primary / num_primary_rays);
        fprintf(stderr, "primary: %llu bbox tests (avg %.1f bbox/primary ray)\n",
                num_bboxes_tested_primary,
                (double)num_bboxes_tested_primary / num_primary_rays);
        fprintf(stderr, "primary: %llu grid steps (avg %.1f cells/primary ray)\n",
                num_grid_steps_primary,
                (double)num_grid_steps_primary / num_primary_rays);
    }

    if (num_shadow_rays_shot > 0) {
        fprintf(stderr, "shadow: %llu triangle tests (avg %.1f tri/shadow ray)\n",
                num_triangles_tested_shadow,
                (double)num_triangles_tested_shadow / num_shadow_rays_shot);
        fprintf(stderr, "shadow: %llu bbox tests (avg %.1f bbox/shadow ray)\n",
                num_bboxes_tested_shadow,
                (double)num_bboxes_tested_shadow / num_shadow_rays_shot);
        fprintf(stderr, "shadow: %llu grid steps (avg %.1f cells/shadow ray)\n",
                num_grid_steps_shadow,
                (double)num_grid_steps_shadow / num_shadow_rays_shot);
    }
}

// Draw the node bboxes of the BVH, for inner nodes at a certain
// level in the tree

static void
draw_bvh_inner_nodes(int level, bvh_node* node)
{
    vec3    center, size;

    if (node->is_leaf)
        return;

    if (level == draw_bvh_mode)
    {
        center = v3_multiply(v3_add(node->bbox.min, node->bbox.max), 0.5);
        size = v3_subtract(node->bbox.max, node->bbox.min);

        glColor3f(1, 0, 0);
        glPushMatrix();
        glTranslatef(center.x, center.y, center.z);
        glScalef(size.x, size.y, size.z);

        // Draw wireframe cube.
        glDrawElements(GL_LINES, 24, GL_UNSIGNED_INT, 0);

        glPopMatrix();
    }
    else
    {
        draw_bvh_inner_nodes(level+1, node->u.inner.left_child);
        draw_bvh_inner_nodes(level+1, node->u.inner.right_child);
    }
}

// Draw leaf node bounding boxes

static void
draw_bvh_leaf_nodes(bvh_node* node)
{
    vec3    center, size;

    if (node->is_leaf)
    {
        center = v3_multiply(v3_add(node->bbox.min, node->bbox.max), 0.5);
        size = v3_subtract(node->bbox.max, node->bbox.min);

        glColor3f(0, 0, 1);
        glPushMatrix();
        glTranslatef(center.x, center.y, center.z);
        glScalef(size.x, size.y, size.z);

        // Draw wireframe cube.
        glDrawElements(GL_LINES, 24, GL_UNSIGNED_INT, 0);

        glPopMatrix();
    }
    else
    {
        draw_bvh_leaf_nodes(node->u.inner.left_child);
        draw_bvh_leaf_nodes(node->u.inner.right_child);
    }
}

void
draw_scene(void)
{
    // clear the draw buffer
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    if (show_raytraced)
    {
        if (needs_rerender)
        {
            // clear the framebuffer
            memset(framebuffer, 255, 3*framebuffer_width*framebuffer_height);

            // trace a new picture
            ray_trace();

            // Update texture buffer.
            glBindTexture(GL_TEXTURE_2D, texBuffers[0]);
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, framebuffer_width,
                            framebuffer_height, GL_RGB, GL_UNSIGNED_BYTE,
                            framebuffer);
            glBindTexture(GL_TEXTURE_2D, 0);

            needs_rerender = 0;
        }

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Draw the framebuffer using a textured quad

        glOrtho(0, framebuffer_width, 0, framebuffer_height, -1, 1);

        glDisable(GL_LIGHTING);
        glDisable(GL_CULL_FACE);
        glEnable(GL_TEXTURE_2D);

        glBindTexture(GL_TEXTURE_2D, texBuffers[0]);

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);

        // Set Texture coordinates.
        glBindBuffer(GL_ARRAY_BUFFER, texCoords[0]);
        glTexCoordPointer(2, GL_FLOAT, 0, 0);

        // Set Vertex coordinates.
        glBindBuffer(GL_ARRAY_BUFFER, vertBuffers[4]);
        glVertexPointer(2, GL_FLOAT, 0, 0);

        // Draw textured quad.
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indBuffers[2]);
        glColor3f(1, 1, 1);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    }
    else
    {
        // Draw scene using OpenGL

        //glutSetWindowTitle("OpenGL view");

        glMatrixMode(GL_PROJECTION);

        glLoadIdentity();
        gluPerspective(VFOV, 1.0*framebuffer_width/framebuffer_height, 0.1, 1000.0);

        glMatrixMode(GL_MODELVIEW);

        setup_camera();

        glEnable(GL_LIGHTING);
        glEnable(GL_CULL_FACE);
        glDisable(GL_TEXTURE_2D);

        // Try to set up the lighting to match the scene

        GLfloat v[4];

        v[0] = scene_ambient_light;
        v[1] = scene_ambient_light;
        v[2] = scene_ambient_light;
        v[3] = 1.0;
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, v);

        for (int l = 0; l < scene_num_lights; l++)
        {
            glEnable(GL_LIGHT0 + l);

            v[0] = scene_lights[l].position.x;
            v[1] = scene_lights[l].position.y;
            v[2] = scene_lights[l].position.z;
            v[3] = 1.0; // we want a positional light source
            glLightfv(GL_LIGHT0 + l, GL_POSITION, v);

            v[0] = v[1] = v[2] = v[3] = 0.0;
            glLightfv(GL_LIGHT0 + l, GL_AMBIENT, v);
            glLightfv(GL_LIGHT0 + l, GL_SPECULAR, v);

            v[0] = v[1] = v[2] = scene_lights[l].intensity;
            v[3] = 1.0;
            glLightfv(GL_LIGHT0 + l, GL_DIFFUSE, v);
        }

        GLfloat one[] = { 1.0, 1.0, 1.0, 1.0 };
        GLfloat zero[] = { 0.0, 0.0, 0.0, 1.0 };

        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, zero);
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, zero);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, one);
		// Draw the triangles in the scene with vertices and normals.
        glBindBuffer(GL_ARRAY_BUFFER, vertBuffers[1]);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, 0);

        glBindBuffer(GL_ARRAY_BUFFER, normBuffers[0]);
        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, 0, 0);

        // Draw the triangles.
        glDrawArrays(GL_TRIANGLES, 0, scene_num_triangles * 9);

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        if (show_normals)
        {
            // Draw vertex normals as red lines
            glDisable(GL_LIGHTING);
            glColor3f(1, 0, 0);

            glBindBuffer(GL_ARRAY_BUFFER, vertBuffers[2]);
            glEnableClientState(GL_VERTEX_ARRAY);
            glVertexPointer(3, GL_FLOAT, 0, 0);

            glDrawArrays(GL_LINES, 0, scene_num_triangles * 18);

            glDisableClientState(GL_VERTEX_ARRAY);
            glBindBuffer(GL_ARRAY_BUFFER, 0);

            glEnable(GL_LIGHTING);
        }

		// Draw the spheres in the scene

        glPushMatrix();

        glBindBuffer(GL_ARRAY_BUFFER, vertBuffers[3]);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, 0);

        glBindBuffer(GL_ARRAY_BUFFER, normBuffers[1]);
        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, 0, 0);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indBuffers[1]);

        glDrawElements(GL_TRIANGLES, scene_num_spheres * strips * strips * 6, GL_UNSIGNED_INT, 0);

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        glPopMatrix();

        // Show BVH node bboxes

        if (show_bvh)
        {
            glDisable(GL_LIGHTING);
            glBindBuffer(GL_ARRAY_BUFFER, vertBuffers[0]);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indBuffers[0]);
            glEnableClientState(GL_VERTEX_ARRAY);
            glVertexPointer(3, GL_FLOAT, 0, 0);

            if (draw_bvh_mode == 0)
                draw_bvh_leaf_nodes(bvh_root);
            else
                draw_bvh_inner_nodes(1, bvh_root);

            glDisableClientState(GL_VERTEX_ARRAY);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        }
    }

    // finally, swap the draw buffers to make the triangles appear on screen
    glutSwapBuffers();
}

void
save_image(void)
{
    FILE *f;

    f = fopen("image.ppm", "wt");
    if (!f)
    {
        fprintf(stderr, "Could not create image file\n");
        return;
    }

    fprintf(f, "P3\n# Raytraced image\n%d %d\n255\n", framebuffer_width, framebuffer_height);
    for (int i = 0; i < 3*framebuffer_width*framebuffer_height; i++)
        fprintf(f, "%d\n", framebuffer[i]);

    printf("Image saved to image.ppm\n");

    fclose(f);
}

void
key_pressed(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 'r':
        {
            // Toggle between OpenGL and ray-traced output
            show_raytraced = 1 - show_raytraced;
            if (show_raytraced)
                glutSetWindowTitle("Ray-tracing [ray-traced output]");
            else
                glutSetWindowTitle("Ray-tracing [OpenGL view]");
            glutPostRedisplay();
            break;
        }
        case 'a':
        {
            // Toggle anti-aliasing (forces immediate re-render)
            do_antialiasing = 1 - do_antialiasing;
            needs_rerender = 1;
            glutPostRedisplay();
            break;
        }
        case 'b':
        {
            // Toggle use of the BVH for intersection testing
            // (forces immediate re-render)
            use_bvh = 1 - use_bvh;
            if(use_bvh) {
                use_octree = 0;
                use_grid = 0;
                printf("use_grid set to %d\n", use_grid);
                printf("use_octree set to %d\n", use_octree);
            }
            printf("use_bvh set to %d\n", use_bvh);
            needs_rerender = 1;
            glutPostRedisplay();
            break;
        }
        case 'B':
        {
            // Show BVH nodes
            show_bvh = 1 - show_bvh;
            if (show_bvh)
                draw_bvh_mode = 0;
            glutPostRedisplay();
            break;
        }
        case 'g':
        {
            use_grid = 1 - use_grid;
            if(use_grid) {
                use_bvh = 0;
                use_octree = 0;
                printf("use_bvh set to %d\n", use_bvh);
                printf("use_octree set to %d\n", use_octree);
            }
            printf("use_grid set to %d\n", use_grid);
            needs_rerender = 1;
            glutPostRedisplay();
            break;
        }
        case 'o':
        {
            use_octree = !use_octree;
            if (use_octree) {
                use_bvh = 0;
                use_grid = 0;
                printf("use_bvh set to %d\n", use_bvh);
                printf("use_grid set to %d\n", use_grid);
            }
            printf("use_octree set to %d\n", use_octree);
            needs_rerender = 1;
            break;
        }
        case ']':
        {
            draw_bvh_mode++;
            glutPostRedisplay();
            break;
        }
        case '[':
        {
            draw_bvh_mode--;
            if (draw_bvh_mode < 0)
                draw_bvh_mode = 0;
            glutPostRedisplay();
            break;
        }
        case 'n':
        {
            // Show vertex normals
            show_normals = 1 - show_normals;
            glutPostRedisplay();
            break;
        }
        case 'c':
        {
            // Dump camera parameters
            printf("azimuth = %.1f, rot_z = %.1f, distance = %.1f\n",
                camAzimuth, camRotZ, camDistance);
            break;

        }
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        {
            // Switch to a predefined viewpoint
            int idx = (int)(key - '1');
            camAzimuth = viewpoints[idx].azimuth;
            camRotZ = viewpoints[idx].rot_z;
            camDistance = viewpoints[idx].distance;

            // Switch to OpenGL viewing
            show_raytraced = 0;

            // And since the camera params changed we need to rerender
            needs_rerender = 1;

            glutPostRedisplay();

            break;
        }
        case 's':
        {
            // Save rendered image to (ascii) .ppm file 'image.ppm'
            save_image();
            break;
        }
        case 'm':
        {
            // Display mouse coordinates (for debugging only)
            printf("x = %d, y = %d\n", x, y);
            break;
        }
        case 'q':
        {
            exit(0);
            break;
        }
        case 't':
        {
            use_striped_background = 1 - use_striped_background;
            break;
        }
    }
}

static void
mouse_func(int button, int state, int x, int y)
{
    if (show_raytraced)
        return;

    // guard against both left and right buttons being pressed at the same time,
    // by only responding when a mouse button is pressed while another one
    // hasn't been pressed yet
    if (state == GLUT_DOWN && mouse_mode == 0)
    {
        if (button == GLUT_LEFT_BUTTON)
        {
            mouse_mode = GLUT_LEFT_BUTTON;

            saved_camRotZ = camRotZ;
            saved_camAzimuth = camAzimuth;

            mx = x;
            my = y;
        }
        else if (button == GLUT_RIGHT_BUTTON)
        {
            mouse_mode = GLUT_RIGHT_BUTTON;

            saved_camDistance = camDistance;

            my = y;
        }
    }
    else if (state == GLUT_UP && button == mouse_mode)
    {
        // pressed button released
        mouse_mode = 0;
    }

}

static void
motion_func(int x, int y)
{
    int dx, dy;

    if (show_raytraced)
        return;

    if (mouse_mode == GLUT_LEFT_BUTTON)
    {
        dx = mx - x;
        dy = my - y;

        camRotZ = saved_camRotZ + dx * 0.25;
        camAzimuth = saved_camAzimuth + dy * 0.25;

        if (camAzimuth > 89.99)
            camAzimuth = 89.99;
        else if (camAzimuth < -89.99)
            camAzimuth = -89.99;

        needs_rerender = 1;
    }
    else if (mouse_mode == GLUT_RIGHT_BUTTON)
    {
        dy = my - y;

        camDistance = saved_camDistance - dy * 0.15;
        if (camDistance < 0.5)
            camDistance = 0.5;
        else if (camDistance > 100.0)
            camDistance = 100.0;

        needs_rerender = 1;
    }
}


int
main(int argc, char **argv)
{
    glutInit(&argc, argv);

    if (--argc != 1)
    {
        printf("Usage: %s file.scn\n", argv[0]);
        exit(-1);
    }

    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

    glutInitWindowSize(framebuffer_width, framebuffer_height);

    glutInitWindowPosition(20, 100);
    glutCreateWindow("Ray tracing");

    glutDisplayFunc(&draw_scene);
    glutIdleFunc(&draw_scene);
    glutReshapeFunc(&resize);
    //glutSpecialFunc(&specialKeyPressed);
    glutKeyboardFunc(&key_pressed);
    glutMouseFunc(&mouse_func);
    glutMotionFunc(&motion_func);

	read_scene(argv[1]);

    init_opengl();
    init_noise();

    glutMainLoop();

    return 1;
}
