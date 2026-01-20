#ifndef TEXTURE_H
#define TEXTURE_H

typedef struct Texture {
    int w, h;
    unsigned char* data; // RGB, row-major, 3 bytes per pixel
} Texture;

// Loads PPM P6 (binary) or P3 (ASCII). Returns 1 on success, 0 on failure.
int  tex_load_ppm(Texture* tex, const char* filename);

// Frees tex->data and resets fields.
void tex_free(Texture* tex);

// Sample with UV in [0,1]. Wraps in U, clamps in V. Returns RGB in [0,1].
void tex_sample_rgb(const Texture* tex, float u, float v, float* r, float* g, float* b);

// Convenience: luminance in [0,1].
float tex_sample_luma(const Texture* tex, float u, float v);

#endif
