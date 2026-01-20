#define _CRT_SECURE_NO_WARNINGS
#include "texture.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

static int read_token(FILE* f, char* out, int maxLen)
{
    int c;

    // skip whitespace + comments
    while (1) {
        c = fgetc(f);
        if (c == EOF) return 0;
        if (isspace(c)) continue;
        if (c == '#') { // comment line
            while ((c = fgetc(f)) != EOF && c != '\n') {}
            continue;
        }
        break;
    }

    int i = 0;
    out[i++] = (char)c;
    while (i < maxLen - 1) {
        c = fgetc(f);
        if (c == EOF || isspace(c) || c == '#') {
            if (c == '#') {
                while ((c = fgetc(f)) != EOF && c != '\n') {}
            }
            break;
        }
        out[i++] = (char)c;
    }
    out[i] = 0;
    return 1;
}

int tex_load_ppm(Texture* tex, const char* filename)
{
    memset(tex, 0, sizeof(*tex));

    FILE* f = fopen(filename, "rb");
    if (!f) return 0;

    char tok[256];
    if (!read_token(f, tok, sizeof(tok))) { fclose(f); return 0; }

    int isP6 = (strcmp(tok, "P6") == 0);
    int isP3 = (strcmp(tok, "P3") == 0);
    if (!isP6 && !isP3) { fclose(f); return 0; }

    if (!read_token(f, tok, sizeof(tok))) { fclose(f); return 0; }
    tex->w = atoi(tok);
    if (!read_token(f, tok, sizeof(tok))) { fclose(f); return 0; }
    tex->h = atoi(tok);
    if (tex->w <= 0 || tex->h <= 0) { fclose(f); return 0; }

    if (!read_token(f, tok, sizeof(tok))) { fclose(f); return 0; }
    int maxv = atoi(tok);
    if (maxv <= 0) { fclose(f); return 0; }

    tex->data = (unsigned char*)malloc((size_t)tex->w * (size_t)tex->h * 3);
    if (!tex->data) { fclose(f); return 0; }

    if (isP6) {
        // Consume a single whitespace after header if present
        int c = fgetc(f);
        if (c != EOF && !isspace(c)) ungetc(c, f);

        size_t want = (size_t)tex->w * (size_t)tex->h * 3;
        size_t got = fread(tex->data, 1, want, f);
        fclose(f);

        if (got != want) { tex_free(tex); return 0; }

        // If maxv != 255, normalize by scaling
        if (maxv != 255) {
            for (size_t i = 0; i < want; i++) {
                float v = tex->data[i] / 255.0f;
                v = v * (255.0f / (float)maxv);
                if (v < 0) v = 0; if (v > 1) v = 1;
                tex->data[i] = (unsigned char)lrintf(v * 255.0f);
            }
        }
        return 1;
    }

    for (int i = 0; i < tex->w * tex->h * 3; i++) {
        if (!read_token(f, tok, sizeof(tok))) { fclose(f); tex_free(tex); return 0; }
        int v = atoi(tok);
        if (v < 0) v = 0;
        if (v > maxv) v = maxv;
        tex->data[i] = (unsigned char)lrintf((float)v * 255.0f / (float)maxv);
    }

    fclose(f);
    return 1;
}

void tex_free(Texture* tex)
{
    if (tex->data) free(tex->data);
    tex->data = NULL;
    tex->w = tex->h = 0;
}

static float wrap01(float x)
{
    x = x - floorf(x);
    if (x < 0.0f) x += 1.0f;
    return x;
}

void tex_sample_rgb(const Texture* tex, float u, float v, float* r, float* g, float* b)
{
    if (!tex || !tex->data || tex->w <= 0 || tex->h <= 0) {
        *r = *g = *b = 0.0f;
        return;
    }

    u = wrap01(u);
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;

    int x = (int)floorf(u * (tex->w - 1));
    int y = (int)floorf(v * (tex->h - 1));

    int idx = (y * tex->w + x) * 3;
    *r = tex->data[idx + 0] / 255.0f;
    *g = tex->data[idx + 1] / 255.0f;
    *b = tex->data[idx + 2] / 255.0f;
}

float tex_sample_luma(const Texture* tex, float u, float v)
{
    float r, g, b;
    tex_sample_rgb(tex, u, v, &r, &g, &b);
    return 0.2126f * r + 0.7152f * g + 0.0722f * b;
}
