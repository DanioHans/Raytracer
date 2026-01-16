#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "shaders.h"
#include "v3math.h"
#include "intersection.h"
#include "scene.h"
#include "texture.h"
#include "shaders.h"
#include "perlin.h"
#include "intersection.h"
#include "quat.h"
#include "constants.h"

int use_striped_background = 1;
int MAX_RAY_LEVEL = 5;

// Small math helpers
static inline float clamp01f(float x) { return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x); }

static inline float smoothstepf(float a, float b, float x)
{
    float t = (x - a) / (b - a);
    t = clamp01f(t);
    return t * t * (3.0f - 2.0f * t);
}


vec3
shade_constant(intersection_point ip)
{
    (void)ip;
    return v3_create(1, 0, 0);
}

// Internal helper for matte / phong
static float
calc_color_val(intersection_point ip, light l)
{
    // Get light vector
    vec3 li = v3_normalize(v3_subtract(l.position, ip.p));

    vec3 si = v3_add(ip.p, v3_multiply(ip.n, 0.001f));

    // Check if overshadowed
    if (shadow_check(si, li))
        return 0.0f;

    return fmaxf(0.0f, v3_dotprod(ip.n, li)) * l.intensity;
}


vec3
shade_matte(intersection_point ip)
{
    float sum = scene_ambient_light;

    // Sum all light rays
    for (int i = 0; i < scene_num_lights; i++)
        sum += calc_color_val(ip, scene_lights[i]);

    // Return grayscale matte
    return v3_create(sum, sum, sum);
}


vec3
shade_blinn_phong(intersection_point ip)
{
    // Material properties
    float kd = 0.7f;
    float ks = 0.3f;
    float alpha = 20.0f;

    vec3 cd = v3_create(1, 1, 1);
    vec3 cs = v3_create(1, 1, 1);

    float ssum = 0.0f;
    float dsum = 0.0f;

    for (int i = 0; i < scene_num_lights; i++)
    {
        light cur = scene_lights[i];

        // Get light vector
        vec3 li = v3_normalize(v3_subtract(cur.position, ip.p));

        vec3 si = v3_add(ip.p, v3_multiply(ip.n, 0.001f));

        // Check if overshadowed
        if (shadow_check(si, li))
            continue;

        // Diffuse contribution
        float diffuse_term = fmaxf(0.0f, v3_dotprod(ip.n, li));
        dsum += cur.intensity * diffuse_term;

        // Halfway vector between view and light direction
        vec3 h = v3_normalize(v3_add(ip.i, li));

        // Specular (Blinn-Phong)
        float specular_term = powf(fmaxf(0.0f, v3_dotprod(ip.n, h)), alpha);
        ssum += cur.intensity * specular_term;
    }

    // Combine
    vec3 color = v3_add(v3_multiply(cs, ssum * ks),
                        v3_multiply(cd, (dsum * kd + scene_ambient_light)));
    return color;
}


vec3
shade_reflection(intersection_point ip)
{
    // Reflect ray direction around normal
    vec3 reflected_dir = v3_subtract(ip.i, v3_multiply(ip.n, 2.0f * v3_dotprod(ip.i, ip.n)));
    reflected_dir = v3_normalize(reflected_dir);

    // Offset origin to avoid self-intersections
    const float eps = 0.001f;
    vec3 reflected_origin = v3_add(ip.p, v3_multiply(reflected_dir, eps));

    // Trace reflected ray
    vec3 reflected_color = ray_color(ip.ray_level + 1, reflected_origin, reflected_dir);

    // Base matte
    vec3 matte_color = shade_matte(ip);

    // Combine 75% matte with 25% reflection
    return v3_add(v3_multiply(matte_color, 0.75f), v3_multiply(reflected_color, 0.25f));
}


vec3
shade_frostedglass(intersection_point ip)
{
    float ior = 1.5f; // index of refraction

    // Spherical-ish UV from normal
    vec3 nn = v3_normalize(ip.n);
    float u = 0.5f + atan2f(nn.y, nn.x) / (2.0f * (float)M_PI);
    float v = 0.5f - asinf(nn.z) / (float)M_PI;

    // Roughness parameters
    float rough_clear = 0.05f;
    float rough_frost = 0.2f;

    float roughness = rough_frost;
    if (scene_frost_has_mask)
    {
        rough_frost = 0.553f;
        float mask = tex_sample_luma(&scene_frost_mask, u, v);

        mask = smoothstepf(0.80f, 0.95f, mask);
        mask = clamp01f(mask);
        mask = powf(mask, 1.5f);

        // Interpolate between clear and frosted
        roughness = rough_clear + (rough_frost - rough_clear) * mask;
    }

    // Convert roughness to GGX alpha
    float alpha_g = fmaxf(0.001f, roughness * roughness);
    // to avoid self intersection
    const float eps = 0.001f;

    //hash random
    float h1 = sinf(ip.p.x * 12.9898f + ip.p.y * 78.233f + ip.p.z * 37.719f + (float)ip.ray_level * 19.19f) * 43758.5453f;
    float u1 = h1 - floorf(h1);

    float h2 = sinf(ip.p.x * 93.9898f + ip.p.y * 67.345f + ip.p.z * 12.345f + (float)ip.ray_level * 11.11f) * 12345.6789f;
    float u2 = h2 - floorf(h2);

    vec3 wi = v3_normalize(v3_multiply(ip.i, -1.0f));

    vec3 n = ip.n;
    float etaI = 1.0f;
    float etaT = ior;

    float cos_wi_n = v3_dotprod(wi, n);
    if (cos_wi_n > 0.0f) {
        //flip normal and swap indices
        n = v3_multiply(n, -1.0f);
        float tmp = etaI; etaI = etaT; etaT = tmp;
        cos_wi_n = v3_dotprod(wi, n);
    }

    // sampling microfacet normal m using GGX distribution around n
    // theta_m = atan( alpha_g * sqrt( u1 / (1-u1) ) ), phi_m = 2*pi*u2
    float phi = 2.0f * (float)M_PI * u2;
    float denom = fmaxf(1e-6f, (1.0f - u1));
    float tan2 = (alpha_g * alpha_g) * (u1 / denom);
    float cos_t = 1.0f / sqrtf(1.0f + tan2);
    float sin_t = sqrtf(fmaxf(0.0f, 1.0f - cos_t * cos_t));

    //Local-space micro-normal
    vec3 m_local = v3_create(cosf(phi) * sin_t, sinf(phi) * sin_t, cos_t);

    //Build orthonormal basis
    vec3 up = (fabsf(n.z) < 0.999f) ? v3_create(0, 0, 1) : v3_create(0, 1, 0);
    vec3 t = v3_normalize(v3_crossprod(up, n));
    vec3 b = v3_crossprod(n, t);

    //Transform micro-normal to world
    vec3 m = v3_add(
        v3_add(v3_multiply(t, m_local.x), v3_multiply(b, m_local.y)),
        v3_multiply(n, m_local.z)
    );
    m = v3_normalize(m);

    // Making sure m is in the same hemisphere as n
    if (v3_dotprod(m, n) < 0.0f)
        m = v3_multiply(m, -1.0f);

    //Fresnel for dielectrics
    // F is depends on wi and microfacet normal m
    float cosi = fmaxf(0.0f, fminf(1.0f, -v3_dotprod(wi, m))); // wi points "into scene", so use -wi·m as incidence
    float etai = etaI, etat = etaT;

    //Snell for Fresnel
    float sint = (etai / etat) * sqrtf(fmaxf(0.0f, 1.0f - cosi * cosi));
    float F = 1.0f;
    if (sint < 1.0f) {
        float cost = sqrtf(fmaxf(0.0f, 1.0f - sint * sint));
        float rs = (etai * cosi - etat * cost) / (etai * cosi + etat * cost);
        float rp = (etat * cosi - etai * cost) / (etat * cosi + etai * cost);
        F = 0.5f * (rs * rs + rp * rp);
    }
    else {
        //Total internal reflection
        F = 1.0f;
    }

    //Reflection direction about microfacet normal m
    //wr = wi - 2*(wi·m)*m
    vec3 wr = v3_normalize(v3_subtract(wi, v3_multiply(m, 2.0f * v3_dotprod(wi, m))));

    //Refraction direction through microfacet normal m (Snell)
    float eta = etaI / etaT;
    float cos_wi_m = -v3_dotprod(wi, m); // incidence cosine
    float k = 1.0f - eta * eta * (1.0f - cos_wi_m * cos_wi_m);

    int tir = (k < 0.0f);
    vec3 wt = wr;
    if (!tir) {
        //wt = eta*wi + (eta*cos_wi_m - sqrt(k))*m
        wt = v3_add(
            v3_multiply(wi, eta),
            v3_multiply(m, (eta * cos_wi_m - sqrtf(k)))
        );
        wt = v3_normalize(wt);
    }

    //Trace rays (keep energy stable via Fresnel mix: F + (1-F) = 1)
    vec3 reflected_color = ray_color(ip.ray_level + 1, v3_add(ip.p, v3_multiply(wr, eps)), wr);

    if (tir) {
        return reflected_color; //all reflection
    }

    vec3 refracted_color = ray_color(ip.ray_level + 1, v3_add(ip.p, v3_multiply(wt, eps)), wt);

    //Fresnel mix
    return v3_add(v3_multiply(reflected_color, F),
        v3_multiply(refracted_color, (1.0f - F)));
}


vec3
shade(intersection_point ip)
{
    switch (ip.material)
    {
    case 0: return shade_constant(ip);
    case 1: return shade_matte(ip);
    case 2: return shade_blinn_phong(ip);
    case 3: return shade_reflection(ip);
    case 4: return shade_frostedglass(ip);
    default: return shade_constant(ip);
    }
}


// Determine the surface color for the first object intersected by
// the given ray, or return the scene background color when no
// intersection is found
vec3
ray_color(int level, vec3 ray_origin, vec3 ray_direction)
{
    intersection_point  ip;

    // If this ray has been reflected too many times, simply
    // return the background color.
    if (level >= 6)
        return scene_background_color;

    // Check if the ray intersects anything in the scene
    if (find_first_intersection(&ip, ray_origin, ray_direction))
    {
        // Shade the found intersection point
        ip.ray_level = level;
        return shade(ip);
    }

    if (!use_striped_background) {
        return scene_background_color;
    }

    vec3 d = v3_normalize(ray_direction);
    float u = 0.5f + atan2f(d.y, d.x) / (2.0f * (float)M_PI);

    float freq = 112.0f; // number of bars
    int k = (int)floorf(u * freq);
    int kk = ((k % 8) + 8) % 8;

    // A simple pastel-ish palette + black bars
    vec3 palette[8] = {
        {0.05f, 0.05f, 0.05f}, // near-black
        {0.70f, 0.90f, 0.85f}, // mint
        {0.80f, 0.75f, 0.95f}, // lavender
        {0.95f, 0.75f, 0.65f}, // peach
        {0.75f, 0.95f, 0.70f}, // light green
        {0.65f, 0.80f, 0.95f}, // sky
        {0.95f, 0.85f, 0.55f}, // warm yellow
        {0.70f, 0.70f, 0.70f}  // neutral gray
    };

    vec3 c = palette[kk];

    // Make every other bar black to match “barcode” look
    if ((k & 1) == 0) c = palette[0];

    float v = 0.5f - asinf(d.z) / (float)M_PI; // [0,1]
    float shade = 0.75f + 0.25f * (1.0f - fabsf(2.0f * v - 1.0f));
    c = v3_multiply(c, shade);

    return c;

}