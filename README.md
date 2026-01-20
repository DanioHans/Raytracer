# Raytracer – BVH / Grid / Octree Comparison

This project is an OpenGL-based ray tracer that can render a scene either with
the fixed-function OpenGL pipeline or via CPU ray tracing. It supports three
acceleration structures:

- **BVH** (Bounding Volume Hierarchy)
- **Uniform Grid**
- **Octree**

The program can also print detailed profiling statistics (rays, intersection
counts, grid steps, etc.) for analysis.

Material support for Dielectric frosted glass with texture mapping, see frosted glass shader and texture code.

---

## Build Instructions

### 1. Requirements

- A C compiler (GCC / Clang / MSVC)
- OpenGL development headers & libraries
- **FreeGLUT** (or GLUT-compatible implementation)
- **GLEW** (OpenGL Extension Wrangler)
- C standard library & `math.h` (`-lm` on Unix-like systems)

2. Linux (recommended)
Install dependencies (example: Debian/Ubuntu)
```
sudo apt update
sudo apt install build-essential freeglut3-dev libglew-dev mesa-common-dev
```


On other distros, install the equivalent packages for:
freeglut (dev)
glew (dev)
OpenGL (dev)

Build with the provided Makefile
This repository includes a Makefile. To build use ```make```

3. Windows
Recommended is using Visual Studio, open the .sln file and build using the Visual Studio interface

### 2. Usage
command line usage:
``` ./raytracer ./path/to/scene.scn ```

Keybindings:
- `r` - switch between raytraced or OpenGL view
- `b` - toggle BVH structure
- `g` - toggle grid structure
- `o` - toggle Octree structure
- `B` - toggle BVH visualisation
- `[` - decrease BVH visualisation depth
- `]` - increase BVH visualisation depth
- `n` - show vertex normals
- `c` - print current camera parameters
- `m` - print mouse coordinates
- `1-6` - predefined camera viewpoints
- `q` - quit application
- `s` - save image
- `a` - toggle anti-aliasing
- `t` - toggle between striped or white background
