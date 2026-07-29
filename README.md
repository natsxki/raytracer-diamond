<div align="center">

# ⋆౨ৎ⋆ Raytracer Diamond ⋆౨ৎ⋆

***Real-time, physically-based raytracing of diamonds ***

<br>

![C++](https://img.shields.io/badge/C++-17-FFB5C2?style=flat-square&logo=cplusplus&logoColor=white)
![OpenGL](https://img.shields.io/badge/OpenGL-Core-C8B6FF?style=flat-square&logo=opengl&logoColor=white)
![GLSL](https://img.shields.io/badge/GLSL-Fragment%20Shader-B5D8FF?style=flat-square)
![GLFW](https://img.shields.io/badge/GLFW-window%20%2B%20input-B8E6D9?style=flat-square)
![GLM](https://img.shields.io/badge/GLM-math-FFE0B5?style=flat-square)

<br>

*A Whitted raytracer that runs entirely in a fragment shader,*
*rendering optically-accurate gemstones with real-time rigid-body physics.*

</div>

---

## What is this?

A computer graphics project that simulates the way light *actually* behaves inside a cut diamond — bouncing, bending, splitting into colors, and slowly getting absorbed, while the stone tumbles and bounces around under gravity. Everything is raytraced live on the GPU, so you can spin the camera, drag the lights around, and drop a fresh diamond whenever you like. 

Built as the final project for **IGR — Fundamentals of Computer Graphics** (Télécom Paris).

---

## Features

### Optics — simulated in GLSL

Every pixel is computed by tracing rays through the scene, with the full physics of gemstones baked into the shader:

- **Whitted raytracing** — recursive reflection & refraction, up to `12` bounces per ray
- **Refraction & reflection** at every surface, using Snell's law
- **Total Internal Reflection (TIR)** — the effect that makes diamonds sparkle, when light gets trapped inside and bounces back out
- **Fresnel** term (Schlick approximation) to blend reflection vs. transmission by viewing angle
- **Chromatic dispersion** — red, green & blue are traced with slightly different indices of refraction (`1.70 / 1.80 / 1.90`), giving that lovely rainbow "fire" 
- **Beer–Lambert absorption** — light dims as it travels through the stone, so thicker paths look richer

### Physics — a tiny rigid-body solver

A hand-written solver (`RigidSolver.hpp`) handles the motion:

- **Free fall** under gravity
- **Quaternion-based rotation** for smooth, gimbal-lock-free tumbling
- **Floor collisions** with restitution (bounciness) and friction
- Optional **walls** to keep the diamond in frame

### Scene & lighting

- Two movable colored lights (a red one and a blue one)
- Toggleable **mirror / matte** floor
- **Dark / light** background modes
- Save the current frame straight to a `.ppm` image

---

## How it works

```
        ┌─────────────┐   diamond.obj triangles
        │   main.cpp  │ ───────────────────────────┐
        │  (C++ host) │                             │  packed into a
        └──────┬──────┘                             │  texture buffer
               │ camera, lights, model matrix       ▼
               │ (uniforms)                  ┌───────────────┐
               └────────────────────────────▶ raytracer.frag │
                                             │  (the GPU!)   │
                                             │               │
   full-screen quad ──▶ every fragment ──▶   │ trace a ray:  │
                                             │  ↳ refract    │
   RigidSolver.hpp ──▶ updates position/     │  ↳ reflect    │
        rotation each frame ────────────────▶│  ↳ Fresnel    │
                                             │  ↳ absorb     │
                                             └───────┬───────┘
                                                     ▼
                                               sparkly pixel 
```

The C++ side is basically a thin host: it opens a window, loads the diamond mesh, streams the triangles to the GPU as a `samplerBuffer`, advances the physics, and draws a single full-screen quad. **All the actual rendering happens in `raytracer.frag`**, once per pixel, every frame.

---

## Controls

| Key | Action |
|:---:|:-------|
| `P` | ▶ / ⏸ play / pause the simulation |
| `R` | ✧ reset — drop a brand-new diamond |
| `F` | reset the camera |
| `M` | toggle floor: mirror ⇄ matte |
| `W` | toggle walls on / off |
| `L` | toggle background: dark ⇄ light |
| `S` | save the current frame as `.ppm` |
| `C` | control the **camera** |
| `1` / `2` | control **light 1** (red) / **light 2** (blue) |
| `↑ ↓ ← →` | move the selected object (x / y) |
| `I` / `O` | move the selected object (z, in / out) |
| mouse drag | rotate · pan · zoom the camera |
| `Esc` | quit |

---

## Building & running

### You'll need

- A C++17 compiler
- **OpenGL** (3.3+ core profile)
- [GLFW](https://www.glfw.org/) · [GLAD](https://glad.dav1d.de/) · [GLM](https://github.com/g-truc/glm)
- A `diamond.obj` mesh placed at `../data/diamond.obj` (relative to the executable)

### Compile

There's no build system committed yet, so here's a quick starting point (adjust paths to your local libs):

```bash
g++ -std=c++17 src/*.cpp glad.c \
    -o raytracer-diamond \
    -lglfw -lGL -ldl

./raytracer-diamond
```

> !!! *Small heads-up:* the `data/` folder (with `diamond.obj`) and a proper build file aren't in the repo yet — you'll want to add your own mesh and, ideally, a `CMakeLists.txt` to make this reproducible

---

## Project structure

```
raytracer-diamond/
├── src/
│   ├── main.cpp           ⋆ window, input, render loop, mesh → GPU
│   ├── raytracer.frag     ✧ the heart: Whitted raytracer in GLSL
│   ├── raytracer.vert     · trivial full-screen quad vertex shader
│   ├── RigidSolver.hpp    ♡ physics: gravity, collisions, quaternions
│   ├── Mesh.{h,cpp}       · OBJ loading & mesh handling
│   ├── camera.h           · view / projection matrices
│   ├── ShaderProgram.{h,cpp}  · GLSL compilation helpers
│   ├── Vector3.hpp        · math primitives
│   ├── Matrix3x3.hpp      · math primitives
│   ├── typedefs.hpp       · shared type aliases
│   └── Error.{h,cpp}      · GL error checking
├── presentation.pdf       ⋆ theory, code walkthrough & features
└── README.md
```

The **[presentation](presentation.pdf)** goes deeper into the theory and the implementation

---

<div align="center">

*Made by [**natsxki**](https://github.com/natsxki) 

</div>
