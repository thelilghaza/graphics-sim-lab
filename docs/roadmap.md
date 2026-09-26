# Graphics & Simulation Lab — Project Roadmap

This document outlines the planned sequence of engineering projects within the Graphics & Simulation Lab.

---

## 1. CPU Ray Tracer (`projects/01-raytracer`) — COMPLETE
* **Status**: Complete through Milestone 5.
* **Focus**: First-principles rendering pipeline.
* **Key Topics**: Ray-geometry intersection (spheres, AABBs), camera ray generation, Lambertian/metallic/dielectric materials, recursive ray tracing, anti-aliasing, bounding volume hierarchies (BVH), multithreading, performance benchmarks.
* **Language/Tooling**: C++20, CTest, PPM output.

## 2. Voxel Engine (`projects/02-voxel-engine`) — NEXT (Design Phase)
* **Status**: In Design & Architecture.
* **Focus**: Volumetric representation, spatial locality, and meshing algorithms.
* **Key Topics**: Chunked 3D spatial grids, naive face culling, greedy meshing algorithm, fast voxel raycasting (DDA), procedural terrain generation, boundary/neighbor handling.
* **Language/Tooling**: C++20, CMake, CTest.

## 3. Performance Lab (`projects/03-performance-lab`) — PLANNED
* **Focus**: Hardware-aware performance engineering.
* **Key Topics**: Cache locality & data layout (SoA vs AoS), SIMD vectorization (AVX2/AVX-512/NEON), lock-free queues, custom allocators, micro-benchmarking protocols.
* **Language/Tooling**: C++20, Custom high-resolution timers.

## 4. Procedural Destruction Sandbox (`projects/04-destruction-sandbox`) — PLANNED
* **Focus**: Structural fracture & physics simulation.
* **Key Topics**: Voronoi 2D/3D partitioning, rigid body dynamic simulation, collision manifolds, impulse-based constraint solvers, structural connectivity graphs.
* **Language/Tooling**: C++20, Native math.

## 5. GPU Crater Simulator (`projects/05-crater-simulator`) — PLANNED
* **Focus**: GPGPU compute & dynamic heightmap deformation.
* **Key Topics**: Compute shaders, surface deformation fields, impact kinetic energy transfer, ejecta particle systems, GPU buffer synchronization.
* **Language/Tooling**: C++20, Vulkan / Direct3D 12 Compute.

## 6. Tiny Game Engine (`projects/06-tiny-engine`) — PLANNED
* **Focus**: Game engine architecture & runtime systems.
* **Key Topics**: Entity-Component-System (ECS), transform hierarchies, scene graphs, event dispatching, input mapping, asset pipeline.
* **Language/Tooling**: C++20.

## 7. WebGPU 3D Engine (`projects/07-webgpu-engine`) — PLANNED
* **Focus**: Modern web-native graphics architecture.
* **Key Topics**: WGSL shader pipelines, Physically Based Rendering (PBR), glTF 2.0 loading, deferred shading vs forward+, WebAssembly bindings.
* **Language/Tooling**: TypeScript / WebGPU / C++ WASM.

## 8. Godot Project Analyzer (`projects/08-godot-analyzer`) — PLANNED
* **Focus**: Developer tooling & static analysis.
* **Key Topics**: AST parsing of GDScript/Scene files, asset cross-referencing, dependency graph generation, performance hotspot diagnostics.
* **Language/Tooling**: C++20 / Tooling CLI.
