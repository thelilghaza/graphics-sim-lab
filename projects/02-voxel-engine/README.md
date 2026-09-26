# Project 02: Voxel Engine

A high-performance C++20 voxel engine focused on volume representation, spatial locality, memory layout, surface extraction (naive culling & greedy meshing), and multithreaded chunk management.

---

## Current Status: In Architecture & Design Phase

Project 02 is currently in its initial architecture, design, and roadmap specification phase. No voxel source implementation has been written yet.

The project will establish a cache-friendly, memory-efficient 3D spatial voxel representation and meshing pipeline built from first principles.

---

## Core Technical Questions Investigated

1. **Voxel Storage & Memory Representation**:
   - How can 3D volumetric data be stored so that random access is $O(1)$, memory usage is predictable, and spatial iteration matches CPU cache lines?
2. **Chunk Sizing & Locality**:
   - What chunk dimensions ($8^3$, $16^3$, $32^3$, or $64^3$) optimize the balance between memory footprint, cache line utilization (L1/L2), boundary neighbor lookups, and meshing update latency?
3. **World Coordinate Conversion**:
   - How do world coordinates $(Wx, Wy, Wz)$ map deterministically to chunk coordinates $(CX, CY, CZ)$ and local voxel coordinates $(lx, ly, lz)$, specifically handling negative world space without division truncation bugs?
4. **Surface Extraction & Meshing Algorithms**:
   - What performance and quad-reduction benefits does greedy meshing provide over naive exposed-face culling, and how does meshing throughput scale across thread counts?
5. **Data vs Renderer Separation**:
   - How can voxel data structures and mesh extraction remain completely decoupled from rendering frameworks, allowing visualization backends to be swapped or benchmarked independently?

---

## Planned Milestone Roadmap

- [ ] **Milestone 1**: Compact Voxel Payload & Contiguous $32^3$ Chunk Storage Architecture
- [ ] **Milestone 2**: World Coordinate Conversion System & Cross-Chunk Neighbor Access API
- [ ] **Milestone 3**: Basic Voxel Editing API & Deterministic Test Worlds (Solid, Empty, Sphere, Plane)
- [ ] **Milestone 4**: Naive Exposed-Face Culling Mesher & Mesh Buffer Data Structure
- [ ] **Milestone 5**: Minimal Visualization Layer & Interactive Camera
- [ ] **Milestone 6**: Greedy Meshing Algorithm & Quad-Reduction Performance Analysis
- [ ] **Milestone 7**: Dynamic Chunk Manager & Distance-Based Chunk Streaming
- [ ] **Milestone 8**: Multithreaded Chunk Generation & Parallel Mesh Extraction
- [ ] **Milestone 9**: Memory Footprint Optimization & Micro-Benchmarking Suite
- [ ] **Milestone 10**: Level of Detail (LOD) & Large-World Scale Experiments

---

## Scope Boundaries & Explicit Non-Goals
- **No Premature Vulkan/GPU Compute**: Initial meshing and storage will be implemented on the CPU to establish clear baselines before offloading to GPU shaders in later projects.
- **No Structural Fracture Physics**: Dynamic destruction solvers and impulse fracture physics belong to *Project 04: Procedural Destruction Sandbox*.
- **No Complex Gameplay Engine**: Audio, scripting, and entity component systems will not be created here.

---

## Build & Test Plan
Once implementation begins, Project 02 will be built via standard repository CMake presets (`default` and `release`) and verified using CTest suite targets in `projects/02-voxel-engine/tests/`.
