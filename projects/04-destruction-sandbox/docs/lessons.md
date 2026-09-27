# Project 04 Engineering Lessons & Architectural Rationale

This document records the architectural design decisions, technical trade-offs, scope boundaries, and cross-project lessons applied during **Project 04 — Procedural Destruction Sandbox**.

---

## 1. Architectural Lessons Applied from Projects 01–03

### 1.1 Lessons from Project 01 (CPU Ray Tracer)
- **Spatial Pruning Efficiency**: Project 01 demonstrated that spatial hierarchy bounds (BVH) are essential for performance. Project 04 applies this principle by utilizing a Dynamic AABB Tree for broadphase collision detection, preventing $O(N^2)$ narrowphase pair evaluations.
- **Analytical Math Verification**: First-principles analytical verification of ray-geometry intersections proved vital in Project 01. In Project 04, rigid body dynamics and GJK/EPA collision manifolds are similarly verified against closed-form analytical equations before performance optimization.

### 1.2 Lessons from Project 02 (Voxel Engine)
- **Decoupled Asynchronous Workflows**: Project 02 used asynchronous worker thread queues for terrain chunk meshing with neighborhood snapshot isolation. Project 04 adopts the same pattern: dynamic Voronoi mesh fracturing tasks are offloaded to background worker threads without blocking the main physics simulation step.
- **Buffer Reuse**: Dynamic vertex/index buffer recycling from Project 02 is adapted in Project 04 to handle dynamic polyhedral shard mesh buffers without runtime dynamic allocation churn.

### 1.3 Lessons from Project 03 (Performance Lab)
- **Memory Allocator Strategy**: Project 03 proved that general heap allocation (`malloc`/`free`) causes lock contention and cache misses. Project 04 uses Project 03's `LinearArena` (monotonic bump allocator) for transient frame contact points and manifold generation, resetting memory in $O(1)$ time per frame step.
- **SIMD Intrinsics**: Direct vector math intrinsics (AVX2/SSE) evaluated in Project 03 are integrated into Project 04's 3D vector, matrix, and AABB transform routines.

---

## 2. Key Phase 0 Design Decisions & Trade-Offs

### 2.1 Voronoi Mesh Clipping vs. CSG Boolean Operations
- **Decision**: Use 3D planar bisector clipping (half-space clipping) rather than general Constructive Solid Geometry (CSG) mesh boolean operations.
- **Rationale**: General CSG operations are computationally expensive, numerically sensitive, and prone to topological errors on complex meshes. Planar bisector clipping on convex polyhedra is deterministic, robust, highly parallelizable, and generates clean watertight convex shard meshes.

### 2.2 Sequential Impulses (PGS) vs. Penalty-Based Collision Methods
- **Decision**: Implement a Projected Gauss-Seidel (PGS) Sequential Impulse solver rather than penalty spring-damper collision methods.
- **Rationale**: Penalty-based collision methods require extremely small timesteps to avoid stiffness explosion and visual jitter. Sequential impulse solvers operate stably on fixed timesteps ($\Delta t = 1/60\text{ s}$), enforcing hard velocity/position constraints while handling static and dynamic friction accurately.

### 2.3 Structural Graph Stress Analysis vs. Finite Element Method (FEM)
- **Decision**: Represent compound objects with a structural connectivity graph evaluating inter-shard stress rather than full continuum Finite Element Method (FEM) elasticity.
- **Rationale**: Real-time FEM continuum simulation is computationally intensive and better suited for soft-body deformation. Structural connectivity graphs provide fast, predictable, real-time stress propagation across rigid body fragments, enabling instant fracture and cascading collapse.

---

## 3. Explicit Scope Boundaries & Non-Goals

To maintain strict architectural focus and engineering quality, the following features are explicitly deferred or placed out of scope for Project 04:

- **GPGPU Compute Shaders**: Dynamic surface deformation and GPU compute fields are explicitly deferred to **Project 05 — GPU Crater Simulator**.
- **Entity-Component-System (ECS)**: Engine runtime entity systems are explicitly deferred to **Project 06 — Tiny Game Engine**.
- **Soft Body / Deformable FEM**: Plastic soft-body continuum mechanics are out of scope; Project 04 focuses exclusively on rigid body dynamics and discrete procedural fracture.
- **Premature Framework Extraction**: In accordance with the repository core principle, no code from Project 04 will be extracted into shared `libs/` until multi-project reuse is demonstrated in a future project.
