# Project 04 Engineering Lessons & Architectural Rationale

This document records the architectural design decisions, technical trade-offs, scope boundaries, and cross-project lessons applied during **Project 04 — Procedural Destruction Sandbox**.

---

## 1. Architectural Lessons Applied from Projects 01–03

### 1.1 Lessons from Project 01 (CPU Ray Tracer)
- **Spatial Pruning Efficiency**: Project 01 demonstrated that spatial hierarchy bounds (BVH) are essential for performance. Project 04 applies this principle by utilizing a Dynamic AABB Tree for broadphase collision detection, preventing $O(N^2)$ narrowphase pair evaluations.
- **Analytical Math Verification**: First-principles analytical verification of ray-geometry intersections proved vital in Project 01. In Project 04 Milestone 1, rigid body dynamics and vector/matrix/quaternion math were similarly verified against closed-form analytical equations before physics solver design.

### 1.2 Lessons from Project 02 (Voxel Engine)
- **Decoupled Asynchronous Workflows**: Project 02 used asynchronous worker thread queues for terrain chunk meshing with neighborhood snapshot isolation. Project 04 adopts the same pattern: dynamic Voronoi mesh fracturing tasks are offloaded to background worker threads without blocking the main physics simulation step.
- **Buffer Reuse**: Dynamic vertex/index buffer recycling from Project 02 is adapted in Project 04 to handle dynamic polyhedral shard mesh buffers without runtime dynamic allocation churn.

### 1.3 Lessons from Project 03 (Performance Lab)
- **Memory Allocator Strategy**: Project 03 proved that general heap allocation (`malloc`/`free`) causes lock contention and cache misses. Project 04 uses Project 03's `LinearArena` (monotonic bump allocator) for transient frame contact points and manifold generation, resetting memory in $O(1)$ time per frame step.
- **SIMD Intrinsics**: Direct vector math intrinsics (AVX2/SSE) evaluated in Project 03 are integrated into Project 04's 3D vector, matrix, and AABB transform routines.

---

## 2. Milestone 1 Engineering Lessons & Design Decisions

### 2.1 Symplectic Euler vs Explicit Euler Integration
- **Lesson**: Standard Explicit Euler updates position using current velocity $\mathbf{x}(t+\Delta t) = \mathbf{x}(t) + \mathbf{v}(t) \Delta t$, which gains artificial kinetic energy over time in harmonic/gravitational systems. Symplectic Euler (Semi-Implicit Euler) updates velocity first and then uses the *new* velocity $\mathbf{v}(t+\Delta t)$ to update position:
  $$\mathbf{v}(t+\Delta t) = \mathbf{v}(t) + \mathbf{a}(t) \Delta t$$
  $$\mathbf{x}(t+\Delta t) = \mathbf{x}(t) + \mathbf{v}(t+\Delta t) \Delta t$$
  This conserves phase-space area and guarantees energy stability in discrete physics integration.

### 2.2 Discrete Recurrence vs Continuous Integral Validation
- **Lesson**: When verifying discrete physics integrators in unit tests, analytical continuous equations $y(t) = y_0 - \frac{1}{2} g t^2$ contain discretization error relative to discrete timestep updates. For discrete Symplectic Euler with fixed $\Delta t$, position after $N$ steps is exactly:
  $$y_N = y_0 + \frac{N(N+1)}{2} g (\Delta t)^2$$
  Comparing unit test results against the exact discrete recurrence formula ensures mathematically exact $10^{-5}$ tolerance verification.

### 2.3 Zero-Length Vector Normalization & Singular Matrix Inversion
- **Lesson**: Blindly dividing vectors by length or matrices by determinant leads to floating-point `NaN` or `Inf` propagation. In M1, `normalize()` explicitly returns `Vec3::zero()` if $\|v\| \le 10^{-6}$, and `Mat3::inverse()` checks $|\det(M)| \le 10^{-6}$, returning `Mat3::zero()` and an explicit `success` flag.

### 2.4 Active Quaternion Rotation Convention
- **Lesson**: Mixing active $v' = q v q^*$ and passive $v' = q^* v q$ quaternion rotation conventions leads to reversed rotation angles. In M1, Hamilton product order and $v' = q v q^*$ active rotation are strictly locked down, with explicit unit tests comparing $q \cdot v \cdot q^*$ directly against $R(q) \mathbf{v}$ matrix transformation.

---

## 3. Explicit Scope Boundaries & Non-Goals

To maintain strict architectural focus and engineering quality, the following features are explicitly deferred or placed out of scope for Project 04:

- **Voronoi Fracture & Mesh Clipping**: Deferred to **Milestone 2**.
- **Collision Detection & Contact Manifolds (GJK/EPA/SAT)**: Deferred to **Milestone 3**.
- **Sequential Impulse Constraint Solver & Friction**: Deferred to **Milestone 4**.
- **Structural Connectivity Graph Stress Analysis**: Deferred to **Milestone 4**.
- **Interactive 3D OpenGL Demo Application**: Deferred to **Milestone 5**.
- **GPGPU Compute Shaders**: Explicitly deferred to **Project 05 — GPU Crater Simulator**.
- **Entity-Component-System (ECS)**: Explicitly deferred to **Project 06 — Tiny Game Engine**.
