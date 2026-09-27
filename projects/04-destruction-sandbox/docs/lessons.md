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

## 3. Milestone 2 Engineering Lessons & Design Decisions

### 3.1 Pairwise Canonical Bisector Planes for Shared-Face Consistency
- **Lesson**: If neighboring Voronoi cells $i$ and $j$ derive bisector planes independently using slightly different floating-point calculations, their common shared face will possess divergent plane coefficients. In M2, the canonical bisector plane between $\mathbf{p}_i$ and $\mathbf{p}_j$ is derived with normal $\mathbf{n} = \frac{\mathbf{p}_j - \mathbf{p}_i}{\|\mathbf{p}_j - \mathbf{p}_i\|}$ and midpoint $\mathbf{m} = \frac{1}{2}(\mathbf{p}_i + \mathbf{p}_j)$. Cell $i$ uses half-space $\mathbf{n} \cdot \mathbf{x} + d \le 0$, while cell $j$ uses flipped plane $-\mathbf{n}$ and $-d$. This guarantees bitwise identical shared-face geometry across adjacent cell boundaries.

### 3.2 Cap-Face Construction via Orthonormal 2D Polar Sorting
- **Lesson**: When a plane clips a convex polyhedron, intersection points on the clipping plane must be ordered into a valid closed polygon. Simply appending points in traversal order produces self-intersecting bow-tie polygons. In M2, intersection points on the plane are collected, deduplicated within $\epsilon = 10^{-5}$, projected into a 2D orthonormal basis $(\mathbf{u}, \mathbf{v})$ on the plane, sorted by polar angle $\theta = \text{atan2}(v, u)$ around the cap centroid, and emitted as a counter-clockwise cap face matching the plane's outward-pointing normal.

### 3.3 Exact Polyhedral Volume & Centroid via Tetrahedral Fan Decomposition
- **Lesson**: Averaging vertices does not yield the true volume centroid (center of mass) of a non-uniform or non-symmetric polyhedron. In M2, volume and centroid are computed by choosing an interior reference point $\mathbf{r}$ (vertex average) and decomposing each face into tetrahedra $(\mathbf{r}, \mathbf{v}_0, \mathbf{v}_k, \mathbf{v}_{k+1})$. Summing signed tetrahedral volumes $V_{tet} = \frac{1}{6} (\mathbf{v}_0 - \mathbf{r}) \cdot \left((\mathbf{v}_k - \mathbf{r}) \times (\mathbf{v}_{k+1} - \mathbf{r})\right)$ and volume-weighted centroids $\mathbf{C} = \frac{1}{V} \sum V_{tet} \mathbf{c}_{tet}$ yields exact $0.0000\%$ volume and mass conservation even for translated boxes away from the origin.

### 3.4 Topological Closed-Manifold Edge Validation
- **Lesson**: Geometric volume calculation alone cannot detect unclosed polyhedral meshes or internal boundary holes. In M2, `MeshValidator` constructs an undirected edge map $(\min(v_a, v_b), \max(v_a, v_b))$ across all faces. For a valid closed convex polyhedron, EVERY undirected edge MUST be shared by exactly two faces (boundary edge count = 0).

---

## 4. Milestone 3 Engineering Lessons & Design Decisions

### 4.1 Two-Step Tree Recursion for Broadphase Candidate Pair Generation
- **Lesson**: Traversing a dynamic AABB binary tree by naïvely pushing subtrees can generate redundant or duplicate candidate pair checks. In M3, `DynamicAabbTree` separates broadphase pair generation into two clean recursive functions: `generate_pairs_recursive(node)` (recursing on left/right children and calling `query_pair(left, right)`) and `query_pair_recursive(na, nb)` (pruning non-overlapping AABBs and generating canonical pairs $(A, B)$ with $A < B$). This guarantees zero duplicate pair generation and zero missed broadphase overlaps.

### 4.2 GJK Simplex Evolution & Termination Protection
- **Lesson**: Unbounded GJK iterations can cycle infinitely when floating-point precision limits cause support point calculations to stall near the origin. In M3, `Gjk` enforces a hard iteration cap (`GJK_MAX_ITERATIONS = 64`), checks direction vectors against $\epsilon = 10^{-6}$, and terminates early if new support points fail to advance past the origin in search direction $\mathbf{d}$.

### 4.3 EPA Polytope Expansion & Fallback SAT Cross-Validation
- **Lesson**: Expanding Polytope Algorithm (EPA) requires a valid non-degenerate 3D simplex (tetrahedron) from GJK. When GJK detects shallow/touching contacts with degenerate initial simplexes, EPA iteration can stagnate. In M3, `Narrowphase` utilizes EPA as the primary penetration engine, but seamlessly falls back to `Sat` (Separating Axis Theorem) if EPA numerical convergence fails. Unit tests (`test_collision`) cross-validate GJK/EPA against SAT, confirming identical collision classification and matching penetration depths within $10^{-2}\text{ m}$ tolerance.

### 4.4 4-Point Reduced Contact Manifolds
- **Lesson**: Storing dozens of contact points per face-face collision increases constraint solver complexity without improving stability. In M3, `ContactManifold::reduce_to_max_4()` reduces candidate contact point sets to a maximum of 4 points by selecting: (1) deepest penetration point, (2) point furthest from point 1, (3) point maximizing triangle area with points 1 and 2, and (4) point maximizing 3D distance/area with points 1, 2, 3. This maximizes contact patch stability while maintaining a fixed solver bound.

---

## 5. Milestone 4 Engineering Lessons & Design Decisions

### 5.1 Split Impulses vs Direct Baumgarte Velocity Bias
- **Lesson**: Adding Baumgarte position correction bias $b = \frac{\beta}{\Delta t} \max(0, d - \text{slop})$ directly into the velocity solver equation causes position stabilization impulses to bleed into body linear and angular velocities. This artificially injects physical kinetic energy, making dynamic bodies bounce or jitter upward in resting stacks. In M4, `SequentialImpulseSolver` implements **Split Impulses**: physical velocity iterations solve velocity response and restitution, while position stabilization iterations solve pseudo-velocities ($\mathbf{v}_{ps}, \boldsymbol{\omega}_{ps}$) on a separate accumulator. Pseudo-velocities update body positions directly ($\mathbf{x} \leftarrow \mathbf{x} + \mathbf{v}_{ps} \Delta t$) without modifying physical velocities $\mathbf{v}$ and $\boldsymbol{\omega}$, preserving exact kinetic energy stability.

### 5.2 Deterministic 64-Bit Warm Start Contact Key
- **Lesson**: Caching accumulated impulses using raw memory pointers breaks determinism across different runs and thread schedules, while using simple body ID pairs causes contact impulse bleeding across multiple contact points on the same body pair. In M4, `WarmStartCache` constructs a deterministic 64-bit key:
  $$\text{key} = (\min(id_A, id_B) \ll 48) \oplus (\max(id_A, id_B) \ll 32) \oplus \text{feature\_hash}$$
  where `feature_hash` combines manifold feature IDs or 3D quantized contact point coordinates. Unused cache entries are tracked by age and pruned automatically after 2 steps, preventing unbounded cache growth.

### 5.3 Upward Normal Reaction Threshold for Support Eligibility
- **Lesson**: Treating every physical collision contact as a vertical support relationship in the structural connectivity graph causes horizontal lateral collisions or wall scrapes to incorrectly count as structural load supports. In M4, `StructuralGraph` evaluates support eligibility using the upward component of the reaction normal: if $-\mathbf{n} \cdot \hat{\mathbf{y}} > 0.3$ (where $\mathbf{n}$ points from A to B), body B provides vertical support to body A.

### 5.4 Iterative Load Propagation & BFS Connectivity Sweeps
- **Lesson**: Evaluating structural graph stress using simple local adjacency fails to identify unanchored cantilever overhangs or floating structural sections. In M4, `StructuralGraph` uses a two-phase evaluation: (1) an iterative load propagation sweep distributing gravitational weight ($m \cdot g$) and transmitted loads across active support edges, breaking edges whose transmitted load exceeds capacity $A \cdot \sigma_{\text{tensile}} \cdot k_{\text{mult}}$; (2) a BFS graph connectivity sweep starting from anchored ground root nodes along active edges, marking reachable nodes as supported and unreachable nodes as unsupported/dynamic debris.

---

---

## 6. Milestone 5 Engineering Lessons & Design Decisions

### 6.1 Strict Physics and Rendering Separation via Immutable State Snapshots
- **Lesson**: Coupling OpenGL draw calls or scene graph state directly into rigid body structures prevents headless automated testing and makes physics simulation non-deterministic across different rendering platforms. In M5, `destruction::render` consumes the simulation strictly via read-only snapshot queries (`world.get_bodies()`, `world.get_colliders()`, `world.get_manifolds()`, `graph.get_edges()`). The physics engine has zero `#include <GL/...>` or `<GLFW/...>` dependencies, allowing headless regression suites (`val_destruction_headless`) and CPU micro-benchmarks (`bench_destruction`) to run in complete isolation from the graphics driver.

### 6.2 Fixed Timestep Accumulator with Spiral-of-Death Clamping
- **Lesson**: Stepping physics with variable frame delta time $\Delta t_{\text{render}}$ causes non-deterministic simulation divergence and solver instability when frame rate fluctuates. In M5, `SandboxApp` accumulates real elapsed time and advances physics strictly in discrete $1/60\text{ s}$ ($16.66\text{ ms}$) sub-steps. To prevent the "spiral of death" (where slow physics simulation leads to longer render frames, causing even more physics sub-steps on the subsequent frame), the sub-step loop is hard-capped at a maximum of 4 sub-steps per visual frame.

### 6.3 Swept Projectile Advancement for High-Speed Impact Integrity
- **Lesson**: High-velocity projectiles moving at $28\text{ m/s}$ traverse $0.467\text{ m}$ per physics step at $\Delta t = 1/60\text{ s}$, which exceeds the half-extent of standard structural blocks ($0.5\text{ m}$), leading to discrete tunneling through targets. In M5, high-speed projectiles calculate an expanded swept AABB encompassing their start and end positions over the frame step. This ensures broadphase detection and narrowphase contact generation reliably register the impact and trigger Voronoi fracture without tunneling.

### 6.4 Clean Fragment Geometry Upload & Normal Reconstruction
- **Lesson**: Dynamically created Voronoi polyhedral fragments have arbitrary facet vertex counts and irregular face topologies that cannot be rendered with fixed cube vertex layouts. In M5, `GlMesh::from_polyhedron` triangulates polyhedral polygon faces using a triangle fan from the first vertex of each face, computes the outward-facing geometric face normal via cross product $(\mathbf{v}_1 - \mathbf{v}_0) \times (\mathbf{v}_2 - \mathbf{v}_0)$, and packs vertex positions, normals, and material colors into a single contiguous VAO/VBO. Shards upload once upon fracture and reuse their GPU buffers while updating only their model transform matrix per frame.

### 6.5 Multi-Workload Canonical Benchmark Aggregation
- **Lesson**: Invoking separate CSV export calls on the same file path in sequential benchmark workloads can trigger overwrite collisions or truncated CSV headers. In M5, `bench_destruction` collects all benchmark results across all 7 evaluation phases (fracture scaling, broadphase, narrowphase, solver, structural graph, integrated physics, and fracture-to-collapse) into a consolidated vector, exporting the full canonical report `destruction_release.csv` in a single atomic write operation.

---

## 7. Explicit Scope Boundaries & Non-Goals

To maintain strict architectural focus and engineering quality, the following features are explicitly deferred or placed out of scope for Project 04:

- **GPGPU Compute Shaders**: Explicitly deferred to **Project 05 — GPU Crater Simulator**.
- **Entity-Component-System (ECS)**: Explicitly deferred to **Project 06 — Tiny Game Engine**.
- **WebGPU / Browser Deployment**: Out of scope for this repository.
- **Soft-Body / FEM Simulation**: Out of scope; Project 04 is an engineering rigid-body destruction sandbox.
- **Third-Party Physics Engine Dependencies**: Strictly prohibited; all math, integrators, clipping, broadphase, narrowphase, solvers, and graph routines are self-contained.
