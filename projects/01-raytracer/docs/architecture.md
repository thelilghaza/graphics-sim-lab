# Project 01 Architecture — CPU Ray Tracer

This document details the architecture, data flow, material scattering models, camera optics, spatial acceleration hierarchy (AABB & BVH), multithreaded work scheduling, determinism strategy, and output pipeline for the CPU Ray Tracer.

---

## Project Status: Project 01 Complete (Milestone 5)

Project 01 is fully complete through Milestone 5. The next project on the repository roadmap is **Project 02 — Voxel Engine**.

---

## Data Flow & Acceleration Pipeline

```text
Image Buffer (Width x Height)
      │
      ├── Work Partitioning ──> Row Ranges [row_start, row_end) assigned to N Worker Threads
      │
Worker Threads (t = 0..N-1)
      │
      ├── Pixel (i, j), Sample s [s = 0..S-1]
      │     │
      │     ▼
      │   sample_seed = SplitMix64(global_seed, i, j, s)
      │   RNG sample_rng(sample_seed)
      │     │
      │     ▼
      │   Camera Ray Generation:
      │         ├── (aperture == 0.0) ==> Pinhole Ray from lookfrom
      │         └── (aperture > 0.0)  ==> Thin-Lens Ray from offset on lens disk
      │     │
      │     ▼
      │   Accelerated Traversal (BVH / Naive Selection):
      │         ├── BVH Mode   : Ray ──> Root AABB ──> Node AABB Test ──> Leaf Primitive Test ──> Closest Hit
      │         └── Naive Mode : Ray ──> Linear Primitive Loop (Reference Oracle)
      │     │
      │     ▼
      │   Recursive ray_color(...) Pipeline -> Thread-Local RenderStats[t]
      │         ├── (depth >= max_depth) ==> Return Color(0, 0, 0)
      │         ├── Hit: Specular / Diffuse Bounces (Lambertian Hybrid Direct + Indirect)
      │         └── Miss: Sky Gradient Background
      │
      ▼
Join Threads ──> Merge Thread-Local RenderStats[t] into Total Stats ──> Gamma 2.0 (sqrt) ──> Write PPM
```

---

## Spatial Acceleration Architecture (AABB & BVH)

### 1. Axis-Aligned Bounding Box (AABB)
- **Data Structure**: Stores minimum corner `minimum` and maximum corner `maximum`.
- **Intersection Routine**: Implements Andrew Kensler's slab algorithm using inverse direction components `invD = 1.0 / ray.direction()[a]`.
- **Robustness**: IEEE 754 division by zero yields `+inf` / `-inf`, correctly handled during interval swapping (`t0`, `t1`). Minimum padding (`delta = 0.0001`) prevents zero-thickness planar bounding boxes.

### 2. Bounding Volume Hierarchy (BVH)
- **Data Structure**: Binary tree (`BVHNode`) containing an `AABB box` and smart pointers to `left` and `right` child `Hittable` instances.
- **Construction Strategy**:
  1. Computes total bounding box for primitive collection span `[start, end)`.
  2. Selects split axis matching the longest dimension of the enclosing bounding box (`box.longest_axis()`).
  3. Sorts primitive objects by bounding box centroids along the selected axis.
  4. Splits span at midpoint `mid = start + span / 2` and recursively constructs child nodes.
  5. Leaves contain 1 or 2 primitive objects.
- **Complexity Characteristics**:
  - *Expected Average Case*: For well-separated geometry, BVH traversal exhibits $O(\log N \cdot R)$ ray-box tests per ray.
  - *Worst Case*: In degenerate cases with extreme spatial overlap or unaligned bounds, traversal degrades toward $O(N \cdot R)$ linear testing.
- **Traversal & Pruning**:
  - `BVHNode::hit` first executes `box.hit(r, ray_tmin, ray_tmax)`. If miss, branch terminates immediately.
  - If hit, tests `left` child; if `left` hits at distance $t_{\text{left}}$, updates `ray_tmax` to $t_{\text{left}}$ when testing `right` child to prune farther branches.

---

## Multithreaded Work Scheduling & Determinism Strategy

1. **Row Partitioning**:
   - Total image height $H$ is partitioned into disjoint contiguous row ranges $[r_{\text{start}}, r_{\text{end}})$ across $N$ worker threads (`std::thread`).
   - Each pixel location $(i, j)$ in the image buffer is written to by exactly one thread, eliminating data races without mutex locking during pixel write operations.

2. **Order-Independent Deterministic PRNG**:
   - Seed mixing formula: `sample_seed = SplitMix64(global_seed, i, j, s)`.
   - Because each sample $(i, j, s)$ initializes its own deterministic local `RNG` instance, thread scheduling and execution order have zero effect on random streams or floating-point calculations.
   - Outputs rendered across 1, 2, 4, 8, or 16 threads are 100% byte-for-byte identical.

3. **Thread-Local Statistics Aggregation**:
   - Each worker thread maintains an independent `RenderStats` accumulator (`thread_stats[t]`), avoiding shared lock or atomic counter contention on the hot path.
   - Main thread merges thread-local statistics (`total_stats.merge(thread_stats[t])`) after all worker threads finish.

---

## Camera Optics: Orientable Pinhole vs Thin-Lens DOF

1. **Orientable Camera Basis**:
   - Camera position (`lookfrom`), target (`lookat`), and up vector (`vup`).
   - Orthonormal basis vectors: $w = \text{unit}(\text{lookfrom} - \text{lookat})$, $u = \text{unit}(\text{vup} \times w)$, $v = w \times u$.

2. **Aperture & Thin-Lens Model**:
   - Parameter `aperture` specifies lens diameter; internal `lens_radius = aperture * 0.5`.
   - **Pinhole Path (`aperture == 0.0`)**: Ray originates directly at `lookfrom`.
   - **Thin-Lens Path (`aperture > 0.0`)**: Ray origin is offset on lens disk ($u \cdot r_x + v \cdot r_y$), targeted at focus point on focus plane at distance `focus_dist`.
