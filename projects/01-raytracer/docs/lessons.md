# Project 01 Lessons Learned — CPU Ray Tracer

Observations, architecture trade-offs, performance analysis, and engineering decisions recorded during development.

---

## 💡 Milestone 5 Engineering Observations

1. **AABB Slab Ray-Box Intersection Robustness**:
   - Andrew Kensler's slab method computes inverse ray directions `invD = 1.0 / dir[a]`. IEEE 754 division by zero produces `+inf` / `-inf`, which correctly flips min/max intervals when using `std::swap(t0, t1)`.
   - Applying a small padding (`delta = 0.0001`) to bounding boxes ensures planar/degenerate objects (such as flat spheres or axis-aligned planes) have non-zero volume, preventing zero-thickness traversal failures.

2. **BVH Construction & Traversal Complexity Bounds**:
   - Splitting primitives along the longest axis (`longest_axis()`) based on bounding box centroids yields a simple, fast $O(N \log N)$ construction algorithm without third-party dependencies.
   - For a 50+ sphere procedural scene, BVH reduced primitive sphere intersection tests from **956 Million to 20.7 Million** (a **46.03x reduction**).
   - *Complexity & Trade-offs*: BVH traversal achieves $O(\log N \cdot R)$ expected average-case complexity for well-separated geometry, but can degrade toward $O(N \cdot R)$ in worst-case overlapping configurations. On small scenes (e.g. 5 spheres), linear traversal is slightly faster (~7%) due to node AABB overhead. Retaining `--accel naive` alongside `--accel bvh` provides an empirical reference oracle to verify when spatial acceleration is beneficial.

3. **Traversal Pruning with Closest Hit ($t$)**:
   - In `BVHNode::hit`, testing the `left` child first and updating `ray_tmax = rec.t` before testing the `right` child allows the right child AABB test to prune entire subtrees that lie farther than the closest hit established so far.

4. **Thread-Local Statistics Aggregation**:
   - Using per-thread local `RenderStats` accumulators merged at completion eliminated shared synchronization on the hot ray-tracing path.
   - Total ray and intersection counts reported by 1, 2, 4, 8, or 16 worker threads match single-threaded runs to the exact digit.

5. **Deterministic Thread Scaling**:
   - Combined with Milestone 4's `SplitMix64` per-sample PRNG seeding (`make_sample_seed`), multithreaded rendering achieves 100% byte-for-byte identical output PPM images regardless of thread count or thread execution timing (verified via SHA256 hashes across 1, 2, 4, 8, and 16 threads).
   - 16 worker threads combined with BVH acceleration reduced procedural scene render time from **5,346 ms to 345 ms** — achieving a **15.48x total performance speedup** over the single-threaded naive baseline.

6. **Compiler Optimization Impact (Debug vs Release)**:
   - Release configuration (`/O2 /Ob2 /DNDEBUG`) produced **4.22x to 8.03x lower render times** than the Debug configuration (`/Od /RTC1`) across all tested workloads.
   - Inlining small vector methods (`Vec3`), stripping runtime check overhead, and optimizing tight slab ray-AABB loops likely account for this significant performance delta.

---

## 💡 Milestone 4 Engineering Observations

1. **Order-Independent Per-Sample PRNG (`SplitMix64`)**:
   - Replacing global sequential PRNG consumption with SplitMix64 per-sample seed mixing (`make_sample_seed`) guarantees exact 100% determinism independent of pixel iteration order.

2. **Aperture & Pinhole Fast-Path**:
   - Distinguishing `aperture == 0.0` as an explicit pinhole fast-path avoids unnecessary random disk sampling calls and floating-point operations.

3. **Thin-Lens DOF Focus Distance**:
   - Scaling viewport dimensions by `focus_dist` ensures that changing focus distance does not alter camera framing or field of view.

4. **Diagnostic Ray Counter Breakdown**:
   - Centralizing ray tracking into `RenderStats` (`primary_samples`, `shadow_rays`, `secondary_rays`, `total_rays`) allows explicit throughput reporting.
