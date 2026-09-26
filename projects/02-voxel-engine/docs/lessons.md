# Project 02 Lessons Learned — Voxel Engine

Observations, architecture trade-offs, performance analysis, and engineering decisions recorded during development.

---

## Milestone 1 Implementation Observations

1. **Voxel Payload Size & Alignment**:
   - Defining `Voxel` as two `uint8_t` members (`type_id` and `flags`) with default constructor guarantees `sizeof(Voxel) == 2` bytes and `std::is_trivially_copyable_v<Voxel> == true`.
   - `static_assert(sizeof(Voxel) == 2)` ensures compiler padding never bloats chunk storage overhead.

2. **Chunk Memory Allocation & Cache Line Contiguity**:
   - Using `std::array<Voxel, 32768>` inside `Chunk` produces an exact $65,536$-byte ($64 \text{ KB}$) contiguous memory block without dynamic heap allocations or pointer overhead.
   - Index arithmetic `lx + ly * 32 + lz * 1024` allows $O(1)$ coordinate lookup while iterating sequentially through memory along $X \to Y \to Z$ strides.

3. **Dual Error-Handling Access Strategy**:
   - `get_voxel(lx, ly, lz)` throws `std::out_of_range` for throwing callers, while `get_voxel(lx, ly, lz, out_voxel)` returns `bool` for non-throwing hot paths.

---

## Milestone 2 Implementation Observations

1. **Integer-Only Floor Division for Negative Coordinates**:
   - Standard C++ integer division `W / 32` truncates toward zero, causing negative coordinates in range $[-31, -1]$ to produce `chunk = 0` instead of `chunk = -1`.
   - Implementing explicit integer floor division `floor_div_32(W) = (W < 0) ? ((W - 31) / 32) : (W / 32)` and local coordinate modulo `floor_mod_32(W) = ((W % 32) + 32) % 32` ensures $W = C \times 32 + L$ with $0 \le L < 32$ across all positive and negative integer space.

2. **Abstract `WorldAccessor` & Concrete `WorldGrid`**:
   - Decoupling world voxel queries behind an abstract `WorldAccessor` interface allows future meshers and raycasters to read neighboring voxels across chunk boundaries without coupling to specific chunk storage backends.
   - `WorldGrid` uses `std::map<ChunkCoord, Chunk>` as a simple, deterministic in-memory chunk map suitable for unit testing and cross-chunk validation without introducing premature streaming architecture.

3. **Missing-Chunk Voxel Semantics**:
   - `WorldGrid::get_voxel(w)` returns default air `Voxel(0,0)` for missing chunks to simplify mesher boundary queries, while `get_voxel(w, out)` returns `false` to allow callers to distinguish unpopulated chunks from explicitly stored air voxels.

---

## Milestone 3 Implementation Observations

1. **Integer-Only Test World Generation**:
   - Generator functions (`generate_solid_world`, `generate_plane_world`, `generate_sphere_world`) use exact integer comparisons ($dx^2 + dy^2 + dz^2 \le r^2$) rather than floating-point math.
   - This eliminates floating-point rounding errors and cross-platform divergence, guaranteeing bitwise identical voxel outputs across runs.

2. **Decoupled Generator Interface**:
   - Generators operate on the abstract `WorldAccessor&` interface rather than concrete `WorldGrid` storage.
   - This allows test world generators to populate any past, present, or future `WorldAccessor` implementation seamlessly.

---

## Milestone 4 Implementation Observations

1. **Transparent Cross-Chunk Neighbor Querying via `WorldAccessor`**:
   - Operating through `WorldAccessor` inside `mesh_chunk` allows neighbor checks for boundary voxels (`lx=31` or `lx=0`) to query neighboring world coordinates `(w.x ± 1, w.y ± 1, w.z ± 1)` transparently.
   - Shared internal faces crossing chunk boundaries (e.g. `c0(31,10,10)` and `c1(0,10,10)`) are culled cleanly without special-case boundary branching inside the mesher.

2. **Full Solid Chunk Baseline Verification**:
   - A full solid $32^3$ chunk ($32,768$ voxels) surrounded by air emits exactly $6,144$ faces ($24,576$ vertices, $36,864$ indices).
   - This proves that all $6 \times 30 \times 32^2 = 576,000$ internal shared faces are culled, reducing emitted face counts by 96.9% compared to unculled geometry ($6 \times 32,768 = 196,608$ faces).

3. **Performance Baseline Benchmark**:
   - Naive exposed-face meshing runs in ~1.88 ms per full solid $32^3$ chunk (532 chunks/sec throughput on single-threaded CPU).
   - Serves as the empirical correctness and throughput benchmark for comparing against Milestone 6 Greedy Meshing.

---

## Milestone 5 Implementation Observations

1. **Decoupled Renderer & MeshData Pipeline**:
   - The CPU voxel storage (`Chunk`, `WorldGrid`) and meshing pipeline (`mesh_chunk`) remain 100% independent of OpenGL and GLFW.
   - `GLMesh` directly consumes `MeshData` (`MeshVertex` array and `uint32_t` indices) and uploads to GPU buffers via `glBufferData`. This allows any future rendering backend (e.g. Vulkan or software rasterizer) to plug into the exact same CPU pipeline without changing storage code.

2. **Self-Contained Embedded Modern OpenGL Loader**:
   - Rather than introducing large external GL loader generators, an embedded 120-line loader (`gl_loader.hpp`/`gl_loader.cpp`) queries function pointers via `glfwGetProcAddress`. This eliminated external package manager requirements and ensures zero link-time or runtime loader mismatches.

3. **Multi-Chunk Scene Aggregation**:
   - In cross-chunk scenes (such as the radius 12 sphere crossing 8 chunks), chunk-local vertex positions $[0.0, 32.0]$ are cleanly offset into world space using chunk coordinate offsets $(cx \times 32, cy \times 32, cz \times 32)$ into a unified `MeshData` buffer, allowing single-draw-call rendering of multi-chunk test geometry.

---

## Milestone 6 Implementation Observations

1. **Canonical Unit-Face Surface Equivalence Verification**:
   - Simply checking quad counts or vertex bounds is insufficient to prove greedy meshing correctness.
   - Decomposing every emitted greedy quad back into its constituent set of $1 \times 1$ canonical unit faces (`CanonicalUnitFace{dir, x, y, z}`) and performing a strict `std::set` equality assertion against the naive mesher's canonical faces rigorously proves that greedy meshing covers the exact visible surface with zero holes, zero extraneous faces, and zero overlapping quads.

2. **Computational Cost vs Geometric Reduction Trade-Off**:
   - Greedy meshing requires scanning 6 directions $\times$ 32 slices, populating $32 \times 32$ 2D slice masks, and performing 2D maximal rectangle expansion.
   - As a result, CPU generation time is higher (~1.5 to 2.9 ms per chunk in Release) compared to naive scanning (~0.3 to 1.8 ms per chunk).
   - In return, planar geometry experiences massive reduction: a full solid chunk drops from 6,144 faces (24,576 vertices) to 6 quads (24 vertices), a 99.90% reduction. Even curved spherical surfaces drop by ~60%. This radically diminishes GPU vertex transformation, rasterization setup, and VBO memory bandwidth.

3. **Material/Voxel-Type Compatibility Enforcement**:
   - Slice masks must store `type_id` rather than boolean visibility. During maximal rectangle expansion along $u$ and $v$, cells are only merged if `mask[u][v] == type`.
   - Verified that adjacent solid voxels with distinct `type_id` values (e.g. type 1 vs type 2) remain separate quads even when sharing a coplanar surface, preserving visual and material correctness.

---

## Milestone 7 Implementation Observations

1. **Boundary Neighbor Mesh Invalidation Necessity**:
   - Because exposed-face culling queries missing chunks as air, when chunk $B$ loads adjacent to chunk $A$, $A$'s exposed boundary faces become internal and must be culled. Conversely, when $B$ unloads, $A$'s shared boundary face must be regenerated immediately.
   - Failing to invalidate the 6 orthogonal neighbors upon chunk load leaves phantom internal faces; failing to invalidate on chunk unload creates visible see-through boundary holes in the world mesh.
   - Tested across X, Y, Z, and negative coordinate chunk boundaries for both Naive and Greedy meshers.

2. **Hysteresis Band Prevents Boundary Thrashing**:
   - Implementing separate `load_radius` (e.g. 2 chunks) and `unload_radius` (e.g. 3 chunks) creates a buffer ring where chunks remain resident.
   - When a camera moves back and forth across a chunk boundary, chunks in the hysteresis band are not repeatedly deallocated, regenerated, and remeshed, drastically reducing CPU frame time spikes.

3. **Incremental GPU Mesh Synchronization**:
   - Rebuilding the entire GPU world vertex buffer on every chunk boundary crossing is prohibitively expensive.
   - Maintaining individual `GLMesh` instances per chunk and incrementally uploading only dirty remeshed chunks (and destroying unloaded chunks) preserves interactive 60+ FPS rendering during continuous flight.

4. **Single-Threaded Baseline for Multithreading (Milestone 8)**:
   - Synchronous chunk streaming and meshing on the main thread takes ~138 ms (Naive) to ~310 ms (Greedy) when crossing a chunk boundary (loading 25 chunks and remeshing dirty neighbors).
   - This baseline clearly demonstrates the necessity of Milestone 8 asynchronous generation and background worker thread pools to eliminate frame drops during high-speed camera movement.

---

## Milestone 8 Implementation Observations

1. **Boundary Snapshot vs WorldGrid Locking**:
   - Locking `WorldGrid` with reader-writer locks or mutexes during worker meshing causes high lock contention because meshing a single chunk evaluates up to $6 \times 32,768 = 196,608$ neighbor checks.
   - Capturing only the 6 orthogonal boundary planes ($12 \text{ KB}$) on the main thread in $< 5 \ \mu\text{s}$ enables 100% lock-free execution on worker threads. Workers operate in complete isolation without mutex overhead or risk of deadlocks.

2. **Multithreaded Scaling Characteristics & Amdahl's Law**:
   - For batch generation and meshing workloads (e.g. initial population of 125 chunks), scaling is strong:
     - 1 worker: 1047.92 ms (Naive) / 2228.53 ms (Greedy)
     - 2 workers: 591.86 ms (1.77x) / 1550.35 ms (1.44x)
     - 4 workers: 352.77 ms (2.97x) / 770.44 ms (2.89x)
     - 8 workers: 207.86 ms (5.04x) / 417.00 ms (5.34x)
   - For small, sequential single-chunk load/unload operations (Workload D), thread dispatch overhead and synchronization dominate, producing ~1.0x scaling. This highlights the architectural rule: parallelize batched spatial regions; avoid per-voxel or micro-task thread dispatch.

3. **Stale Result Invalidation Under High-Speed Traversal**:
   - Rapid camera movement generates cascading load and unload requests for the same chunk before background workers finish computing.
   - In Workload C (repeated boundary crossings), over 500 stale jobs were submitted and safely discarded upon completion without resurrecting unloaded chunks or overwriting newer state.
   - Assigning a monotonic 64-bit generation version token to every chunk coordinate proved simple, robust, and completely race-free.

4. **Preserving Determinism Across Concurrent Schedulers**:
   - Because OS thread scheduling is non-deterministic, jobs complete in arbitrary order.
   - Pushing completed results into a thread-safe queue and integrating them on the main thread without sorting chunks by completion order preserves bitwise identical vertex buffers, index arrays, and quad counts regardless of worker count.

---

## Initial Design Decisions & Architecture Trade-offs

1. **Flat Contiguous 1D Chunk Array Selection**:
   - Chose flat contiguous 1D array (`voxels[lx + ly * 32 + lz * 1024]`) for chunk storage instead of pointer-based 3D arrays or sparse hash maps.
   - Trade-off: Allocates memory for empty volume within loaded chunks, but guarantees $O(1)$ random access, zero pointer chasing, and sequential memory access matching 64-byte CPU cache lines.

2. **Chunk Sizing ($32^3$ Voxel Volume)**:
   - Evaluated $8^3$, $16^3$, $32^3$, and $64^3$ chunk sizes.
   - Selected $32^3$ ($32,768$ voxels per chunk = 64 KB at 2 bytes per voxel) because it fits within modern CPU L1/L2 cache, minimizes boundary neighbor checks relative to volume, and provides optimal meshing update granularity.

3. **Negative Coordinate Handling in World Space**:
   - Arithmetic right shift (`Wx >> 5`) and explicit floor division formulas prevent truncating negative coordinates toward zero, ensuring negative world positions map correctly to negative chunk coordinates without coordinate aliasing bugs.

4. **Baseline-First Meshing Strategy**:
   - Planned Naive Exposed-Face Culling in Milestone 4 before implementing Greedy Meshing in Milestone 6.
   - Follows the repository engineering principle: establish a clear correctness baseline and measure throughput before introducing advanced optimizations.

---

## Rejected Approaches

1. **Sparse Hash Maps (`std::unordered_map<IVec3, Voxel>`) for Chunk Volume Storage**:
   - Rejected due to node pointer memory overhead and hash collision cache misses during surface extraction loops.
2. **Large Per-Voxel Struct Payloads (64+ Bytes)**:
   - Rejected because large structs scale poorly to millions of voxels (16 GB RAM for 268M voxels vs 512 MB for 2-byte payload).
