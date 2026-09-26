# Project 02 Architecture — Voxel Engine

This document details the architectural design, memory layouts, spatial coordinate systems, surface extraction strategies, and performance considerations for the Voxel Engine.

---

## 1. Voxel Data Representation Evaluation

The core requirement of a voxel storage system is balancing memory footprint, random access speed ($O(1)$), linear cache locality, and simplicity for dynamic editing.

### Candidate Representations Evaluated

1. **Flat Contiguous 1D Array (`voxels[lx + ly * 32 + lz * 1024]`) — SELECTED FOR MVP**
   - **Pros**: $O(1)$ index arithmetic, contiguous memory layout matching CPU cache lines, zero pointer overhead, optimal SIMD/memcpy operations, trivial serialization.
   - **Cons**: Allocates memory for entire chunk volume regardless of solid/air density ratio.
   - **Verdict**: Optimal choice for chunked volumetric storage where active terrain chunks contain high solid volume.

2. **3D Array Representation (`Voxel voxels[32][32][32]`)**
   - **Pros**: Natural 3D indexing syntax.
   - **Cons**: Sub-array pointers incur pointer-chasing overhead and break contiguous memory alignment unless represented as a single flat block.
   - **Verdict**: Equivalent to flat array if contiguous, but flat 1D array with explicit index stride functions is preferred for C++ clarity.

3. **Sparse Hash Storage (`std::unordered_map<IVec3, Voxel>`)**
   - **Pros**: Minimal memory footprint for sparse, isolated, or scattered voxels.
   - **Cons**: High memory overhead per entry (hash node pointers, bucket overhead), hash collision costs, terrible cache locality, slow neighbor lookup during meshing.
   - **Verdict**: Rejected for terrain chunk storage due to severe meshing cache misses.

4. **Palette / Run-Length Encoded (RLE) Compression**
   - **Pros**: Reduces RAM footprint for uniform chunks (e.g., thousands of identical air or stone blocks).
   - **Cons**: Random writes require shifting memory or decompressing RLE runs; meshing requires decoding passes.
   - **Verdict**: Deferred to future chunk persistence/network compression milestones.

5. **Hierarchical Octree (Sparse Voxel Octree / SVO)**
   - **Pros**: Efficient spatial raycasting and LOD scaling over huge spatial distances.
   - **Cons**: High node pointer chasing overhead, expensive dynamic voxel modification (tree rebalancing/node splitting).
   - **Verdict**: Deferred to future LOD experiments after flat chunk storage baselines are established.

---

## 2. Chunk Dimensions & Memory Calculation

Selecting the chunk size $S \times S \times S$ directly impacts CPU cache line utilization, chunk mesh rebuild latency, draw call counts, and boundary neighbor checks.

### Memory Footprint per Chunk Size Candidates

| Chunk Size | Voxel Count ($S^3$) | 1 Byte / Voxel | 2 Bytes / Voxel (MVP) | 4 Bytes / Voxel | Cache Locality Profile |
|---|---|---|---|---|---|
| **$8^3$** | 512 | 0.5 KB | 1.0 KB | 2.0 KB | Fits in L1, but excessive chunk header & draw call overhead. |
| **$16^3$** | 4,096 | 4.0 KB | 8.0 KB | 16.0 KB | Fits in L1, but high cross-chunk boundary lookup ratio. |
| **$32^3$** | **32,768** | **32.0 KB** | **64.0 KB** | **128.0 KB** | **RECOMMENDED MVP**: Fits in L1/L2 cache; optimal meshing balance. |
| **$64^3$** | 262,144 | 256.0 KB | 512.0 KB | 1,024.0 KB | Exceeds L1 cache; higher re-meshing latency per single voxel edit. |

### Justification for $32^3$ Chunk Size
- At 2 bytes per voxel, a $32^3$ chunk occupies exactly **64 KB** (fits neatly into modern L2 CPU cache).
- Iterating through $32,768$ voxels in z-y-x order streams sequentially through contiguous 64-byte CPU cache lines without cache thrashing.
- Boundary surface voxels ($6 \times 32^2 = 6,144$ boundary faces) represent a low percentage (~18.7%) of total volume compared to $16^3$ chunks (~37.5%).

---

## 3. Compact Voxel Data Payload

To scale to large worlds containing millions of voxels, the per-voxel struct payload must remain minimal.

```cpp
struct Voxel {
    uint8_t type_id{0};  // 0 = Air, 1 = Stone, 2 = Dirt, 3 = Grass, etc.
    uint8_t flags{0};    // Bit flags: Bit 0 = Solid, Bits 1-3 = State, Bits 4-7 = Reserved
};
```

### Impact of Voxel Size on World RAM Footprint
A world grid of $32 \times 8 \times 32$ chunks ($1,024 \times 256 \times 1,024$ voxels = $268,435,456$ voxels):
- **64 bytes / voxel struct**: **16.0 GB RAM** (prohibitive).
- **4 bytes / voxel struct**: **1.0 GB RAM**.
- **2 bytes / voxel struct (MVP)**: **512 MB RAM** (efficient).
- **1 byte / voxel struct**: **256 MB RAM**.

---

## 4. World & Chunk Coordinate Conversion System

Converting world coordinates $(Wx, Wy, Wz)$ to chunk coordinates $(CX, CY, CZ)$ and local voxel coordinates $(lx, ly, lz)$ must handle negative coordinates correctly.

### Floor Division for Negative Space
Naive integer division in C++ (`Wx / 32`) truncates toward zero (e.g. `-1 / 32 = 0`), which incorrectly maps negative coordinates $-1 \dots -31$ to chunk index $0$ instead of chunk index $-1$.

For chunk size $S = 32 = 2^5$:

1. **Chunk Coordinate Formula**:
   - Arithmetic right shift (C++ signed integer shift):
     $$\text{CX} = Wx \gg 5$$
   - Equivalent arithmetic floor formula:
     $$\text{CX} = (Wx < 0) ? ((Wx - 31) / 32) : (Wx / 32)$$

2. **Local Voxel Coordinate Formula**:
   - Bitwise AND mask (for power of 2 size $S=32$):
     $$lx = Wx \& 31$$
   - Equivalent modulo formula:
     $$lx = ((Wx \% 32) + 32) \% 32$$

### Unit Test Verification Vectors
- $Wx = 0 \implies \text{CX} = 0, lx = 0$
- $Wx = 31 \implies \text{CX} = 0, lx = 31$
- $Wx = 32 \implies \text{CX} = 1, lx = 0$
- $Wx = -1 \implies \text{CX} = -1, lx = 31$
- $Wx = -32 \implies \text{CX} = -1, lx = 0$
- $Wx = -33 \implies \text{CX} = -2, lx = 31$

---

## 5. Boundary & Cross-Chunk Neighbor Access Architecture

During surface extraction (meshing), checking face visibility requires querying the neighbor voxel adjacent to each solid voxel face.

```text
Local Voxel (lx, ly, lz)
      │
      ├── (0 < lx < 31) ──> Query local chunk array at (lx ± 1, ly, lz)
      │
      └── (lx == 31)    ──> Boundary query! Requires neighbor chunk (CX + 1, CY, CZ) at local lx = 0
```

### Decoupled `WorldAccessor` Interface
To prevent tight coupling between the mesher, chunk storage, and rendering backend, cross-chunk queries are resolved via an abstract interface:

```cpp
class WorldAccessor {
public:
    virtual ~WorldAccessor() = default;
    virtual Voxel get_voxel(int world_x, int world_y, int world_z) const = 0;
};
```

---

## 6. Surface Extraction & Meshing Strategy

Meshing follows the empirical baseline-first methodology:

1. **Milestone 4 — Naive Exposed-Face Culling (Baseline - IMPLEMENTED)**:
   - Iterates through all $32^3 = 32,768$ local voxels `(lx, ly, lz)` in a chunk.
   - For each solid voxel, queries all 6 neighbor directions (+X, -X, +Y, -Y, +Z, -Z) via `WorldAccessor`.
   - Emits a quad (4 `MeshVertex` struct elements, 6 `uint32_t` indices) if and only if the neighbor is non-solid / air (`!world.is_solid(neighbor_w)`).
   - **Cross-Chunk Boundaries**: Neighbor queries cross local chunk boundaries transparently via `WorldAccessor`, correctly culling faces adjacent to solid voxels in neighboring chunks while emitting faces adjacent to unpopulated/missing chunks.
   - **Face Geometry Convention**:
     - Local voxel `(lx, ly, lz)` occupies unit cube $[lx, lx+1] \times [ly, ly+1] \times [lz, lz+1]$.
     - Vertex positions lie in chunk-local floating point range $[0.0, 32.0]$.
     - Outward-facing normals: $+X(1,0,0)$, $-X(-1,0,0)$, $+Y(0,1,0)$, $-Y(0,-1,0)$, $+Z(0,0,1)$, $-Z(0,0,-1)$.
     - Counter-clockwise (CCW) winding order looking at face from outside.
   - **Baseline Face Counts**:
     - Single Isolated Voxel: 6 faces, 24 vertices, 36 indices.
     - Two Adjacent Voxels: 10 faces, 40 vertices, 60 indices (shared face culled).
     - Full Solid $32^3$ Chunk: 6,144 faces, 24,576 vertices, 36,864 indices ($6 \times 30 \times 32^2 = 576,000$ internal shared faces culled).
   - Serves as the correctness oracle and meshing performance baseline.

2. **Milestone 6 — Greedy Meshing Algorithm (IMPLEMENTED)**:
   - **Algorithm Overview**: Standard greedy meshing across all 6 principal face orientations (`PosX`, `NegX`, `PosY`, `NegY`, `PosZ`, `NegZ`).
   - **2D Slice-Mask Construction**: For each of the 32 slices along the face normal axis, constructs a $32 \times 32$ 2D slice mask. A mask entry stores the voxel's `type_id` if the voxel is solid and its neighbor in the direction of the normal is non-solid (or missing).
   - **Maximal Deterministic Rectangle Expansion**: Iterates through mask cells $(u, v)$. For each non-zero cell, finds the maximal contiguous width $W$ of identical `type_id` along $u$, then extends height $H$ along $v$ as long as all cells in $[u, u+W-1] \times \{v+H\}$ match the identical `type_id`.
   - **Material Compatibility**: Merging requires exact `type_id` match. Faces with distinct voxel `type_id`s never merge into the same quad, preventing texture or material bleeding.
   - **Geometry Conventions**: Emits one quad per merged rectangle covering area $W \times H$. Preserves Milestone 4 geometry conventions: 4 vertices and 6 indices per emitted quad, outward-facing axis-aligned normals, and counter-clockwise winding order.
   - **Determinism**: Direction processing order is strictly fixed (`PosX` -> `NegX` -> `PosY` -> `NegY` -> `PosZ` -> `NegZ`). Slices and coordinates are iterated monotonically, ensuring bitwise deterministic vertex and index buffers.
   - **Surface Equivalence**: Validated by canonical unit-face decomposition (`extract_canonical_faces`), proving bitwise identical visible surface coverage compared to naive meshing across all test workloads.

---

## 7. Data vs Rendering Backend Isolation (Milestones 5 & 6 Implementation)

The voxel storage classes (`Chunk`, `WorldGrid`) and surface meshers (`mesh_chunk`, `greedy_mesh_chunk`) remain 100% independent of any graphics API or windowing library.

```text
[ Voxel Data (Chunk / WorldGrid) ]
              │
              ▼
[ Surface Mesher (Naive / Greedy) ] ──> Generates CPU MeshData (Vertices + Indices)
              │
              ▼
[ OpenGL Visualization Backend ] ──> Uploads to GPU (VBO, EBO, VAO) -> Render
```

### Visualization & Viewer Stack
1. **Windowing & Context**: GLFW 3.4 creates a minimal OpenGL 3.3 Core Profile context (`1280x720`).
2. **Function Loader**: Embedded lightweight function loader (`init_gl_loader`) dynamically loads OpenGL 3.3 function pointers via `glfwGetProcAddress` without external loader dependencies.
3. **GPU Resources (`GLMesh`)**: Generates and manages `VAO`, `VBO` (vertex buffer holding `MeshVertex`), and `EBO` (index buffer holding 32-bit indices) with clean lifetime management.
4. **Shading Model (`GLShader`)**: Directional lighting shader with ambient (35%) + diffuse (65%) lighting evaluated from face normal `vNormal` and light vector `uLightDir`.
5. **Interactive Camera (`Camera`)**: Free-fly camera (3-axis translation with pitch/yaw look) with WASDQE delta-time movement and smooth pitch/yaw mouse look.
6. **Deterministic Scenes (`SceneManager`)**: Provides real-time switching between Solid Chunk ($32^3$), Planar World ($y \le 15$), and Cross-Chunk Sphere (radius 12 across 8 chunks).
7. **Runtime Mesher Switching**: Live toggling between Naive (`N`) and Greedy (`G`) meshing dynamically regenerates the active scene mesh without changing camera position or scene definitions.

## 8. Dynamic Chunk Manager & Distance-Based Streaming Architecture (Milestone 7)

Milestone 7 introduces the `ChunkManager` to manage the lifecycle of resident voxel chunks in memory dynamically around an active camera or query position.

```text
Camera Position (World Space)
          │
          ▼
   ChunkManager (Residency & Lifecycle Coordinator)
     ├── Evaluates Chebyshev Distance: max(|dx|, |dy|, |dz|)
     ├── Unloads Chunks where dist > unload_radius
     ├── Allocates & Generates Chunks where dist <= load_radius
     │     └── Invokes Deterministic Chunk Generator
     ├── Invalidates 6 Orthogonal Neighbors on Load/Unload
     └── Remeshes Dirty Chunks (Naive or Greedy Mesher)
          │
          ▼
      WorldGrid (Underlying Voxel Chunk Storage)
          │
          ▼
  SceneManager (OpenGL Visualization Bridge)
     ├── Synchronizes Uploaded GLMesh Objects
     └── Draws Chunk Meshes with Per-Chunk Model Matrices: translate(coord * 32.0f)
```

### Streaming Policy & Chebyshev Distance Metric
- **Distance Metric**: Chebyshev distance ($L_\infty$ norm) in integer chunk coordinates:
  $$D_\infty(A, B) = \max(|A.x - B.x|, |A.y - B.y|, |A.z - B.z|)$$
  Chebyshev distance produces cubical chunk streaming regions of dimension $(2r + 1)^3$, ensuring uniform rendering distance along all Cartesian directions without directional pop-in artifacts.
- **Hysteresis Band**: Configurable `load_radius` ($r_{\text{load}}$, default 2) and `unload_radius` ($r_{\text{unload}}$, default 3).
  - A chunk coordinate $C$ is loaded when $D_\infty(C, C_{\text{cam}}) \le r_{\text{load}}$.
  - A resident chunk $C$ is unloaded only when $D_\infty(C, C_{\text{cam}}) > r_{\text{unload}}$.
  - The interval $[r_{\text{load}} + 1, r_{\text{unload}}]$ forms a hysteresis buffer zone. Small camera fluctuations along chunk boundaries do not trigger oscillatory load/unload cycles.

### Deterministic Terrain Generation
- All chunk content is generated deterministically via `generate_default_terrain_chunk` (or a custom `ChunkGenerator` callback).
- Employs a continuous integer triangle-wave elevation function based on global integer world coordinates `(wx, wz)`.
- Above elevation $y=26$, chunks are open sky (empty air). Below elevation $y=-6$, chunks are solid stone bedrock. Intersecting chunks contain stone, dirt, and grass voxels.
- Unloading and reloading the identical chunk coordinate guarantees bitwise identical voxel state.

### Boundary Neighbor Mesh Invalidation Rules
Because exposed-face culling and greedy meshing evaluate neighboring voxels through `WorldAccessor`, missing chunks are treated as air, exposing boundary faces.
1. **On Chunk Load**:
   - The newly loaded chunk $C$ is generated and marked dirty.
   - All 6 orthogonal neighbors $N \in \{C \pm (1,0,0), C \pm (0,1,0), C \pm (0,0,1)\}$ that are currently resident are marked dirty.
   - Remeshing $C$ and $N$ culls internal shared faces that are now occluded.
2. **On Chunk Unload**:
   - The chunk $C$ is removed from `WorldGrid` and its `MeshData` / `GLMesh` is freed.
   - All 6 orthogonal resident neighbors of $C$ are marked dirty and remeshed.
   - Exposed faces facing the newly empty space are restored, preventing visible boundary holes.

### Incremental GPU Mesh Synchronization
`SceneManager` tracks dirty and unloaded chunk lists from `ChunkManager`. During `update()`, it destroys `GLMesh` instances for unloaded chunks and updates/uploads `GLMesh` instances for remeshed chunks, maintaining high runtime performance without rebuilding the entire world GPU buffer.

---

---

## 9. Multithreaded Chunk Generation & Parallel Mesh Extraction (Milestone 8)

Milestone 8 parallelizes chunk procedural generation and CPU surface mesh extraction using a dedicated worker thread pool, maintaining strict data isolation, race-free snapshot consistency, versioned stale-job rejection, and decoupled main-thread OpenGL synchronization.

```text
Main / Render Thread                           Worker Threads (Fixed Pool)
────────────────────                           ───────────────────────────
1. Determine Desired Chunks (Chebyshev)
2. Evict Out-of-Radius Chunks
3. Capture 12 KB Neighborhood Snapshots
4. Dispatch ChunkBuildTasks ───────────────► 5. ThreadPool Task Queue
                                             6. Procedural Generation (Chunk)
                                             7. CPU Meshing (Naive / Greedy MeshData)
8. Completed Results Queue ◄──────────────── 8. Move ChunkBuildResult to Queue
9. Stale-Version & Residency Validation
10. Integrate Chunk into WorldGrid
11. Synchronize GLMesh Buffers (OpenGL 3.3)
12. Render Frame
```

### Worker Pool Architecture
- **Implementation**: Fixed-size `ThreadPool` utilizing standard C++20 facilities (`std::thread`, `std::mutex`, `std::condition_variable`, `std::queue`, `std::atomic`).
- **Worker Configuration Policy**: Configurable via `StreamingConfig::worker_count`.
  - Default: `std::max(1u, std::thread::hardware_concurrency() - 1)` (preserves one hardware thread for the main render thread).
  - Explicit: Configurable to any positive integer (e.g. 1, 2, 4, 8) or `0` for synchronous immediate debugging.
- **Graceful Lifecycle**:
  - `enqueue(task)`: Pushes work into the task queue and notifies a worker via condition variable.
  - `wait_idle()`: Blocks calling thread until all active tasks complete and the task queue is empty.
  - `stop()`: Signals shutdown flag, notifies all workers, joins all threads cleanly, and guarantees zero leaked threads or deadlocked shutdowns.

### Data Ownership & Memory Boundaries
To eliminate multi-threading data races, data ownership transitions through strict linear phases:
1. **Task Creation (Main Thread)**:
   - Captures an immutable `ChunkNeighborhoodSnapshot` from `WorldGrid`.
   - Constructs a `ChunkBuildTask` containing the target chunk coordinate, monotonic version token, meshing mode, and snapshot.
2. **Execution (Worker Thread)**:
   - The worker owns the task and snapshot exclusively.
   - If generating, writes directly into the snapshot's center chunk.
   - Runs CPU mesher (`mesh_chunk` or `greedy_mesh_chunk`), reading from the snapshot via `WorldAccessor` without locking or accessing `WorldGrid`.
   - Moves the generated `Chunk` and `MeshData` into a `ChunkBuildResult`.
   - Pushes the result onto a thread-safe completed queue (`std::mutex` protected).
3. **Integration (Main Thread)**:
   - Validates the result against current chunk residency and generation version.
   - Inserts the generated chunk into `WorldGrid`.
   - Stores `MeshData` and marks the chunk dirty for GPU upload.

### World Consistency & Snapshot Strategy
Worker meshing requires examining the adjacent boundary voxels of the 6 orthogonal neighbors (+X, -X, +Y, -Y, +Z, -Z). Reading directly from a live mutable `WorldGrid` across threads would risk reading half-updated chunks or require coarse locks that serialize workers.

Milestone 8 implements **Strategy B: Read-Only Neighborhood Snapshot**:
- `ChunkNeighborhoodSnapshot` captures only the boundary planes:
  - 6 faces $\times 32 \times 32$ voxels $\times 2$ bytes $= 12 \text{ KB}$ memory overhead.
- Captured synchronously on the main thread in $< 5 \ \mu\text{s}$ per chunk before worker dispatch.
- Inherits from `WorldGrid` / `WorldAccessor`: internal voxel reads query the local chunk buffer; boundary coordinate reads access the captured boundary arrays; missing neighbors evaluate as air.
- Workers mesh in 100% memory isolation with zero locking, zero cache contention, and zero race conditions.

### Stale Job Protection & Versioning
When a camera moves rapidly, chunks may be queued, unloaded, re-requested, or switched between Naive and Greedy meshing before a worker completes building them.
- A monotonic 64-bit counter (`next_version`) assigns a unique version token to each build request: `chunk_versions[coord] = v`.
- Upon completion, the main thread checks:
  1. Is the chunk coordinate still in `desired_chunks`?
  2. Does `result.version == chunk_versions[coord]`?
  3. Does `result.mesher_type == current_mesher_type`?
- If any check fails, the result is discarded immediately without corrupting `WorldGrid`, and `metrics.jobs_discarded_stale` is incremented.

### OpenGL Main-Thread Invariant
- **Strict Prohibition**: Worker threads **MUST NOT** and **DO NOT** execute any OpenGL calls, create/delete `GLMesh` instances, allocate VAO/VBO/EBO handles, compile shaders, or touch GLFW.
- **Main Thread Synchronization**: `SceneManager` handles all GPU resource updates during its regular `update(camera)` call on the main render thread. Completed CPU meshes are uploaded to GPU buffers sequentially, preserving graphics driver stability and context thread affinity.

### Determinism Guarantee
Because worker execution order is non-deterministic due to OS thread scheduling:
- Results are never sorted by completion order.
- Voxel coordinate generation formulas and mesher quad sorting remain strictly deterministic.
- Given identical seed parameters, chunk coordinates, and mesher type, the emitted `MeshData` (vertices, indices, quads) is bitwise identical regardless of worker count (1, 2, 4, or 8 workers).

---

## 10. Memory Footprint Optimization & Buffer Reuse Architecture (Milestone 9)

Milestone 9 eliminates dynamic memory churn and optimizes process memory consumption across continuous chunk streaming, neighbor invalidation, and multithreaded meshing.

```text
ChunkManager (Main Thread)
  ├── recycled_mesh_buffers (Bounded Pool: max 128 MeshData)
  │     ▲                                      │
  │     │ 4. Reclaims old/stale/evicted        │ 1. Passes pre-allocated buffer
  │     │    buffers via clear()               ▼
  │     └────────────── ChunkBuildResult ◄──── ChunkBuildTask
  │                                            │
  │                                            ▼
  │                                     Worker Thread
  │                                       ├── Exclusive Buffer Ownership
  │                                       ├── task.mesh.clear() (retains capacity)
  │                                       └── mesher writes directly (0 allocations)
  │
  └── WorldGrid (std::map<ChunkCoord, Chunk>)
        └── 64 KB contiguous Chunk per node (0.07% node overhead)
```

### Exact Memory Model & Formulas
1. **Raw Voxel Chunk**:
   - $32 \times 32 \times 32 \text{ voxels} \times 2 \text{ bytes} = 65,536 \text{ bytes}$ ($64 \text{ KB}$).
   - Stored contiguously in a single flat array without heap allocations or per-voxel pointers.
2. **Mesh Vertex & Quad Footprint**:
   - `sizeof(MeshVertex)`: exactly 24 bytes ($3 \times 4\text{B position} + 3 \times 4\text{B normal}$).
   - `sizeof(uint32_t)`: exactly 4 bytes.
   - Per Emitted Quad: 4 vertices ($4 \times 24\text{B} = 96\text{B}$) + 6 indices ($6 \times 4\text{B} = 24\text{B}$) $= 120\text{ bytes per quad}$.
3. **Representative Mesh Memory Profiles**:
   - Full Solid $32^3$ Chunk:
     - Naive: 6,144 quads $\to$ 24,576 vertices (589,824 B) + 36,864 indices (147,456 B) $= 737,280\text{ B}$ ($720\text{ KB}$).
     - Greedy: 6 quads $\to$ 24 vertices (576 B) + 36 indices (144 B) $= 720\text{ bytes}$.
   - Planar World ($y \le 15$):
     - Naive: 4,096 quads $\to$ 16,384 vertices (393,216 B) + 24,576 indices (98,304 B) $= 491,520\text{ B}$ ($480\text{ KB}$).
     - Greedy: 6 quads $\to$ 720 bytes.
   - Sphere World ($r=12$):
     - Naive: 2,646 quads $\to$ 10,584 vertices (254,016 B) + 15,876 indices (63,504 B) $= 317,520\text{ B}$ ($310\text{ KB}$).
     - Greedy: 1,050 quads $\to$ 4,200 vertices (100,800 B) + 6,300 indices (25,200 B) $= 126,000\text{ B}$ ($123\text{ KB}$).

### Allocation Hotspot Findings
Profiling dynamic streaming identified that the dominant source of heap churn was **vector reallocation during meshing**:
- Every time a chunk loaded or a neighbor remeshed, a fresh `MeshData` allocated empty vectors.
- Geometric vector reallocation ($0 \to 1 \to 2 \dots \to 24,576$) caused multiple `malloc`/`free` cycles per chunk.
- When chunks unloaded or remeshed, the old vector memory was released to the OS heap, only for new chunks to immediately request fresh buffers.
- Over 10 boundary crossings (Workload E), the unoptimized engine performed **2,250 dynamic buffer allocations**.

### Zero-Allocation Buffer Reuse Architecture
- `ChunkManager` maintains a bounded recycle pool (`recycled_mesh_buffers`, capped at 128 buffers).
- When a task is queued, `acquire_mesh_buffer()` pops a reusable `MeshData` and moves it into `ChunkBuildTask`.
- The worker thread takes exclusive ownership of `task.mesh`. It calls `task.mesh.clear()`, which resets `size` to 0 while keeping `capacity` intact.
- The mesher writes directly into the pre-allocated buffer with **zero dynamic heap allocations**.
- The worker moves `task.mesh` into `ChunkBuildResult`.
- On the main thread, if an existing mesh is replaced or unloaded, its old buffer is returned to the recycle pool via `recycle_mesh_buffer()`.
- Discarded stale tasks also return their buffers to the pool.
- During steady-state streaming (e.g. Workload F with 50 remesh cycles), **fresh allocations drop from 100 to 1** (99% allocation churn reduction).

### Chunk Pooling & WorldGrid Analysis
- **WorldGrid Container Evaluation**: `std::map<ChunkCoord, Chunk>` stores chunks ordered by coordinate.
  - Node size: key (12B) + value (65,536B) + 3 pointers (24B) + color (8B) $= 65,580\text{ bytes}$.
  - For 150 resident chunks, total raw chunk payload is $9.38\text{ MB}$. Total map node pointer overhead is only $7.2\text{ KB}$ (**0.07% overhead**).
  - Search cost is $O(\log N)$, requiring $\le 8$ pointer hops for 150 chunks.
- **Chunk Pooling Decision**:
  - `Chunk` contains a contiguous $64\text{ KB}$ array.
  - Chunks are only allocated/deallocated upon entering or leaving the streaming radius (25 chunks per boundary crossing), which is 100x less frequent than mesh operations.
  - Because node overhead is only 0.07% and chunk churn is minimal, adding a complex chunk pool to `WorldGrid` was measured to provide negligible benefit and would compromise the clean container abstraction.
  - **Verdict**: Chunk pooling in `WorldGrid` was investigated and deliberately rejected; the flat contiguous 64 KB storage is preserved as-is.

### Selective Chunk Copying
In Milestone 8, every task completion moved `task.snapshot.center_chunk` (64 KB) into `ChunkBuildResult`. During remeshing, generation was not performed, rendering the 64 KB move redundant.
- Milestone 9 restricts chunk copying: `if (task.need_generation) { res.chunk = std::move(task.snapshot.center_chunk); }`.
- Remesh tasks transfer only the mesh buffer, saving 64 KB of memory copies per remeshed chunk.

---

## 11. Future Architectural Roadmap (Milestone 10+)

- **Level of Detail (Milestone 10)**: Hierarchical octrees or downsampled chunk representations for extreme render distances.
