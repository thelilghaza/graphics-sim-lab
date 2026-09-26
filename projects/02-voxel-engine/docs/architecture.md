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

Meshing will follow the empirical baseline-first methodology:

1. **Milestone 4 — Naive Exposed-Face Culling (Baseline)**:
   - Iterates through all $32^3$ voxels in a chunk.
   - For each solid voxel, checks all 6 neighbor positions (+X, -X, +Y, -Y, +Z, -Z).
   - Generates quad vertices only for faces bordering air voxels or world boundaries.
   - Serves as the correctness oracle and meshing performance baseline.

2. **Milestone 6 — Greedy Meshing Algorithm**:
   - Sweeps 2D slices along each axis.
   - Merges adjacent coplanar quad faces sharing identical material types into larger rectangular quads.
   - Reduces quad and vertex counts by 60%–80%, significantly cutting GPU vertex processing and index memory overhead.

---

## 7. Data vs Rendering Backend Isolation

The voxel storage classes (`Chunk`, `WorldGrid`) and surface meshers (`NaiveMesher`, `GreedyMesher`) must remain 100% independent of any graphics API or windowing library.

```text
[ Voxel Data (Chunk / WorldGrid) ]
              │
              ▼
[ Surface Mesher (Naive / Greedy) ] ──> Generates MeshBuffer (Vertices + Indices)
              │
              ▼
[ Renderer / Visualization Backend ] (Minimal OpenGL / Native Layer)
```

This clean separation ensures voxel algorithms can be unit tested and benchmarked headless without windowing dependencies.

---

## 8. Multithreading & Future Considerations

- **Embarrassingly Parallel Chunk Pipelines**: Chunk terrain generation (procedural noise) and surface mesh extraction are independent per chunk $(CX, CY, CZ)$.
- **Thread Safety**: Worker threads receive read-only references to neighbor voxel data during mesh generation, producing isolated `MeshBuffer` outputs without shared mutable state locks.
