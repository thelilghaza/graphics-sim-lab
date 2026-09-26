# Project 02: Voxel Engine

A high-performance C++20 voxel engine focused on volume representation, spatial locality, memory layout, surface extraction (naive culling & greedy meshing), and multithreaded chunk management.

---

## Current Status: Milestone 7 Complete

Milestones 1–6 established the compact voxel payload, contiguous chunk storage, deterministic world coordinate conversion, abstract `WorldAccessor`, deterministic test-world generators, naive exposed-face mesher baseline, minimal OpenGL 3.3 Core visualization, and greedy meshing with quad reduction.

Milestone 7 implements a single-threaded dynamic chunk manager (`ChunkManager`) providing deterministic distance-based streaming around the camera position using Chebyshev distance with separate load and unload radii (hysteresis). It adds a continuous deterministic procedural terrain generator, dynamic neighbor mesh invalidation across chunk boundaries (X, Y, Z, and negative coordinates), OpenGL viewer integration with Scene 4 (Dynamic Streaming World) and real-time title status, a comprehensive 12-test suite (`test_chunk_manager`), and dedicated streaming benchmarks (`bench_chunk_manager`).

---

## Implemented Behavior

### Milestone 1 — Compact Voxel Payload & Contiguous Storage
- **Compact Voxel Payload**: `Voxel` struct containing `uint8_t type_id` and `uint8_t flags` (`sizeof(Voxel) == 2` with `static_assert`).
- **Contiguous Chunk Storage**: `Chunk` storing exactly $32 \times 32 \times 32 = 32,768$ voxels in a contiguous 64 KB array (`sizeof(Chunk) == 65536`) with zero heap indirection or per-voxel pointers.
- **Index Arithmetic**: Local coordinate indexing via `index = lx + ly * 32 + lz * 1024` for $0 \le lx, ly, lz < 32$.
- **Storage API**: `get_voxel`, `set_voxel`, `fill`, `clear`, `get_voxel_at_index`, `set_voxel_at_index`, and bounds validation (`in_bounds`).

### Milestone 2 — World Coordinates & Cross-Chunk Access
- **World-to-Local Decomposition**: Utilities for decomposing any integer world coordinate $W$ into chunk coordinate $C$ and local coordinate $L$ satisfying $W = C \times 32 + L$ with $0 \le L < 32$.
- **Negative Coordinate Floor Division**: Integer-only floor division formula `floor_div_32(W) = (W < 0) ? ((W - 31) / 32) : (W / 32)` and modulo formula `floor_mod_32(W) = ((W % 32) + 32) % 32` ensuring deterministic negative space mapping without floating-point conversion.
- **Coordinate Types**: Lightweight, explicit coordinate types `WorldCoord`, `ChunkCoord`, and `LocalCoord` for type-safe spatial queries.
- **WorldAccessor Abstraction**: Abstract `WorldAccessor` interface for querying world-space voxels (`get_voxel`, `set_voxel`, `has_chunk`, `has_voxel`) across chunk boundaries without exposing internal chunk storage layout.
- **WorldGrid Container**: Concrete `WorldGrid` storage container backed by `std::map<ChunkCoord, Chunk>` for managing loaded chunks and auto-allocating chunks on write.
- **Missing-Chunk Behavior**: `WorldGrid::get_voxel(w)` returns default air `Voxel(0,0)` for unpopulated chunks, while `get_voxel(w, out)` returns `false` and `has_voxel(w)` returns `false`.

### Milestone 3 — Voxel Editing & Deterministic Test Worlds
- **Voxel Editing API**: `clear_voxel` for setting voxels to air `Voxel(0,0)`, `is_solid` query helper (`type_id != 0`), `fill_box` for region filling, and `count_solid_voxels()` on `WorldGrid`.
- **Default Air Representation**: Defined explicitly as `Voxel(0, 0)` with `type_id == 0`. Unpopulated chunks fall back to air on read.
- **Deterministic Test World Generators**:
  - `generate_solid_world`: Fills a finite bounding box with a chosen solid voxel.
  - `generate_plane_world`: Generates a planar surface along X, Y, or Z axis with integer boundary checks ($c \le \text{plane\_coord}$).
  - `generate_sphere_world`: Generates a sphere using integer squared-distance test ($dx^2 + dy^2 + dz^2 \le r^2$) without floating-point operations.
- **Cross-Chunk Test Cases**: Verified test geometry (planes, spheres) crossing chunk boundaries ($X/Y/Z = 31/32$, negative boundaries $W = -1/0$).
- **Determinism Verification**: Independent generation runs verified for 100% bitwise exact voxel equality.

### Milestone 4 — Naive Exposed-Face Culling Mesher & CPU Mesh Buffer
- **Renderer-Independent CPU Mesh Buffers**: `MeshVertex` (position + normal) and `MeshData` (contiguous vertices and 32-bit indices) completely isolated from graphics APIs.
- **Naive Exposed-Face Culling**: `mesh_chunk` evaluates all $32,768$ voxels per chunk, testing 6 neighbor directions via `WorldAccessor`. Emits quads only when adjacent to air or missing chunks.
- **Cross-Chunk Boundary Culling**: Shared internal faces crossing chunk boundaries are transparently culled via `WorldAccessor`.
- **Exact Baseline Counts**: Full solid $32^3$ chunk emits exactly $6,144$ faces ($24,576$ vertices, $36,864$ indices).
- **Benchmark Suite**: `bench_naive_mesher` measuring mesh generation times across empty, single-voxel, full-solid, plane, and sphere worlds.

### Milestone 5 — Minimal OpenGL Visualization & Interactive Camera
- **OpenGL 3.3 Core Profile**: Minimal context created using GLFW 3.4 with an embedded modern OpenGL function loader (`init_gl_loader`).
- **Clean Architecture Separation**: CPU voxel simulation and meshing remain 100% graphics-free; `GLMesh` takes CPU `MeshData` and uploads to GPU VBO/EBO/VAO buffers.
- **Directional Lighting Shader**: Minimal vertex and fragment shaders computing ambient + diffuse directional lighting from face normals with uniform base colors.
- **Interactive 3D Camera**: Free-fly camera (3-axis translation with pitch/yaw look) with delta-time keyboard movement (W/A/S/D/Q/E), smooth pitch/yaw mouse look, and reset functionality.
- **Visual Test Scenes**: Interactive scene switching (keys 1/2/3) across:
  - Scene 1: Full Solid Chunk ($32^3$)
  - Scene 2: Planar World ($y \le 15$)
  - Scene 3: Cross-Chunk Sphere (radius 12 centered at $(31,31,31)$ spanning 8 chunks)
- **Viewer Executable**: `voxel_viewer` with real-time FPS counter, window title stats, and optional automated test flags (`--test-all-scenes`, `--test-frames`, `--headless`).

### Milestone 6 — Greedy Meshing Algorithm & Performance Analysis
- **Greedy Meshing Algorithm**: Standard greedy 2D slice-mask algorithm (`greedy_mesh_chunk`) iterating across all six principal face directions (`PosX`, `NegX`, `PosY`, `NegY`, `PosZ`, `NegZ`). For each 2D slice, visible exposed faces are identified and merged into maximal deterministic rectangles.
- **Material & Semantic Compatibility**: Adjacent faces merge if and only if they share identical voxel `type_id`. Faces with different `type_id` values remain separate quads, preventing invalid cross-material merging.
- **Deterministic Geometry & Conventions**: Maintains 4 vertices and 6 indices per emitted quad, outward-facing axis-aligned normals, deterministic vertex order, and deterministic CCW triangle winding. Identical inputs yield bitwise identical vertex and index buffers.
- **Rigorous Surface Equivalence**: Canonical unit-face decomposition verifies that greedy meshes describe the exact same visible surface as naive exposed-face meshes with zero holes or overlapping quads.
- **Geometry Reduction**:
  - Full Solid $32^3$ Chunk: Collapses from 6,144 naive faces to exactly 6 greedy quads (99.90% face/vertex/index reduction).
  - Planar World ($y \le 15$): Collapses from 4,096 naive faces to 6 greedy quads (99.85% reduction).
  - Sphere World ($r=12$): Drops from 2,646 faces to 1,050 quads (60.32% reduction).
  - Mixed-Voxel Stripes: Drops from 4,096 faces to 34 quads (99.17% reduction).
- **Computational Cost Analysis**: Greedy meshing requires 2D mask construction and maximal rectangle expansion, resulting in higher CPU generation time (~1.5-2.9 ms vs ~0.3-1.8 ms naive), trading generation time for up to 99.9% reduction in GPU vertex/index workload.
- **Viewer Integration**: Interactive keys `N` (Naive mesher) and `G` (Greedy mesher) switch meshing algorithms on the active scene in real time without altering camera position.

### Milestone 7 — Dynamic Chunk Manager & Distance-Based Streaming
- **Single-Threaded Chunk Manager**: `ChunkManager` orchestrates the residency and lifecycle of active voxel chunks around a camera or world position, coordinating allocation in `WorldGrid`, generation, meshing, and mesh eviction.
- **Distance-Based Streaming Policy**: Deterministic chunk-space Chebyshev distance ($L_\infty = \max(|dx|, |dy|, |dz|)$). Chunks within `load_radius` are loaded and meshed; chunks strictly beyond `unload_radius` are unloaded.
- **Boundary Hysteresis**: Separate configurable `load_radius` (default 2) and `unload_radius` (default 3) create a hysteresis buffer zone that eliminates thrashing and load/unload churn along boundary edges.
- **Deterministic Procedural Terrain Generator**: Continuous, noise-free integer triangle-wave elevation function based on integer coordinates. Bitwise identical contents guaranteed upon chunk unload and reload.
- **Boundary Neighbor Mesh Invalidation**: When any chunk loads or unloads, all 6 adjacent orthogonal neighbors (+X, -X, +Y, -Y, +Z, -Z) are marked dirty and remeshed. Culls internal shared boundary faces when neighbors appear, and regenerates exposed boundary faces when neighbors unload, preventing visual boundary holes.
- **Streaming Metrics**: Live metrics tracking `chunks_loaded_this_update`, `chunks_unloaded_this_update`, `total_chunks_loaded`, `total_chunks_unloaded`, `currently_loaded_chunks`, `chunks_meshed_this_update`, and total geometry counts.
- **OpenGL Viewer Integration**: Scene 4 (Dynamic Streaming World) tracks the free-fly camera in real time, streaming chunks dynamically as the camera flies across positive or negative space. Window title status displays current camera chunk coordinate, loaded chunk count, recent load/unload deltas, active mesher, and total geometry.
- **Unit Test Suite**: Dedicated `test_chunk_manager` verifying initial load, same-chunk movement, $\pm X/Y/Z$ boundary crossings, deterministic reload, hysteresis band preservation, missing-chunk semantics, neighbor mesh invalidation across all axes and negative space, duplicate load suppression, and deterministic coordinate sets.
- **Streaming Benchmark Suite**: `bench_chunk_manager` measuring single-threaded update throughput across initial population, single-chunk boundary moves, repeated 10-step crossings, and manual load/unload sequences for both Naive and Greedy meshing.

---

## Planned Milestone Roadmap

- [x] **Milestone 1**: Compact Voxel Payload & Contiguous $32^3$ Chunk Storage Architecture
- [x] **Milestone 2**: World Coordinate Conversion System & Cross-Chunk Neighbor Access API
- [x] **Milestone 3**: Basic Voxel Editing API & Deterministic Test Worlds (Solid, Empty, Sphere, Plane)
- [x] **Milestone 4**: Naive Exposed-Face Culling Mesher & Mesh Buffer Data Structure
- [x] **Milestone 5**: Minimal Visualization Layer & Interactive Camera
- [x] **Milestone 6**: Greedy Meshing Algorithm & Quad-Reduction Performance Analysis
- [x] **Milestone 7**: Dynamic Chunk Manager & Distance-Based Chunk Streaming
- [ ] **Milestone 8**: Multithreaded Chunk Generation & Parallel Mesh Extraction
- [ ] **Milestone 9**: Memory Footprint Optimization & Micro-Benchmarking Suite
- [ ] **Milestone 10**: Level of Detail (LOD) & Large-World Scale Experiments

---

## Scope Boundaries & Explicit Non-Goals
- **Single-Threaded Streaming**: Milestone 7 streaming executes synchronously on the main thread. Multithreaded chunk generation, asynchronous job queues, and parallel mesh extraction are deferred to Milestone 8.
- **No Optimizations to Greedy Meshing**: Milestone 7 preserves the exact Milestone 6 greedy meshing algorithm as an evaluation baseline without premature optimization.
- **No Level of Detail (LOD)**: Chunks stream at full $32^3$ resolution. Hierarchical LOD and distance-based downsampling are deferred to Milestone 10.
- **No Procedural Noise Libraries / Biomes**: Procedural terrain uses integer math; Perlin/Simplex noise, biomes, and infinite procedural simulation are non-goals.
- **No Chunk Disk Persistence**: Dynamic chunk management operates entirely in-memory.
- **Naive Mesher Retained**: The naive exposed-face mesher remains fully supported as the correctness baseline and comparison reference.
- **No Complex Gameplay Engine**: Audio, scripting, physics solvers, and ECS systems are not created here.

---

## Build & Test Instructions

```bash
# Debug Build
cmake --preset default
cmake --build --preset default
ctest --preset default --output-on-failure

# Release Build
cmake --preset release
cmake --build --preset release
ctest --preset release --output-on-failure

# Benchmark
./build/release/projects/02-voxel-engine/bench_greedy_mesher.exe

# Interactive Viewer
./build/release/projects/02-voxel-engine/voxel_viewer.exe
```
