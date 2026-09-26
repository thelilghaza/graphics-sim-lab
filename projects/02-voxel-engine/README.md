# Project 02: Voxel Engine

A high-performance C++20 voxel engine focused on volume representation, spatial locality, memory layout, surface extraction (naive culling & greedy meshing), and multithreaded chunk management.

---

## Current Status: Milestone 2 Complete

Milestone 1 introduced the compact 2-byte `Voxel` payload and contiguous $32^3$ (`32,768` voxel) `Chunk` storage architecture ($65,536$ bytes).

Milestone 2 extends this foundation with deterministic integer world coordinate conversion, explicit coordinate types (`WorldCoord`, `ChunkCoord`, `LocalCoord`), abstract `WorldAccessor` interface, concrete `WorldGrid` container, and cross-chunk neighbor access testing.

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
- **Unit Test Suite**: CTest targets `test_voxel`, `test_chunk`, `test_coordinates`, and `test_world_accessor` verifying coordinate decomposition, negative boundaries, reconstruction invariants, missing-chunk behavior, and cross-chunk boundary reads/writes across X, Y, and Z axes.

---

## Planned Milestone Roadmap

- [x] **Milestone 1**: Compact Voxel Payload & Contiguous $32^3$ Chunk Storage Architecture
- [x] **Milestone 2**: World Coordinate Conversion System & Cross-Chunk Neighbor Access API
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
```
