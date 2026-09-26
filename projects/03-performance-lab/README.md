# Project 03: Performance Lab

A hardware-aware micro-benchmarking laboratory dedicated to CPU cache locality optimization, SIMD vectorization (AVX2 / SSE4.2 / NEON intrinsics), atomic lock-free queue concurrency, and memory allocation efficiency.

---

### Current Status: Phase 0 Complete (Architecture & Design Phase)

Phase 0 establishes the technical roadmap, architectural boundaries, micro-benchmarking framework design, and milestone breakdown for Project 03 — Performance Lab. No source code or executable binaries have been implemented yet.

---

## Technical Focus & Objectives

1. **Cache Locality & Data Layout**:
   - Compare Array of Structures (AoS), Structure of Arrays (SoA), and Array of Structures of Arrays (AoSoA) across linear scanning and random spatial access patterns.
   - Quantify cache-line utilization (64-byte L1/L2 strides) and memory bandwidth efficiency.

2. **SIMD Vectorization & Compiler Intrinsics**:
   - Evaluate scalar C++ math routines against compiler auto-vectorization and explicit AVX2 / SSE4.2 SIMD compiler intrinsics (`<immintrin.h>`).
   - Benchmark high-throughput geometric kernels including 4-way parallel batch dot products and 4-way parallel ray-AABB bounding box intersection tests.

3. **Multi-Threaded Concurrency & Lock-Free Work Queues**:
   - Measure lock contention overhead in mutex-synchronized work queues under 1, 2, 4, 8, and 16 worker threads.
   - Design and benchmark lock-free atomic single-producer single-consumer (SPSC) and multi-producer multi-consumer (MPMC) ring buffer queues.

4. **Memory Allocator Churn & Cache Locality**:
   - Measure dynamic heap allocation and deallocation churn (`malloc`/`free`, `new`/`delete`, `std::allocator`).
   - Benchmark linear arena / bump allocators against system heap allocators for transient batch workloads.

5. **Reproducible Micro-Benchmarking Protocols**:
   - Implement high-precision, low-overhead micro-benchmarking harness with statistical variance metrics (mean, median, standard deviation, min, max, throughput ops/sec).

---

## Relationship to Projects 01 & 02

- **Project 01 (Ray Tracer)**: Established first-principles ray tracing, bounding volume hierarchies (BVH), and material evaluation.
- **Project 02 (Voxel Engine)**: Established compact 3D spatial grids, greedy meshing, multithreaded snapshot generation, and zero-allocation mesh buffer recycling.
- **Project 03 (Performance Lab)**: Isolates low-level hardware performance characteristics observed in Projects 01 and 02 into pure, standalone micro-benchmarks. Project 03 builds its own minimal benchmark kernels without linking or depending on Project 01 or Project 02 source code.

---

## Planned Milestone Roadmap

- [x] **Phase 0**: Discovery, Technical Roadmap & Architecture Review
- [ ] **Milestone 1**: Micro-Benchmarking Harness & High-Precision Timing Infrastructure
- [ ] **Milestone 2**: Cache Locality & Data Layout Benchmarks (AoS vs SoA vs Stride Access)
- [ ] **Milestone 3**: SIMD Vectorization & Intrinsic Acceleration (AVX2 / SSE4.2 Vector & Geometry Kernels)
- [ ] **Milestone 4**: Thread Contention & Lock-Free vs Mutex Synchronization Queues
- [ ] **Milestone 5**: Memory Allocator Churn & Arena / Bump Allocator Benchmarks

---

## Scope Boundaries & Explicit Non-Goals

- **No Speculative Frameworks**: Shared libraries are NOT created upfront in `libs/`. All code remains strictly isolated in `projects/03-performance-lab/`.
- **No Third-Party Benchmarking Frameworks**: Micro-benchmarking relies strictly on standard C++20 `std::chrono` and native OS timer primitives to maintain lightweight, transparent execution.
- **No GPU Compute / Shaders**: GPU performance, compute shaders, and VRAM memory profiling are deferred to Project 05 (Crater Simulator).
- **No Monolithic Engine Abstractions**: Focus is entirely on isolated micro-benchmarks and hardware measurement.

---

## Build & Test Instructions

```bash
# Configure build
cmake --preset default

# Build all Project 03 targets
cmake --build --preset default

# Run test suite via CTest
ctest --preset default --output-on-failure
```
