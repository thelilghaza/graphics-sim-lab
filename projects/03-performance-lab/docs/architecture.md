# Project 03 Architecture — Performance Lab

This document details the architectural design, technical objectives, data layouts, benchmarking methodology, and milestone plans for Project 03 — Performance Lab.

---

## 1. Problem Statement & Motivation

Modern CPU architectures feature deep memory hierarchies (L1, L2, L3 caches), SIMD vector execution units (AVX2, SSE4.2, NEON), and multi-core execution pipelines. High-level object-oriented programming abstractions frequently obscure the physical reality of hardware execution:

- Pointer-heavy object graphs cause L1/L2 cache misses.
- Array-of-Structures (AoS) data layouts prevent auto-vectorization and waste cache-line bandwidth.
- Mutex-based thread synchronization introduces thread context switches and lock contention spikes.
- Dynamic heap allocation churn (`malloc`/`free`) causes allocator fragmentation and locks heap mutexes.

Project 03 — Performance Lab provides an isolated, scientific micro-benchmarking environment to measure, analyze, and optimize these low-level CPU performance bottlenecks under controlled conditions.

---

## 2. Technical Objectives

1. **High-Precision Measurement**: Construct a deterministic micro-benchmarking harness (`BenchRunner`) capable of measuring nanosecond-level execution times and statistical variance without measurement overhead artifacts.
2. **Data Layout Optimization**: Measure the cache line utilization and memory throughput of Array of Structures (AoS) vs Structure of Arrays (SoA) vs Array of Structures of Arrays (AoSoA).
3. **SIMD Vectorization Efficiency**: Compare scalar C++ loop execution against compiler auto-vectorization (`/O2` / `-O3`) and explicit SIMD compiler intrinsics (`<immintrin.h>`).
4. **Lock-Free Concurrency**: Evaluate atomic single-producer single-consumer (SPSC) and multi-producer multi-consumer (MPMC) lock-free ring buffers against traditional `std::mutex` work queues under high multi-core contention.
5. **Memory Allocator Efficiency**: Benchmark custom linear arena / bump allocators against standard heap allocators for transient batch workloads.

---

## 3. Micro-Benchmarking Architecture & Framework Design

To ensure reliable, reproducible benchmark data, the benchmarking harness (`BenchRunner`) enforces strict execution protocols:

```text
+-------------------------------------------------------------------+
|                        BenchRunner Harness                        |
+-------------------------------------------------------------------+
|  1. Warmup Runs (5 iterations)   --> Populate L1/L2 CPU Caches    |
|  2. Timed Runs (100 iterations)  --> High-precision std::chrono   |
|  3. Cache Flush Utility          --> Evict L3 cache between tests |
|  4. Statistical Aggregation      --> Mean, Median, StdDev, Min    |
|  5. Formatted Output             --> CLI Table & CSV Export       |
+-------------------------------------------------------------------+
```

### Key Measurement Rules
- **Volatile Execution Barriers**: Use explicit memory barriers (`asm volatile` / `DoNotOptimize` wrappers) to prevent compiler optimization loops from eliminating benchmarked computation.
- **Warmup Phase**: Execute 5 un-timed warmup iterations before recording timing data to warm instruction and data caches.
- **Statistical Metrics**: Report mean time, median time, standard deviation, minimum time, maximum time, and throughput metrics (Operations/sec, MB/sec bandwidth).
- **Release-vs-Debug Verification**: Execute benchmarks in Release configuration (`/O2`, `NDEBUG`) as canonical baseline, with explicit side-by-side comparison against Debug builds to document compiler optimizations.

---

## 4. Data Layout & Cache Locality Evaluation

The spatial memory layout of data structures dictates how effectively CPU cache lines (64 bytes) are utilized.

### AoS vs SoA Comparison Model

```text
AoS Layout (Array of Structures):
[ Pos.x Pos.y Pos.z Vel.x Vel.y Vel.z Color Mass ] [ Pos.x Pos.y ... ]
|<-------------------- 32 Bytes ------------------>|

SoA Layout (Structure of Arrays):
Positions: [ PosX0 PosX1 PosX2 ... ] [ PosY0 PosY1 ... ] [ PosZ0 ... ]
Velocities:[ VelX0 VelX1 VelX2 ... ] [ VelY0 VelY1 ... ] [ VelZ0 ... ]
```

### Evaluated Workloads
- **Workload A (Linear Scan Position Update)**: Iterates over 100,000 particles updating position by velocity vector. Measures cache throughput when accessing contiguous floats versus reading padded structs.
- **Workload B (Filtered Property Query)**: Iterates over 100,000 particles checking a single flag or mass threshold. Demonstrates that SoA avoids loading unwanted struct fields into L1 cache lines.
- **Workload C (Random Spatial Access)**: Evaluates random index lookups to compare cache penalty when spatial locality is destroyed.

---

## 5. SIMD Vectorization Strategy

Modern CPUs execute SIMD instructions processing multiple 32-bit floating-point values simultaneously (4 floats in 128-bit SSE/NEON, 8 floats in 256-bit AVX2).

### Evaluated Vectorization Kernels
1. **Batch 3D/4D Vector Dot Products**:
   - Scalar implementation: Sequential loop evaluating $dx \cdot dy \cdot dz$.
   - Auto-vectorized implementation: Clean scalar loop compiled with `/O2` / `-O3`.
   - AVX2 Intrinsics: `_mm256_fmadd_ps` evaluating 8 parallel dot products per instruction cycle.
2. **Batch Ray-AABB Bounding Box Intersections**:
   - Scalar implementation: Slab method testing $t_{\text{min}}$ and $t_{\text{max}}$ sequentially.
   - AVX2 4-Way Parallel SIMD: Intersects 1 ray against 4 AABBs simultaneously using 128-bit / 256-bit vector registers.

---

## 6. Multi-Threaded Concurrency & Queue Architectures

Work dispatching across CPU worker threads requires thread-safe queues.

### Evaluated Queue Implementations
1. **`MutexQueue<T>`**: Standard `std::queue<T>` protected by `std::mutex` and `std::condition_variable`.
2. **`SPSCQueue<T>` (Single-Producer Single-Consumer)**: Lock-free atomic ring buffer using acquire-release memory ordering (`std::atomic<size_t>` head/tail pointers).
3. **`MPMCQueue<T>` (Multi-Producer Multi-Consumer)**: Lock-free atomic ring buffer utilizing atomic fetch-add or compare-and-swap (CAS) loops.

### Contention Testing Matrix
- 1 Producer / 1 Consumer (SPSC baseline)
- 2 Producers / 2 Consumers
- 4 Producers / 4 Consumers
- 8 Producers / 8 Consumers

---

## 7. Memory Allocator & Cache Churn Analysis

Dynamic heap allocation (`malloc` / `new`) incurs metadata management overhead and mutex synchronization inside OS heap allocators.

### Evaluated Allocator Strategies
1. **System Heap (`std::allocator`)**: Standard dynamic allocation per object/batch.
2. **Pre-Allocated Capacity Reuse**: Reserving vector capacity (`vector::reserve()` / `vector::clear()`).
3. **Linear Arena / Bump Allocator (`ArenaAllocator`)**: Pre-allocates a contiguous memory block (e.g. 1 MB). Allocations advance a single offset pointer (`bump_ptr += size`). Resetting the entire arena takes $O(1)$ time (`bump_ptr = 0`).

---

## 8. Deterministic Testing Strategy

Before performance measurements are gathered, functional correctness must be verified via automated CTest suites:

- **SIMD Correctness Tests**: Assert that SIMD vector dot products and ray-box intersection outputs match scalar calculation results within $10^{-5}$ floating-point tolerance.
- **Data Layout Equivalence**: Assert that AoS and SoA transformations yield bitwise identical numerical state.
- **Queue Concurrency & Invariants**: Assert FIFO ordering, no lost items, no double reads, and correct queue size accounting under multithreaded stress testing.
- **Arena Allocator Invariants**: Assert correct byte alignment (8-byte / 16-byte / 32-byte), out-of-memory handling, and clean arena resets.

---

## 9. Project 02 / Project 03 Architectural Boundaries

- **Strict Isolation**: Project 03 operates in `projects/03-performance-lab/`. It does NOT link, import, or depend on Project 01 or Project 02 source files.
- **No Premature Shared Code**: In accordance with the Delayed Shared Library Extraction Rule, no code is extracted into `libs/` during Phase 0.
- **Independent Execution**: Project 03 maintains its own CMake build target, unit test executable, and benchmark executables.
