# Project 03 Benchmarks — Performance Lab

Benchmark protocols, measurement methodologies, and performance suite descriptions for CPU cache locality, SIMD vectorization, lock-free queues, and memory allocation.

---

## Benchmarking Protocol & Methodology

All micro-benchmarks in Project 03 execute under strict, scientific timing conditions to guarantee accurate and reproducible performance data.

### Measurement Framework Rules

1. **High-Precision Clock Source**:
   - Uses standard C++20 `std::chrono::high_resolution_clock` or native OS high-resolution timers (`QueryPerformanceCounter` on Windows, `clock_gettime(CLOCK_MONOTONIC)` on Linux).

2. **Warmup Protocol**:
   - Each benchmark workload executes **5 warmup iterations** before recording timing measurements to populate L1/L2 CPU caches and stabilize CPU frequency scaling.

3. **Repeated Timed Iterations**:
   - Each workload executes **100 timed iterations**.

4. **Statistical Metrics Reported**:
   - **Mean Time**: Arithmetic average time per iteration ($\mu\text{s}$ or $\text{ns}$).
   - **Median Time**: 50th percentile execution time ($\mu\text{s}$ or $\text{ns}$).
   - **Standard Deviation**: Measure of statistical variance.
   - **Minimum Time**: Lowest recorded execution time.
   - **Maximum Time**: Highest recorded execution time (identifies OS scheduling latency spikes).
   - **Throughput**: Calculated operations per second ($\text{Ops/sec}$) or bandwidth ($\text{MB/sec}$).

5. **Optimization Barrier Enforcement**:
   - Benchmark loops wrap output results in memory barriers or volatile writes to prevent compiler dead-code elimination.

6. **Release Build Requirement**:
   - Canonical benchmark reports are gathered strictly from **Release builds** (`/O2` optimization, `NDEBUG`).

---

## Benchmark Suite Overview

### 1. Milestone 2: Cache Locality & Data Layout (`bench_cache_locality`)
- **Workload A**: AoS vs SoA vs AoSoA 3D position vector updates (100,000 items).
- **Workload B**: Filtered property queries (single float attribute read across 100,000 items).
- **Workload C**: Strided access patterns (stride 1, 2, 4, 8, 16, 32, 64 floats).

### 2. Milestone 3: SIMD Vectorization (`bench_simd_vectorization`)
- **Workload A**: Batch 3D vector dot products (Scalar vs Auto-vectorized vs AVX2 8-way SIMD).
- **Workload B**: Batch Ray-AABB bounding box intersections (Scalar vs AVX2 4-way SIMD).

### 3. Milestone 4: Lock-Free Concurrency Queues (`bench_lockfree_queues`)
- **Workload A**: Single-Producer Single-Consumer throughput (`SPSCQueue` vs `MutexQueue`).
- **Workload B**: Multi-Producer Multi-Consumer contention scaling across 2, 4, 8, and 16 worker threads (`MPMCQueue` vs `MutexQueue`).

### 4. Milestone 5: Memory Allocators (`bench_allocator_churn`)
- **Workload A**: High-frequency small-object allocation churn (1,000,000 allocations).
- **Workload B**: `std::allocator` vs Capacity Reservation vs Linear `ArenaAllocator`.

---

## Benchmark Report Directory Structure

Benchmark execution text logs and CSV reports are saved to:
`projects/03-performance-lab/benchmarks/reports/`
