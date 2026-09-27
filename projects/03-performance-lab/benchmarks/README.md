# Project 03 Benchmarks — Performance Lab

Benchmark protocols, measurement methodologies, and performance suite descriptions for CPU cache locality, SIMD vectorization, lock-free queues, and memory allocation.

---

## Benchmarking Protocol & Methodology

All micro-benchmarks in Project 03 execute under strict, scientific timing conditions to guarantee accurate and reproducible performance data.

### Measurement Framework Rules

1. **High-Precision Clock Source**:
   - Uses standard C++20 `std::chrono::steady_clock` to record monotonic iteration durations in microseconds.

2. **Warmup Protocol**:
   - Each benchmark workload executes **5 warmup iterations** before recording timing measurements to populate CPU caches.

3. **Repeated Timed Iterations**:
   - Each workload executes **100 timed iterations**.

4. **Statistical Metrics Reported**:
   - **Mean Time**: Arithmetic average time per iteration ($\mu\text{s}$).
   - **Median Time**: 50th percentile execution time ($\mu\text{s}$).
   - **Standard Deviation**: Sample standard deviation ($s$, $N-1$ denominator).
   - **Minimum & Maximum Time**: Extremes of measured iteration durations ($\mu\text{s}$).
   - **Throughput & Bandwidth**: $\text{Ops/sec}$ and $\text{MB/s}$ ($1 \text{ MB} = 10^6 \text{ bytes}$).

5. **Optimization Barrier Enforcement**:
   - Benchmark loops pass computed scalar values through `do_not_optimize()` and `clobber_memory()` to prevent compiler dead-code elimination.

6. **Release Build Requirement**:
   - Canonical benchmark reports are gathered strictly from **Release builds** (`/O2` optimization, `NDEBUG`).

---

## Benchmark Suite Overview

### Milestone 1: Harness Validation Baseline (`bench_harness`)
- **Workload A**: Integer Accumulation Loop (1,000,000 ops).
- **Workload B**: Floating-Point Accumulation Loop (1,000,000 ops).
- **Workload C**: Contiguous Array Traversal (500,000 uint64 elements = 4 MB data).
- **Workload D**: Deterministic Scalar Array Transformation (250,000 float elements).

### Milestone 3: SIMD Vectorization & Intrinsic Acceleration (`bench_simd_vectorization`)
- **Kernel A (Vector Fused Arithmetic)**: Evaluates $\text{sum} += a[i] \cdot b[i] + c[i]$ over 16K, 1M, and 16M float arrays across Scalar Ref (`#pragma loop(no_vector)`), Compiler Opt (`/O2`), Explicit SSE, and Explicit AVX2.
- **Kernel B (AXPY Vector Transform)**: Evaluates $y[i] = a \cdot x[i] + y[i]$ over 16K, 1M, and 16M float arrays across Scalar Ref, Compiler Opt, Explicit SSE, and Explicit AVX2. Measures memory bandwidth and throughput.
- **Kernel C (Batch Ray-AABB Geometry)**: Evaluates 4-wide and 8-wide packet Ray-AABB slab intersection testing across 50,000 ray packets (200,000 rays).

### Milestone 4: Thread Contention & Synchronization Queues (`bench_lockfree_queues`)
- **Suite 1 (SPSC Lock-Free Ring Buffer)**: Evaluates single-producer/single-consumer bounded lock-free ring buffer across capacities 64, 1024, and 16384 items (24-byte payload).
- **Suite 2 (MPMC Bounded Lock-Free Concurrency Scaling)**: Evaluates Dmitry Vyukov sequence-number ring-buffer algorithm across 1P1C, 2P2C, 4P4C, and 8P8C topologies at capacity 1024.
- **Suite 3 (Mutex Bounded Queue Baseline Scaling)**: Evaluates `std::mutex` + condition variable bounded queue under identical 1P1C, 2P2C, 4P4C, and 8P8C topologies at capacity 1024.
- **Suite 4 (Capacity Scaling under 4P / 4C Contention)**: Directly compares MPMC and Mutex queues across capacities 64 and 16384 under fixed 4P / 4C multi-threaded contention.

---

## Benchmark Report Directory Structure

Benchmark execution text logs and CSV reports are saved to:
`projects/03-performance-lab/benchmarks/reports/`

Generated reports:
- `harness_validation_release.csv` / `harness_validation_debug.csv`
- `cache_locality_release.csv` / `cache_locality_debug.csv`
- `simd_release.csv` / `simd_debug.csv`
- `queues_release.csv` / `queues_debug.csv`
