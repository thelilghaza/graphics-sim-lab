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

### Milestone 2: Cache Locality & Data Layout (`bench_cache_locality`)
- **Suite 1 (Sequential Traversal)**: Compares AoS (Array of Structures), SoA (Structure of Arrays), and AoSoA (Array of Structures of Arrays, tile width 16) across 100,000 logical records (3.2 MB logical footprint). Evaluates $(\text{pos} \cdot \text{vel}) \times \text{mass}$ dot product kernel.
- **Suite 2 (Constant-Access Stride)**: Evaluates strided accesses (strides 1, 2, 4, 8, 16, 32, 64, 128, 256) over a contiguous float array. Fixed access count of 100,000 elements ensures identical operation counts across all strides. Stride 16 (64 bytes) represents single cache-line spacing.
- **Suite 3 (Working-Set Scaling)**: Evaluates single-pass traversal over working sets spanning 15 orders of magnitude (4 KiB to 64 MiB), measuring memory bandwidth as datasets scale from L1/L2 cache into DRAM.

---

## Benchmark Report Directory Structure

Benchmark execution text logs and CSV reports are saved to:
`projects/03-performance-lab/benchmarks/reports/`

Generated reports:
- `harness_validation_release.csv` / `harness_validation_debug.csv`
- `cache_locality_release.csv` / `cache_locality_debug.csv`
