# Project 03 Lessons Learned — Performance Lab

Observations, architecture trade-offs, performance analysis methodology, and design principles established during Phase 0 discovery.

---

## Phase 0 Architectural Observations

1. **Isolation of Performance Experimentation**:
   - Isolating micro-benchmarks into a dedicated project (`projects/03-performance-lab`) prevents polluting complex application pipelines (such as the ray tracer or voxel engine) with experimental optimization flags, SIMD intrinsics, or custom allocators.

2. **Necessity of Compiler Optimization Verification**:
   - Compiler auto-vectorization and loop unrolling vary significantly between compiler families (MSVC vs GCC vs Clang) and optimization levels (`/O2` vs `/O3` vs `/O1`).
   - Micro-benchmarks must verify build flags and inspect generated disassembly or compiler reports to confirm whether auto-vectorization actually occurred.

3. **Cache Line Alignment Requirements**:
   - Modern SIMD vector loads (`_mm256_load_ps`) require 32-byte memory alignment (`alignas(32)`). Unaligned loads (`_mm256_loadu_ps`) incur penalties on older microarchitectures and must be benchmarked explicitly.

4. **Queue Concurrency & False Sharing**:
   - Atomic structures eliminate mutex sleeping overhead, but atomic cache-line bouncing (false sharing between producer head and consumer tail pointers) can severely limit throughput if atomic variables share a single 64-byte cache line.
   - Using explicit cache-line padding (`alignas(64)`) between atomic head and tail pointers is a mandatory design rule for high-performance concurrent queues.

5. **Allocator Lifetime Economics & Bulk Reclamation**:
   - Linear arena allocators trade arbitrary individual deallocation for O(1) bulk reset speed. In workloads where object lifetimes are bounded by a frame or batch phase, arenas avoid individual general-purpose allocation/deallocation operations and reclaim the entire batch with a single reset.
   - Fixed-block pools trade request-size flexibility for fast O(1) reuse of preallocated blocks without returning memory to the system runtime heap.
   - Allocator benchmarking must strictly distinguish per-allocation lifetime management from bulk-lifetime allocation rather than asserting universal superiority for any single strategy.

---

## Initial Design Decisions & Trade-Offs

1. **Plain-Text Scientific Benchmarking Output**:
   - Benchmark executables print structured plain-text summaries and export CSV files to `benchmarks/reports/`.
   - Avoids external dependencies like Google Benchmark, preserving fast compilation and transparent execution.

2. **Standalone Kernel Implementations**:
   - Performance Lab builds self-contained benchmark kernels rather than linking Project 01 or Project 02 classes.
   - Follows the core repository architectural principle: keep projects independently buildable and avoid premature shared libraries in `libs/`.
