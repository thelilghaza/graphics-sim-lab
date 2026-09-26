# Project 02 Lessons Learned — Voxel Engine

Observations, architecture trade-offs, performance analysis, and engineering decisions recorded during development.

---

## Initial Design Decisions & Architecture Trade-offs

1. **Flat Contiguous 1D Chunk Array Selection**:
   - Chose flat contiguous 1D array (`voxels[lx + ly * 32 + lz * 1024]`) for chunk storage instead of pointer-based 3D arrays or sparse hash maps.
   - Trade-off: Allocates memory for empty volume within loaded chunks, but guarantees $O(1)$ random access, zero pointer chasing, and sequential memory access matching 64-byte CPU cache lines.

2. **Chunk Sizing ($32^3$ Voxel Volume)**:
   - Evaluated $8^3$, $16^3$, $32^3$, and $64^3$ chunk sizes.
   - Selected $32^3$ ($32,768$ voxels per chunk = 64 KB at 2 bytes per voxel) because it fits within modern CPU L1/L2 cache, minimizes boundary neighbor checks relative to volume, and provides optimal meshing update granularity.

3. **Negative Coordinate Handling in World Space**:
   - Arithmetic right shift (`Wx >> 5`) and explicit floor division formulas prevent truncating negative coordinates toward zero, ensuring negative world positions map correctly to negative chunk coordinates without coordinate aliasing bugs.

4. **Baseline-First Meshing Strategy**:
   - Planned Naive Exposed-Face Culling in Milestone 4 before implementing Greedy Meshing in Milestone 6.
   - Follows the repository engineering principle: establish a clear correctness baseline and measure throughput before introducing advanced optimizations.

---

## Experiments & Measurement Findings

*(Will be populated with empirical data during implementation milestones)*

---

## Problems & Failure Modes

*(Will be populated as challenges arise during development)*

---

## Rejected Approaches

1. **Sparse Hash Maps (`std::unordered_map<IVec3, Voxel>`) for Chunk Volume Storage**:
   - Rejected due to node pointer memory overhead and hash collision cache misses during surface extraction loops.
2. **Large Per-Voxel Struct Payloads (64+ Bytes)**:
   - Rejected because large structs scale poorly to millions of voxels (16 GB RAM for 268M voxels vs 512 MB for 2-byte payload).
