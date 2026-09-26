#ifndef PERFORMANCE_LAB_DATA_LAYOUTS_HPP
#define PERFORMANCE_LAB_DATA_LAYOUTS_HPP

#include <cstddef>
#include <cstdint>
#include <vector>
#include <cmath>

namespace performance_lab {

/**
 * @brief Array of Structures (AoS) Layout.
 * 
 * Each record is a contiguous struct of 32 bytes (8 floats/ints):
 * - Position (x, y, z)
 * - Velocity (vx, vy, vz)
 * - Mass
 * - ID
 */
struct alignas(16) RecordAoS {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float vx{0.0f};
    float vy{0.0f};
    float vz{0.0f};
    float mass{0.0f};
    uint32_t id{0};
};

static_assert(sizeof(RecordAoS) == 32, "RecordAoS must be exactly 32 bytes");

/**
 * @brief Structure of Arrays (SoA) Layout.
 * 
 * Each field is stored in its own contiguous std::vector array.
 */
struct RecordSoA {
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
    std::vector<float> vx;
    std::vector<float> vy;
    std::vector<float> vz;
    std::vector<float> mass;
    std::vector<uint32_t> id;

    void resize(size_t count) {
        x.resize(count);
        y.resize(count);
        z.resize(count);
        vx.resize(count);
        vy.resize(count);
        vz.resize(count);
        mass.resize(count);
        id.resize(count);
    }

    size_t size() const {
        return x.size();
    }
};

/**
 * @brief Array of Structures of Arrays (AoSoA) Tile.
 * 
 * Groups TILE_WIDTH records into a single tile, where fields are stored in contiguous sub-arrays of size TILE_WIDTH.
 * Default tile width is 16 records.
 */
constexpr size_t DEFAULT_TILE_WIDTH = 16;

template <size_t TILE_WIDTH = DEFAULT_TILE_WIDTH>
struct alignas(32) TileAoSoA {
    float x[TILE_WIDTH];
    float y[TILE_WIDTH];
    float z[TILE_WIDTH];
    float vx[TILE_WIDTH];
    float vy[TILE_WIDTH];
    float vz[TILE_WIDTH];
    float mass[TILE_WIDTH];
    uint32_t id[TILE_WIDTH];
};

template <size_t TILE_WIDTH = DEFAULT_TILE_WIDTH>
struct RecordAoSoA {
    std::vector<TileAoSoA<TILE_WIDTH>> tiles;
    size_t record_count{0};

    void resize(size_t count) {
        record_count = count;
        size_t num_tiles = (count + TILE_WIDTH - 1) / TILE_WIDTH;
        tiles.resize(num_tiles);
    }

    size_t size() const {
        return record_count;
    }
};

// ============================================================================
// Deterministic Generators
// ============================================================================

inline std::vector<RecordAoS> generate_aos(size_t count) {
    std::vector<RecordAoS> data(count);
    for (size_t i = 0; i < count; ++i) {
        float fi = static_cast<float>(i);
        data[i].x = fi * 0.1f;
        data[i].y = fi * 0.2f;
        data[i].z = fi * 0.3f;
        data[i].vx = fi * 0.01f;
        data[i].vy = fi * 0.02f;
        data[i].vz = fi * 0.03f;
        data[i].mass = 1.0f + static_cast<float>(i % 100) * 0.05f;
        data[i].id = static_cast<uint32_t>(i + 1);
    }
    return data;
}

inline RecordSoA generate_soa(size_t count) {
    RecordSoA data;
    data.resize(count);
    for (size_t i = 0; i < count; ++i) {
        float fi = static_cast<float>(i);
        data.x[i] = fi * 0.1f;
        data.y[i] = fi * 0.2f;
        data.z[i] = fi * 0.3f;
        data.vx[i] = fi * 0.01f;
        data.vy[i] = fi * 0.02f;
        data.vz[i] = fi * 0.03f;
        data.mass[i] = 1.0f + static_cast<float>(i % 100) * 0.05f;
        data.id[i] = static_cast<uint32_t>(i + 1);
    }
    return data;
}

template <size_t TILE_WIDTH = DEFAULT_TILE_WIDTH>
inline RecordAoSoA<TILE_WIDTH> generate_aosoa(size_t count) {
    RecordAoSoA<TILE_WIDTH> data;
    data.resize(count);
    for (size_t i = 0; i < count; ++i) {
        size_t tile_idx = i / TILE_WIDTH;
        size_t slot_idx = i % TILE_WIDTH;
        float fi = static_cast<float>(i);
        data.tiles[tile_idx].x[slot_idx] = fi * 0.1f;
        data.tiles[tile_idx].y[slot_idx] = fi * 0.2f;
        data.tiles[tile_idx].z[slot_idx] = fi * 0.3f;
        data.tiles[tile_idx].vx[slot_idx] = fi * 0.01f;
        data.tiles[tile_idx].vy[slot_idx] = fi * 0.02f;
        data.tiles[tile_idx].vz[slot_idx] = fi * 0.03f;
        data.tiles[tile_idx].mass[slot_idx] = 1.0f + static_cast<float>(i % 100) * 0.05f;
        data.tiles[tile_idx].id[slot_idx] = static_cast<uint32_t>(i + 1);
    }
    return data;
}

// ============================================================================
// Sequential Workload Checksum Functions (Identical Mathematical Work)
// ============================================================================

inline double compute_checksum_aos(const std::vector<RecordAoS>& data) {
    double accum = 0.0;
    for (size_t i = 0; i < data.size(); ++i) {
        const auto& r = data[i];
        float dot_pos_vel = r.x * r.vx + r.y * r.vy + r.z * r.vz;
        accum += static_cast<double>(dot_pos_vel * r.mass);
    }
    return accum;
}

inline double compute_checksum_soa(const RecordSoA& data) {
    double accum = 0.0;
    size_t n = data.size();
    for (size_t i = 0; i < n; ++i) {
        float dot_pos_vel = data.x[i] * data.vx[i] + data.y[i] * data.vy[i] + data.z[i] * data.vz[i];
        accum += static_cast<double>(dot_pos_vel * data.mass[i]);
    }
    return accum;
}

template <size_t TILE_WIDTH = DEFAULT_TILE_WIDTH>
inline double compute_checksum_aosoa(const RecordAoSoA<TILE_WIDTH>& data) {
    double accum = 0.0;
    size_t count = data.record_count;
    size_t num_tiles = data.tiles.size();

    for (size_t t = 0; t < num_tiles; ++t) {
        const auto& tile = data.tiles[t];
        size_t items_in_tile = (t == num_tiles - 1 && (count % TILE_WIDTH != 0))
                                   ? (count % TILE_WIDTH)
                                   : TILE_WIDTH;
        for (size_t k = 0; k < items_in_tile; ++k) {
            float dot_pos_vel = tile.x[k] * tile.vx[k] + tile.y[k] * tile.vy[k] + tile.z[k] * tile.vz[k];
            accum += static_cast<double>(dot_pos_vel * tile.mass[k]);
        }
    }
    return accum;
}

} // namespace performance_lab

#endif // PERFORMANCE_LAB_DATA_LAYOUTS_HPP
