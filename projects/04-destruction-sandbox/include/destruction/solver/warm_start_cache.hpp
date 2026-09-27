#ifndef DESTRUCTION_WARM_START_CACHE_HPP
#define DESTRUCTION_WARM_START_CACHE_HPP

#include <cstdint>
#include <unordered_map>
#include <vector>
#include <algorithm>

namespace destruction::solver {

struct CachedImpulse {
    float normal_impulse{0.0f};
    float tangent_impulse_1{0.0f};
    float tangent_impulse_2{0.0f};
    uint32_t age{0};
};

class WarmStartCache {
private:
    std::unordered_map<uint64_t, CachedImpulse> cache_;

public:
    WarmStartCache() = default;

    bool find(uint64_t key, float& out_normal, float& out_t1, float& out_t2) const {
        auto it = cache_.find(key);
        if (it != cache_.end()) {
            out_normal = it->second.normal_impulse;
            out_t1 = it->second.tangent_impulse_1;
            out_t2 = it->second.tangent_impulse_2;
            return true;
        }
        return false;
    }

    void insert(uint64_t key, float normal, float t1, float t2) {
        CachedImpulse ci;
        ci.normal_impulse = normal;
        ci.tangent_impulse_1 = t1;
        ci.tangent_impulse_2 = t2;
        ci.age = 0;
        cache_[key] = ci;
    }

    void age_and_prune(uint32_t max_age = 2) {
        std::vector<uint64_t> keys_to_remove;
        for (auto& pair : cache_) {
            pair.second.age++;
            if (pair.second.age > max_age) {
                keys_to_remove.push_back(pair.first);
            }
        }
        for (uint64_t k : keys_to_remove) {
            cache_.erase(k);
        }
    }

    size_t size() const {
        return cache_.size();
    }

    void clear() {
        cache_.clear();
    }
};

} // namespace destruction::solver

#endif // DESTRUCTION_WARM_START_CACHE_HPP
