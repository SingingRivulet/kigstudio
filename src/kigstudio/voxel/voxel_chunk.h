#pragma once

// Chunk：32³ 体素块，4 种存储类型，暴露相同接口
//   Dense   —— 512 个 uint64_t 的位图（4KB，堆分配）
//   AllZero —— 全 0，无负载
//   AllOne  —— 全 1，无负载（实心内部区域的内存从 4KB 降到几十字节）
//   Subtree —— 细分为 16³×8 或 8³×64 个叶子的单层子树，叶子为同质或稠密，
//              便于雕刻时只物化/处理局部子节点

#include <cstdint>
#include <cstring>
#include <memory>

namespace sinriv::kigstudio::voxel {

// ================= Subtree =================

struct SubtreeData {
    struct Child {
        enum class Type : uint8_t { AllZero, AllOne, Dense } type =
            Type::AllZero;
        std::unique_ptr<uint64_t[]> words;  // 仅 Dense 分配
    };

    int child_size = 16;   // 16 或 8
    int child_count = 8;   // 8 或 64
    std::unique_ptr<Child[]> children;

    inline void init(int cs) {
        child_size = cs;
        int dim = 32 / cs;
        child_count = dim * dim * dim;
        children = std::make_unique<Child[]>(child_count);
    }

    inline int childWords() const {
        return child_size * child_size * child_size / 64;
    }

    inline int childIndex(int x, int y, int z) const {
        int dim = 32 / child_size;
        return ((z / child_size) * dim + (y / child_size)) * dim +
               (x / child_size);
    }

    inline bool get(int x, int y, int z) const {
        const Child& c = children[childIndex(x, y, z)];
        if (c.type == Child::Type::AllZero)
            return false;
        if (c.type == Child::Type::AllOne)
            return true;
        int lx = x % child_size, ly = y % child_size, lz = z % child_size;
        int idx = (lz * child_size + ly) * child_size + lx;
        return (c.words[idx >> 6] >> (idx & 63)) & 1ULL;
    }

    // 设置/清除单个体素。叶子为同质时按需物化为稠密叶子。
    inline void setVoxel(int x, int y, int z, bool value) {
        Child& c = children[childIndex(x, y, z)];
        if (value) {
            if (c.type == Child::Type::AllOne)
                return;
            if (c.type != Child::Type::Dense) {
                c.words = std::make_unique<uint64_t[]>(childWords());
                std::memset(c.words.get(), 0,
                            sizeof(uint64_t) * childWords());
                c.type = Child::Type::Dense;
            }
        } else {
            if (c.type == Child::Type::AllZero)
                return;
            if (c.type == Child::Type::AllOne) {
                c.words = std::make_unique<uint64_t[]>(childWords());
                std::memset(c.words.get(), 0xFF,
                            sizeof(uint64_t) * childWords());
                c.type = Child::Type::Dense;
            }
        }
        int lx = x % child_size, ly = y % child_size, lz = z % child_size;
        int idx = (lz * child_size + ly) * child_size + lx;
        if (value) {
            c.words[idx >> 6] |= (1ULL << (idx & 63));
        } else {
            c.words[idx >> 6] &= ~(1ULL << (idx & 63));
        }
    }

    // 叶子级压缩：稠密叶子全 0/全 1 时回收为同质叶子
    inline void compressChild(Child& c) const {
        if (c.type != Child::Type::Dense)
            return;
        bool zero = true, one = true;
        int n = childWords();
        for (int i = 0; i < n; ++i) {
            zero &= (c.words[i] == 0);
            one &= (c.words[i] == ~0ULL);
            if (!zero && !one)
                return;
        }
        c.words.reset();
        c.type = zero ? Child::Type::AllZero : Child::Type::AllOne;
    }

    inline void compress() const {
        for (int i = 0; i < child_count; ++i) {
            compressChild(children[i]);
        }
    }

    // 所有叶子同质且一致时返回 0/1，否则返回 -1
    inline int homogeneousValue() const {
        int result = -1;
        for (int i = 0; i < child_count; ++i) {
            const Child& c = children[i];
            if (c.type == Child::Type::Dense)
                return -1;
            int v = (c.type == Child::Type::AllOne) ? 1 : 0;
            if (result == -1) {
                result = v;
            } else if (result != v) {
                return -1;
            }
        }
        return result;
    }

    // 取某一行（固定 y,z，x 从 x0 开始共 child_size 位）的位段
    inline uint32_t rowSegment(int x0, int y, int z) const {
        const Child& c = children[childIndex(x0, y, z)];
        uint32_t mask = (uint32_t(1) << child_size) - 1;
        if (c.type == Child::Type::AllZero)
            return 0;
        if (c.type == Child::Type::AllOne)
            return mask;
        int ly = y % child_size, lz = z % child_size;
        int base = (lz * child_size + ly) * child_size + (x0 % child_size);
        return uint32_t(c.words[base >> 6] >> (base & 63)) & mask;
    }

    // 取 chunk 的第 i 个 word（体素索引 [64i, 64i+64)，x 最快序）
    inline uint64_t wordAt(int i) const {
        uint64_t result = 0;
        // 64 个连续体素索引 = 2 个完整的 x 行（每行 32 体素）
        for (int half = 0; half < 2; ++half) {
            int r = i * 2 + half;
            int y = r & 31, z = r >> 5;
            uint32_t row = 0;
            for (int x0 = 0; x0 < 32; x0 += child_size) {
                row |= rowSegment(x0, y, z) << x0;
            }
            result |= uint64_t(row) << (half * 32);
        }
        return result;
    }

    inline SubtreeData* clone() const {
        auto* copy = new SubtreeData();
        copy->child_size = child_size;
        copy->child_count = child_count;
        copy->children = std::make_unique<Child[]>(child_count);
        int n = childWords();
        for (int i = 0; i < child_count; ++i) {
            copy->children[i].type = children[i].type;
            if (children[i].type == Child::Type::Dense) {
                copy->children[i].words = std::make_unique<uint64_t[]>(n);
                std::memcpy(copy->children[i].words.get(), children[i].words.get(),
                            sizeof(uint64_t) * n);
            }
        }
        return copy;
    }
};

// ================= Chunk =================

struct Chunk {
    static constexpr int SIZE = 32;
    static constexpr int VOXEL_COUNT = SIZE * SIZE * SIZE;  // 32768
    static constexpr int WORD_COUNT = VOXEL_COUNT / 64;     // 512

    enum class Type : uint8_t { Dense, AllZero, AllOne, Subtree };

    Type type = Type::AllZero;  // 默认全 0：无分配（chunks[key] 零成本）
    std::unique_ptr<uint64_t[]> dense;    // 512 words，仅 Dense
    std::unique_ptr<SubtreeData> subtree; // 仅 Subtree

    Chunk() = default;
    ~Chunk() = default;

    Chunk(Chunk&&) noexcept = default;
    Chunk& operator=(Chunk&&) noexcept = default;

    Chunk(const Chunk& other) { copyFrom(other); }
    Chunk& operator=(const Chunk& other) {
        if (this != &other) {
            copyFrom(other);
        }
        return *this;
    }

    inline void copyFrom(const Chunk& other) {
        type = other.type;
        if (other.type == Type::Dense) {
            dense = std::make_unique<uint64_t[]>(WORD_COUNT);
            std::memcpy(dense.get(), other.dense.get(),
                        sizeof(uint64_t) * WORD_COUNT);
            subtree.reset();
        } else if (other.type == Type::Subtree) {
            subtree.reset(other.subtree->clone());
            dense.reset();
        } else {
            dense.reset();
            subtree.reset();
        }
    }

    inline int index(int x, int y, int z) const {
        return (z * SIZE + y) * SIZE + x;
    }

    // ============ 读取（任意类型）============

    inline bool get(int x, int y, int z) const {
        switch (type) {
        case Type::AllZero:
            return false;
        case Type::AllOne:
            return true;
        case Type::Subtree:
            return subtree->get(x, y, z);
        default: {
            int i = index(x, y, z);
            return (dense[i >> 6] >> (i & 63)) & 1ULL;
        }
        }
    }

    inline uint64_t getWord(int i) const {
        switch (type) {
        case Type::AllZero:
            return 0;
        case Type::AllOne:
            return ~0ULL;
        case Type::Subtree:
            return subtree->wordAt(i);
        default:
            return dense[i];
        }
    }

    // 展开为 512 个 word 的稠密位图
    inline void extractWords(uint64_t* out) const {
        switch (type) {
        case Type::AllZero:
            std::memset(out, 0, sizeof(uint64_t) * WORD_COUNT);
            return;
        case Type::AllOne:
            std::memset(out, 0xFF, sizeof(uint64_t) * WORD_COUNT);
            return;
        case Type::Subtree:
            for (int i = 0; i < WORD_COUNT; ++i)
                out[i] = subtree->wordAt(i);
            return;
        default:
            std::memcpy(out, dense.get(), sizeof(uint64_t) * WORD_COUNT);
            return;
        }
    }

    inline bool empty() const {
        switch (type) {
        case Type::AllZero:
            return true;
        case Type::AllOne:
            return false;
        case Type::Subtree:
            for (int i = 0; i < subtree->child_count; ++i) {
                const auto& c = subtree->children[i];
                if (c.type != SubtreeData::Child::Type::AllZero)
                    return false;
            }
            return true;
        default:
            for (int i = 0; i < WORD_COUNT; i++)
                if (dense[i])
                    return false;
            return true;
        }
    }

    inline bool full() const {
        switch (type) {
        case Type::AllZero:
            return false;
        case Type::AllOne:
            return true;
        case Type::Subtree:
            for (int i = 0; i < subtree->child_count; ++i) {
                const auto& c = subtree->children[i];
                if (c.type != SubtreeData::Child::Type::AllOne)
                    return false;
            }
            return true;
        default:
            for (int i = 0; i < WORD_COUNT; i++)
                if (dense[i] != ~0ULL)
                    return false;
            return true;
        }
    }

    // 内容级比较（表示不同但内容相同也判等），替代 undo 的 memcmp
    inline bool equals(const Chunk& other) const {
        if (type == other.type) {
            switch (type) {
            case Type::AllZero:
            case Type::AllOne:
                return true;
            case Type::Dense:
                return std::memcmp(dense.get(), other.dense.get(),
                                   sizeof(uint64_t) * WORD_COUNT) == 0;
            default:
                break;  // Subtree：走逐 word 内容比较
            }
        }
        if (empty() && other.empty())
            return true;
        if (full() && other.full())
            return true;
        uint64_t a[WORD_COUNT], b[WORD_COUNT];
        extractWords(a);
        other.extractWords(b);
        return std::memcmp(a, b, sizeof(a)) == 0;
    }

    // ============ 写入（可能触发类型转换）============

    inline void ensureDense() {
        if (type == Type::Dense)
            return;
        auto words = std::make_unique<uint64_t[]>(WORD_COUNT);
        extractWords(words.get());
        dense = std::move(words);
        subtree.reset();
        type = Type::Dense;
    }

    // 直接用 512 个 word 覆盖内容（布尔运算/反序列化用）
    inline void setDenseWords(const uint64_t* words) {
        ensureDense();
        std::memcpy(dense.get(), words, sizeof(uint64_t) * WORD_COUNT);
    }

    inline void set(int x, int y, int z) {
        if (type == Type::AllOne)
            return;
        if (type == Type::Subtree) {
            subtree->setVoxel(x, y, z, true);
            return;
        }
        ensureDense();
        int i = index(x, y, z);
        dense[i >> 6] |= (1ULL << (i & 63));
    }

    inline void setWord(int i, uint64_t w) {
        if (type == Type::AllOne && w == ~0ULL)
            return;
        if (type == Type::AllZero && w == 0)
            return;
        ensureDense();
        dense[i] = w;
    }

    inline void clear(int x, int y, int z) {
        if (type == Type::AllZero)
            return;
        if (type == Type::AllOne) {
            // 雕刻挖实心：细分为 16³ 叶子，只物化目标叶子
            subdivide(16);
        }
        if (type == Type::Subtree) {
            subtree->setVoxel(x, y, z, false);
            subtree->compressChild(
                subtree->children[subtree->childIndex(x, y, z)]);
            tryMergeSubtree();
            return;
        }
        int i = index(x, y, z);
        dense[i >> 6] &= ~(1ULL << (i & 63));
    }

    // ============ 类型转换 ============

    // 细分为子树（child_size = 16 或 8）；已是相同粒度子树时 no-op
    inline void subdivide(int child_size) {
        if (type == Type::Subtree && subtree->child_size == child_size)
            return;

        uint64_t old_words[WORD_COUNT];
        const uint64_t* src = nullptr;
        if (type == Type::Dense || type == Type::Subtree) {
            extractWords(old_words);
            src = old_words;
        }

        auto st = std::make_unique<SubtreeData>();
        st->init(child_size);

        if (type == Type::AllOne) {
            for (int i = 0; i < st->child_count; ++i)
                st->children[i].type = SubtreeData::Child::Type::AllOne;
        } else if (src) {
            // 从稠密位图按段拼装每个叶子的位图
            uint32_t mask = (uint32_t(1) << child_size) - 1;
            int cs = child_size;
            int dim = 32 / cs;
            int leaf_words = st->childWords();
            for (int cz = 0; cz < dim; ++cz) {
                for (int cy = 0; cy < dim; ++cy) {
                    for (int cx = 0; cx < dim; ++cx) {
                        int ci = (cz * dim + cy) * dim + cx;
                        auto words = std::make_unique<uint64_t[]>(leaf_words);
                        for (int w = 0; w < leaf_words; ++w) {
                            uint64_t lw = 0;
                            // 每个叶子 word = (64/cs) 个 x 行段
                            for (int row = 0; row < 64 / cs; ++row) {
                                int lv = (w << 6) + row * cs;
                                int lz = lv / (cs * cs);
                                int ly = (lv % (cs * cs)) / cs;
                                int gx = cx * cs, gy = cy * cs + ly,
                                    gz = cz * cs + lz;
                                int cidx = (gz * SIZE + gy) * SIZE + gx;
                                lw |= ((src[cidx >> 6] >> (cidx & 63)) & mask)
                                      << (row * cs);
                            }
                            words[w] = lw;
                        }
                        st->children[ci].words = std::move(words);
                        st->children[ci].type = SubtreeData::Child::Type::Dense;
                        st->compressChild(st->children[ci]);
                    }
                }
            }
        }
        // AllZero：叶子默认即 AllZero

        dense.reset();
        subtree = std::move(st);
        type = Type::Subtree;
    }

    // 子树叶子同质一致时合并回同质 chunk
    inline void tryMergeSubtree() {
        if (type != Type::Subtree)
            return;
        int v = subtree->homogeneousValue();
        if (v < 0)
            return;
        subtree.reset();
        type = (v == 1) ? Type::AllOne : Type::AllZero;
    }

    // 压缩：Dense 全 0/全 1 → 同质；Subtree 叶子压缩 + 合并
    inline void compress() {
        if (type == Type::Dense) {
            bool zero = true, one = true;
            for (int i = 0; i < WORD_COUNT; ++i) {
                zero &= (dense[i] == 0);
                one &= (dense[i] == ~0ULL);
                if (!zero && !one)
                    return;
            }
            dense.reset();
            type = zero ? Type::AllZero : Type::AllOne;
        } else if (type == Type::Subtree) {
            subtree->compress();
            tryMergeSubtree();
        }
    }

    // ============ 布尔逐位运算（就地，含同质短路）============

    inline void orWith(const Chunk& other) {
        if (other.type == Type::AllZero || type == Type::AllOne)
            return;
        if (other.type == Type::AllOne) {
            dense.reset();
            subtree.reset();
            type = Type::AllOne;
            return;
        }
        if (type == Type::AllZero) {
            copyFrom(other);
            return;
        }
        uint64_t a[WORD_COUNT], b[WORD_COUNT];
        extractWords(a);
        other.extractWords(b);
        for (int i = 0; i < WORD_COUNT; ++i)
            a[i] |= b[i];
        setDenseWords(a);
        compress();
    }

    inline void andWith(const Chunk& other) {
        if (type == Type::AllZero || other.type == Type::AllOne)
            return;
        if (other.type == Type::AllZero) {
            dense.reset();
            subtree.reset();
            type = Type::AllZero;
            return;
        }
        if (type == Type::AllOne) {
            copyFrom(other);
            return;
        }
        uint64_t a[WORD_COUNT], b[WORD_COUNT];
        extractWords(a);
        other.extractWords(b);
        for (int i = 0; i < WORD_COUNT; ++i)
            a[i] &= b[i];
        setDenseWords(a);
        compress();
    }

    inline void andNotWith(const Chunk& other) {
        if (type == Type::AllZero || other.type == Type::AllZero)
            return;
        if (other.type == Type::AllOne) {
            dense.reset();
            subtree.reset();
            type = Type::AllZero;
            return;
        }
        uint64_t a[WORD_COUNT], b[WORD_COUNT];
        extractWords(a);
        other.extractWords(b);
        for (int i = 0; i < WORD_COUNT; ++i)
            a[i] &= ~b[i];
        setDenseWords(a);
        compress();
    }
};

}  // namespace sinriv::kigstudio::voxel
