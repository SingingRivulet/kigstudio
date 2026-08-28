#include "test_common.h"

#include <cassert>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

#include "kigstudio/utils/compress.h"
#include "kigstudio/voxel/voxel.h"

using namespace sinriv::kigstudio;
using namespace sinriv::kigstudio::voxel;

namespace {

void fill_box(VoxelGrid& grid, int min_v, int max_v) {
    std::vector<Vec3i> pts;
    for (int x = min_v; x <= max_v; ++x)
        for (int y = min_v; y <= max_v; ++y)
            for (int z = min_v; z <= max_v; ++z)
                pts.emplace_back(x, y, z);
    grid.insertMany(pts);
}

// 逐体素蛮力比较两个 grid 在 [min_v, max_v]^3 范围内完全一致
void assert_grids_equal(const VoxelGrid& a,
                        const VoxelGrid& b,
                        int min_v,
                        int max_v) {
    for (int x = min_v; x <= max_v; ++x)
        for (int y = min_v; y <= max_v; ++y)
            for (int z = min_v; z <= max_v; ++z)
                assert(a.contains(x, y, z) == b.contains(x, y, z) &&
                       "grid content mismatch");
}

}  // namespace

int main() {
    setup_test_environment();

    // ============ 用例 1：insertMany 后 compress → AllOne ============
    {
        VoxelGrid grid;
        fill_box(grid, 0, 31);  // 恰好填满 chunk (0,0,0)

        auto it = grid.chunks.find(packChunkKey(0, 0, 0));
        assert(it != grid.chunks.end());
        assert(it->second.type == Chunk::Type::AllOne &&
               "full chunk should compress to AllOne");

        for (int x = 0; x < 32; ++x)
            for (int y = 0; y < 32; ++y)
                for (int z = 0; z < 32; ++z)
                    assert(grid.contains(x, y, z));

        // 迭代器应产出恰好 32768 个体素
        long count = 0;
        for (const auto& v : grid) {
            (void)v;
            ++count;
        }
        assert(count == 32768 && "iterator should yield all voxels of AllOne");
        std::cout << "case 1 passed" << std::endl;
    }

    // ============ 用例 2：clear 命中 AllOne → 自动细分 → 清空后回收 ============
    {
        VoxelGrid grid;
        fill_box(grid, 0, 31);
        uint64_t key = packChunkKey(0, 0, 0);

        grid.remove(Vec3i(0, 0, 0));
        auto it = grid.chunks.find(key);
        assert(it != grid.chunks.end());
        assert(it->second.type == Chunk::Type::Subtree &&
               "clearing an AllOne chunk should subdivide it");
        const auto& st = *it->second.subtree;
        assert(st.child_size == 16 && st.child_count == 8);
        int dense_leaves = 0, one_leaves = 0;
        for (int i = 0; i < st.child_count; ++i) {
            if (st.children[i].type == SubtreeData::Child::Type::Dense)
                ++dense_leaves;
            if (st.children[i].type == SubtreeData::Child::Type::AllOne)
                ++one_leaves;
        }
        assert(dense_leaves == 1 && one_leaves == 7);
        assert(!grid.contains(0, 0, 0));
        assert(grid.contains(1, 1, 1));
        assert(grid.contains(31, 31, 31));

        // 清空整个 chunk → 叶子全 AllZero → 合并 → 从 map 擦除
        std::vector<Vec3i> rest;
        for (int x = 0; x < 32; ++x)
            for (int y = 0; y < 32; ++y)
                for (int z = 0; z < 32; ++z)
                    rest.emplace_back(x, y, z);
        grid.removeMany(rest);
        assert(grid.chunks.find(key) == grid.chunks.end() &&
               "fully cleared chunk should be erased");
        std::cout << "case 2 passed" << std::endl;
    }

    // ============ 用例 3：Dense 显式 subdivide(8) 后内容一致 ============
    {
        VoxelGrid grid;
        fill_box(grid, 3, 20);  // chunk (0,0,0) 内的部分块
        uint64_t key = packChunkKey(0, 0, 0);
        auto it = grid.chunks.find(key);
        assert(it != grid.chunks.end());
        assert(it->second.type == Chunk::Type::Dense);

        // 细分前快照
        std::vector<char> before(32 * 32 * 32);
        for (int x = 0; x < 32; ++x)
            for (int y = 0; y < 32; ++y)
                for (int z = 0; z < 32; ++z)
                    before[(z * 32 + y) * 32 + x] =
                        it->second.get(x, y, z) ? 1 : 0;

        it->second.subdivide(8);
        assert(it->second.type == Chunk::Type::Subtree);
        assert(it->second.subtree->child_size == 8);

        for (int x = 0; x < 32; ++x)
            for (int y = 0; y < 32; ++y)
                for (int z = 0; z < 32; ++z)
                    assert((it->second.get(x, y, z) ? 1 : 0) ==
                               before[(z * 32 + y) * 32 + x] &&
                           "subdivide(8) changed content");

        // getWord 也应与稠密展开一致
        uint64_t words[Chunk::WORD_COUNT];
        it->second.extractWords(words);
        for (int i = 0; i < Chunk::WORD_COUNT; ++i)
            assert(words[i] == it->second.getWord(i));

        // 子树上读写
        it->second.set(31, 31, 31);
        assert(it->second.get(31, 31, 31));
        it->second.clear(5, 5, 5);
        assert(!it->second.get(5, 5, 5));
        assert(it->second.get(6, 5, 5));
        std::cout << "case 3 passed" << std::endl;
    }

    // ============ 用例 4：布尔运算与逐体素参考实现等价 ============
    {
        VoxelGrid a, b;
        fill_box(a, 0, 40);
        fill_box(b, 20, 60);

        // 让 b 的某个 partial chunk 变成 Subtree（混合类型参与运算）
        b.chunks.find(packChunkKey(1, 1, 1))->second.subdivide(16);

        auto u = a.unionWith_local(b);
        auto in = a.intersection_local(b);
        auto d = a.difference_local(b);

        for (int x = -5; x <= 70; ++x) {
            for (int y = -5; y <= 70; ++y) {
                for (int z = -5; z <= 70; ++z) {
                    bool av = a.contains(x, y, z);
                    bool bv = b.contains(x, y, z);
                    assert(u.contains(x, y, z) == (av || bv));
                    assert(in.contains(x, y, z) == (av && bv));
                    assert(d.contains(x, y, z) == (av && !bv));
                }
            }
        }
        std::cout << "case 4 passed" << std::endl;
    }

    // ============ 用例 5：equals 按内容比较，与表示无关 ============
    {
        Chunk dense_full;
        for (int x = 0; x < 32; ++x)
            for (int y = 0; y < 32; ++y)
                for (int z = 0; z < 32; ++z)
                    dense_full.set(x, y, z);
        assert(dense_full.type == Chunk::Type::Dense);

        Chunk all_one;
        all_one.type = Chunk::Type::AllOne;
        assert(dense_full.equals(all_one));
        assert(all_one.equals(dense_full));

        Chunk dense_hole = dense_full;
        dense_hole.clear(0, 0, 0);
        assert(!dense_hole.equals(all_one));

        Chunk zero1, zero2;
        zero2.ensureDense();  // Dense 全 0 vs AllZero：内容相同
        assert(zero1.equals(zero2));
        std::cout << "case 5 passed" << std::endl;
    }

    // ============ 用例 6：VXGRID2 序列化往返（含 4 种类型） ============
    {
        VoxelGrid grid;
        fill_box(grid, 0, 40);  // chunk(0,*,*) 全满→AllOne，其余 partial→Dense
        // 把一个 partial chunk 细分成 Subtree
        uint64_t sub_key = packChunkKey(1, 1, 1);
        grid.chunks.find(sub_key)->second.subdivide(16);
        grid.global_position = {1.5f, -2.0f, 3.25f};
        grid.voxel_size = {0.5f, 0.5f, 0.5f};

        const char* path = "test_voxel_chunk_types.vxgrid";
        std::string error;
        bool ok = save(path, grid, &error);
        assert(ok && "save v2 failed");

        VoxelGrid loaded;
        ok = load(path, loaded);
        assert(ok && "load v2 failed");
        assert(loaded.global_position.x == 1.5f &&
               loaded.global_position.z == 3.25f);
        assert(loaded.voxel_size.x == 0.5f);
        assert_grids_equal(grid, loaded, -5, 70);

        // 类型保持
        assert(loaded.chunks.find(packChunkKey(0, 0, 0))->second.type ==
               Chunk::Type::AllOne);
        auto lit = loaded.chunks.find(sub_key);
        assert(lit != loaded.chunks.end());
        assert(lit->second.type == Chunk::Type::Subtree);
        assert(lit->second.subtree->child_size == 16);
        std::cout << "case 6 passed" << std::endl;
    }

    // ============ 用例 7：VXGRID1 旧格式兼容读取 ============
    {
        // 手工构造一个 v1 文件：1 个全 1 稠密 chunk
        const char* path = "test_voxel_chunk_types_v1.vxgrid";
        {
            std::ofstream ofs(path, std::ios::binary);
            char magic[8] = {'V', 'X', 'G', 'R', 'I', 'D', '1', '\0'};
            uint32_t version = 1;
            ofs.write(magic, 8);
            ofs.write(reinterpret_cast<char*>(&version), sizeof(version));
            vec3<float> pos{0.f, 0.f, 0.f}, vs{1.f, 1.f, 1.f};
            ofs.write(reinterpret_cast<char*>(&pos), sizeof(pos));
            ofs.write(reinterpret_cast<char*>(&vs), sizeof(vs));
            uint32_t chunk_count = 1;
            ofs.write(reinterpret_cast<char*>(&chunk_count),
                      sizeof(chunk_count));

            std::vector<uint8_t> raw;
            uint64_t key = packChunkKey(0, 0, 0);
            size_t off = raw.size();
            raw.resize(off + sizeof(uint64_t) +
                       sizeof(uint64_t) * Chunk::WORD_COUNT);
            std::memcpy(raw.data() + off, &key, sizeof(uint64_t));
            std::memset(raw.data() + off + sizeof(uint64_t), 0xFF,
                        sizeof(uint64_t) * Chunk::WORD_COUNT);

            std::vector<uint8_t> compressed;
            bool ok = zlibCompress(raw, compressed);
            assert(ok);
            uint32_t comp_size = (uint32_t)compressed.size();
            uint32_t raw_size = (uint32_t)raw.size();
            ofs.write(reinterpret_cast<char*>(&comp_size), sizeof(comp_size));
            ofs.write(reinterpret_cast<char*>(&raw_size), sizeof(raw_size));
            ofs.write(reinterpret_cast<char*>(compressed.data()), comp_size);
        }

        VoxelGrid grid;
        bool ok = load(path, grid);
        assert(ok && "load v1 failed");
        auto it = grid.chunks.find(packChunkKey(0, 0, 0));
        assert(it != grid.chunks.end());
        assert(it->second.type == Chunk::Type::Dense &&
               "v1 chunks should load as Dense");
        for (int x = 0; x < 32; ++x)
            for (int y = 0; y < 32; ++y)
                for (int z = 0; z < 32; ++z)
                    assert(grid.contains(x, y, z));
        std::cout << "case 7 passed" << std::endl;
    }

    std::cout << "Test Passed!" << std::endl;
    return 0;
}
