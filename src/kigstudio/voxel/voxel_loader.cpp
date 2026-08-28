#include "voxel.h"
#include "kigstudio/utils/compress.h"

namespace sinriv::kigstudio {

namespace {

using voxel::Chunk;
using voxel::SubtreeData;

void appendBytes(std::vector<uint8_t>& buf, const void* data, size_t size) {
    size_t off = buf.size();
    buf.resize(off + size);
    std::memcpy(buf.data() + off, data, size);
}

template <typename T>
void appendPod(std::vector<uint8_t>& buf, const T& v) {
    appendBytes(buf, &v, sizeof(T));
}

// ============ VXGRID2 chunk 记录 ============
// 每 chunk：key u64 + type u8（0=Dense, 1=AllOne, 2=Subtree；AllZero 不写入）
//   Dense：512 × u64
//   AllOne：无负载
//   Subtree：child_size u8 + 每叶子（type u8：0=AllZero,1=AllOne,2=Dense；
//            Dense 叶子跟随 childWords × u64）

uint32_t serializableChunkCount(const voxel::VoxelGrid& grid) {
    uint32_t n = 0;
    for (const auto& [key, chunk] : grid.chunks) {
        if (chunk.type != Chunk::Type::AllZero)
            ++n;
    }
    return n;
}

std::vector<uint8_t> serializeChunksV2(const voxel::VoxelGrid& grid) {
    std::vector<uint8_t> raw;
    raw.reserve(grid.chunks.size() * 16);
    for (const auto& [key, chunk] : grid.chunks) {
        if (chunk.type == Chunk::Type::AllZero)
            continue;
        appendPod(raw, key);
        if (chunk.type == Chunk::Type::AllOne) {
            appendPod(raw, uint8_t(1));
        } else if (chunk.type == Chunk::Type::Subtree) {
            appendPod(raw, uint8_t(2));
            const auto& st = *chunk.subtree;
            appendPod(raw, uint8_t(st.child_size));
            size_t words_bytes =
                sizeof(uint64_t) * static_cast<size_t>(st.childWords());
            for (int i = 0; i < st.child_count; ++i) {
                const auto& child = st.children[i];
                uint8_t ct = child.type == SubtreeData::Child::Type::AllZero
                                 ? 0
                                 : child.type ==
                                           SubtreeData::Child::Type::AllOne
                                       ? 1
                                       : 2;
                appendPod(raw, ct);
                if (ct == 2) {
                    appendBytes(raw, child.words.get(), words_bytes);
                }
            }
        } else {
            appendPod(raw, uint8_t(0));
            appendBytes(raw, chunk.dense.get(),
                        sizeof(uint64_t) * Chunk::WORD_COUNT);
        }
    }
    return raw;
}

bool deserializeChunks(const std::vector<uint8_t>& raw,
                       uint32_t version,
                       uint32_t chunk_count,
                       voxel::VoxelGrid& grid) {
    grid.chunks.clear();
    const uint8_t* p = raw.data();
    size_t size = raw.size();
    size_t off = 0;

    auto take = [&](void* dst, size_t n) -> bool {
        if (off + n > size)
            return false;
        std::memcpy(dst, p + off, n);
        off += n;
        return true;
    };

    constexpr size_t kDenseBytes = sizeof(uint64_t) * Chunk::WORD_COUNT;

    for (uint32_t i = 0; i < chunk_count; ++i) {
        uint64_t key;
        if (!take(&key, sizeof(key)))
            return false;

        if (version == 1) {
            // 旧格式：全部为稠密位图
            uint64_t words[Chunk::WORD_COUNT];
            if (!take(words, kDenseBytes))
                return false;
            Chunk chunk;
            chunk.setDenseWords(words);
            if (!chunk.empty())
                grid.chunks[key] = std::move(chunk);
            continue;
        }

        uint8_t type;
        if (!take(&type, sizeof(type)))
            return false;

        if (type == 0) {  // Dense
            uint64_t words[Chunk::WORD_COUNT];
            if (!take(words, kDenseBytes))
                return false;
            Chunk chunk;
            chunk.setDenseWords(words);
            grid.chunks[key] = std::move(chunk);
        } else if (type == 1) {  // AllOne
            Chunk chunk;
            chunk.type = Chunk::Type::AllOne;
            grid.chunks[key] = std::move(chunk);
        } else if (type == 2) {  // Subtree
            uint8_t cs;
            if (!take(&cs, sizeof(cs)))
                return false;
            if (cs != 16 && cs != 8)
                return false;
            Chunk chunk;
            auto st = std::make_unique<SubtreeData>();
            st->init(cs);
            size_t words_bytes =
                sizeof(uint64_t) * static_cast<size_t>(st->childWords());
            for (int c = 0; c < st->child_count; ++c) {
                uint8_t ct;
                if (!take(&ct, sizeof(ct)))
                    return false;
                auto& child = st->children[c];
                if (ct == 1) {
                    child.type = SubtreeData::Child::Type::AllOne;
                } else if (ct == 2) {
                    child.type = SubtreeData::Child::Type::Dense;
                    child.words =
                        std::make_unique<uint64_t[]>(st->childWords());
                    if (!take(child.words.get(), words_bytes))
                        return false;
                } else if (ct != 0) {
                    return false;
                }
            }
            chunk.subtree = std::move(st);
            chunk.type = Chunk::Type::Subtree;
            if (!chunk.empty())
                grid.chunks[key] = std::move(chunk);
        } else {
            return false;
        }
    }
    return true;
}

}  // namespace

bool save(const std::filesystem::path& path,
          const voxel::VoxelGrid& grid,
          std::string* error) {
#ifdef _WIN32
    FILE* fp = _wfopen(path.wstring().c_str(), L"wb");
    if (!fp) {
        if (error)
            *error = "open file failed";
        return false;
    }
    auto write = [&](const void* data, size_t size) -> bool {
        return fwrite(data, 1, size, fp) == size;
    };
    bool ok = true;
    const char magic[8] = {'V', 'X', 'G', 'R', 'I', 'D', '2', '\0'};
    uint32_t version = 2;
    ok = ok && write(magic, 8);
    ok = ok && write(&version, sizeof(version));
    ok = ok && write(&grid.global_position, sizeof(grid.global_position));
    ok = ok && write(&grid.voxel_size, sizeof(grid.voxel_size));
    uint32_t chunk_count = serializableChunkCount(grid);
    ok = ok && write(&chunk_count, sizeof(chunk_count));
    std::vector<uint8_t> raw = serializeChunksV2(grid);
    std::vector<uint8_t> compressed;
    if (!zlibCompress(raw, compressed)) {
        if (error)
            *error = "zlib compress failed";
        fclose(fp);
        return false;
    }
    uint32_t comp_size = (uint32_t)compressed.size();
    uint32_t raw_size = (uint32_t)raw.size();
    ok = ok && write(&comp_size, sizeof(comp_size));
    ok = ok && write(&raw_size, sizeof(raw_size));
    ok = ok && write(compressed.data(), comp_size);
    fclose(fp);
    if (!ok && error)
        *error = "write file failed";
    return ok;
#else
    std::ofstream ofs(path.c_str(), std::ios::binary);
    if (!ofs) {
        if (error)
            *error = "open file failed";
        return false;
    }
    const char magic[8] = {'V', 'X', 'G', 'R', 'I', 'D', '2', '\0'};
    uint32_t version = 2;
    ofs.write(magic, 8);
    ofs.write(reinterpret_cast<char*>(&version), sizeof(version));
    ofs.write(reinterpret_cast<const char*>(&grid.global_position),
              sizeof(grid.global_position));
    ofs.write(reinterpret_cast<const char*>(&grid.voxel_size),
              sizeof(grid.voxel_size));
    uint32_t chunk_count = serializableChunkCount(grid);
    ofs.write(reinterpret_cast<char*>(&chunk_count), sizeof(chunk_count));
    std::vector<uint8_t> raw = serializeChunksV2(grid);
    std::vector<uint8_t> compressed;
    if (!zlibCompress(raw, compressed)) {
        if (error)
            *error = "zlib compress failed";
        return false;
    }
    uint32_t comp_size = (uint32_t)compressed.size();
    uint32_t raw_size = (uint32_t)raw.size();
    ofs.write(reinterpret_cast<char*>(&comp_size), sizeof(comp_size));
    ofs.write(reinterpret_cast<char*>(&raw_size), sizeof(raw_size));
    ofs.write(reinterpret_cast<char*>(compressed.data()), comp_size);
    return true;
#endif
}

bool load(const std::filesystem::path& path, voxel::VoxelGrid& grid) {
#ifdef _WIN32
    FILE* fp = _wfopen(path.wstring().c_str(), L"rb");
    if (!fp)
        return false;
    auto read = [&](void* data, size_t size) -> bool {
        return fread(data, 1, size, fp) == size;
    };
    char magic[8];
    uint32_t version;
    if (!read(magic, 8) || !read(&version, sizeof(version))) {
        fclose(fp);
        return false;
    }
    bool v1 = std::strncmp(magic, "VXGRID1", 7) == 0 && version == 1;
    bool v2 = std::strncmp(magic, "VXGRID2", 7) == 0 && version == 2;
    if (!v1 && !v2) {
        fclose(fp);
        return false;
    }
    if (!read(&grid.global_position, sizeof(grid.global_position)) ||
        !read(&grid.voxel_size, sizeof(grid.voxel_size))) {
        fclose(fp);
        return false;
    }
    uint32_t chunk_count;
    if (!read(&chunk_count, sizeof(chunk_count))) {
        fclose(fp);
        return false;
    }
    uint32_t comp_size, raw_size;
    if (!read(&comp_size, sizeof(comp_size)) ||
        !read(&raw_size, sizeof(raw_size))) {
        fclose(fp);
        return false;
    }
    std::vector<uint8_t> compressed(comp_size);
    if (!read(compressed.data(), comp_size)) {
        fclose(fp);
        return false;
    }
    fclose(fp);
    std::vector<uint8_t> raw;
    if (!zlibDecompress(compressed, raw, raw_size))
        return false;
    return deserializeChunks(raw, version, chunk_count, grid);
#else
    std::ifstream ifs(path.c_str(), std::ios::binary);
    if (!ifs)
        return false;
    char magic[8];
    uint32_t version;
    ifs.read(magic, 8);
    ifs.read(reinterpret_cast<char*>(&version), sizeof(version));
    bool v1 = std::strncmp(magic, "VXGRID1", 7) == 0 && version == 1;
    bool v2 = std::strncmp(magic, "VXGRID2", 7) == 0 && version == 2;
    if (!v1 && !v2)
        return false;
    ifs.read(reinterpret_cast<char*>(&grid.global_position),
             sizeof(grid.global_position));
    ifs.read(reinterpret_cast<char*>(&grid.voxel_size),
             sizeof(grid.voxel_size));
    uint32_t chunk_count;
    ifs.read(reinterpret_cast<char*>(&chunk_count), sizeof(chunk_count));
    uint32_t comp_size, raw_size;
    ifs.read(reinterpret_cast<char*>(&comp_size), sizeof(comp_size));
    ifs.read(reinterpret_cast<char*>(&raw_size), sizeof(raw_size));
    std::vector<uint8_t> compressed(comp_size);
    ifs.read(reinterpret_cast<char*>(compressed.data()), comp_size);
    std::vector<uint8_t> raw;
    if (!zlibDecompress(compressed, raw, raw_size))
        return false;
    return deserializeChunks(raw, version, chunk_count, grid);
#endif
}
}  // namespace sinriv::kigstudio
