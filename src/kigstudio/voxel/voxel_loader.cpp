#include "voxel.h"
#include "kigstudio/utils/compress.h"

namespace sinriv::kigstudio {

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
    const char magic[8] = {'V', 'X', 'G', 'R', 'I', 'D', '1', '\0'};
    uint32_t version = 1;
    ok = ok && write(magic, 8);
    ok = ok && write(&version, sizeof(version));
    ok = ok && write(&grid.global_position, sizeof(grid.global_position));
    ok = ok && write(&grid.voxel_size, sizeof(grid.voxel_size));
    uint32_t chunk_count = (uint32_t)grid.chunks.size();
    ok = ok && write(&chunk_count, sizeof(chunk_count));
    std::vector<uint8_t> raw;
    raw.reserve(chunk_count * (sizeof(uint64_t) + sizeof(voxel::Chunk)));
    for (const auto& [key, chunk] : grid.chunks) {
        size_t offset = raw.size();
        raw.resize(offset + sizeof(uint64_t) + sizeof(voxel::Chunk));
        std::memcpy(raw.data() + offset, &key, sizeof(uint64_t));
        std::memcpy(raw.data() + offset + sizeof(uint64_t), chunk.data,
                    sizeof(chunk.data));
    }
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
    const char magic[8] = {'V', 'X', 'G', 'R', 'I', 'D', '1', '\0'};
    uint32_t version = 1;
    ofs.write(magic, 8);
    ofs.write(reinterpret_cast<char*>(&version), sizeof(version));
    ofs.write(reinterpret_cast<const char*>(&grid.global_position),
              sizeof(grid.global_position));
    ofs.write(reinterpret_cast<const char*>(&grid.voxel_size),
              sizeof(grid.voxel_size));
    uint32_t chunk_count = (uint32_t)grid.chunks.size();
    ofs.write(reinterpret_cast<char*>(&chunk_count), sizeof(chunk_count));
    std::vector<uint8_t> raw;
    raw.reserve(chunk_count * (sizeof(uint64_t) + sizeof(voxel::Chunk)));
    for (const auto& [key, chunk] : grid.chunks) {
        size_t offset = raw.size();
        raw.resize(offset + sizeof(uint64_t) + sizeof(voxel::Chunk));
        std::memcpy(raw.data() + offset, &key, sizeof(uint64_t));
        std::memcpy(raw.data() + offset + sizeof(uint64_t), chunk.data,
                    sizeof(chunk.data));
    }
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
    if (std::strncmp(magic, "VXGRID1", 7) != 0 || version != 1) {
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
    grid.chunks.clear();
    size_t offset = 0;
    for (uint32_t i = 0; i < chunk_count; i++) {
        uint64_t key;
        std::memcpy(&key, raw.data() + offset, sizeof(uint64_t));
        offset += sizeof(uint64_t);
        voxel::Chunk chunk;
        std::memcpy(chunk.data, raw.data() + offset, sizeof(chunk.data));
        offset += sizeof(chunk.data);
        if (!chunk.empty())
            grid.chunks[key] = chunk;
    }
    return true;
#else
    std::ifstream ifs(path.c_str(), std::ios::binary);
    if (!ifs)
        return false;
    char magic[8];
    uint32_t version;
    ifs.read(magic, 8);
    ifs.read(reinterpret_cast<char*>(&version), sizeof(version));
    if (std::strncmp(magic, "VXGRID1", 7) != 0 || version != 1)
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
    grid.chunks.clear();
    size_t offset = 0;
    for (uint32_t i = 0; i < chunk_count; i++) {
        uint64_t key;
        std::memcpy(&key, raw.data() + offset, sizeof(uint64_t));
        offset += sizeof(uint64_t);
        voxel::Chunk chunk;
        std::memcpy(chunk.data, raw.data() + offset, sizeof(chunk.data));
        offset += sizeof(chunk.data);
        if (!chunk.empty())
            grid.chunks[key] = chunk;
    }
    return true;
#endif
}
}  // namespace sinriv::kigstudio