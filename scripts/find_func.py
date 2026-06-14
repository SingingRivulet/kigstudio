import re

with open('ui/render_voxel_render_geo.cpp', 'r', encoding='utf-8') as f:
    content = f.read()

pattern = r'void RenderVoxelList::load_from_node\(int target_item_id,\s*int source_node_id,\s*int node_source_data_type,\s*int node_source_sdf_subdivisions,\s*bool node_source_sdf_simplify,\s*float node_source_sdf_simplify_ratio,\s*int load_mode\) \{'
match = re.search(pattern, content)
if not match:
    print('Function signature not found')
    exit(1)

start = match.start()
brace_start = content.find('{', match.end() - 1)
pos = brace_start + 1
depth = 1
while pos < len(content) and depth > 0:
    if content[pos] == '{':
        depth += 1
    elif content[pos] == '}':
        depth -= 1
    pos += 1
end = pos

print(f'Found function from {start} to {end}')
print(f'Length: {end - start}')
