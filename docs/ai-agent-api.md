# AI Agent API Architecture (HTTP + Live GUI)

## Overview

KigStudio runs normally with its full 3D GUI. An HTTP server is embedded inside the app, listening on `127.0.0.1:18920`. AI agents send REST requests — all operations execute on the main thread and are **visible on screen in real time**.

```
┌──────────────────────────────────────────────────┐
│               kigstudio.exe (GUI)                │
│  ┌─────────────────┐  ┌───────────────────────┐  │
│  │ 3D Viewport     │  │ ImGui Panels          │  │
│  │ (bgfx)          │  │ (node tree, editor)   │  │
│  │                 │  │                       │  │
│  │  AI operations  │  │  params update live   │  │
│  │  visible here   │  │  as AI changes them   │  │
│  └─────────────────┘  └───────────────────────┘  │
│                                                    │
│  ┌─────────────────────────────────────────────┐  │
│  │  HTTP Server (cpp-httplib, port 18920)      │  │
│  │  POST /api/v1/nodes/:id/segment             │  │
│  │  POST /api/v1/mesh/repair        ↕ REST    │  │
│  │  GET  /api/v1/system/queue       ↕ WS      │  │
│  │  WS   /api/v1/ws                 progress  │  │
│  └─────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────┘
          ↕ HTTP (localhost:18920)
┌──────────────────────┐
│  AI Agent            │
│  (Python / Claude /  │
│   any HTTP client)   │
└──────────────────────┘
```

### Why HTTP + GUI Instead of Headless?

| Headless | HTTP + Live GUI |
|----------|-----------------|
| No visual feedback | User sees every change in real time |
| Agent spawns a separate process | Agent connects to already-running app |
| Batch-oriented | Interactive — user and AI collaborate |
| Stdin/stdout protocol | Standard REST, universal client support |
| State lost on exit | App keeps running, state persists |

---

## Layer 1: HTTP Server

### Library: cpp-httplib

Single-file header (`dep/cpp-httplib/httplib.h`), MIT licensed. Chosen over Boost.Beast because:
- One `#include`, zero build-system changes
- Thread-safe server with built-in thread pool
- Native JSON body support (just call `json.parse()`)
- WebSocket support for progress streaming
- Battle-tested (60k+ GitHub stars)

### Startup

The HTTP server starts automatically when kigstudio launches:
```
kigstudio.exe                    → GUI starts, HTTP on 127.0.0.1:18920
kigstudio.exe --agent-port 8080  → custom port
kigstudio.exe --no-agent         → disable HTTP server entirely
```

### Threading Model

```
┌── HTTP Thread Pool ──────────────────────┐
│  httplib worker thread                   │
│  1. Parse HTTP request + JSON body       │
│  2. Push AgentCommand to command_queue   │
│  3. Wait on std::promise (with timeout)  │
│  4. Format HTTP response + send          │
└──────────────────────────────────────────┘
              │ push          │ wait
              ▼               ▼
┌── Command Queue (mutex + condition_variable) ──┐
│  std::queue<AgentCommand>                       │
└─────────────────────────────────────────────────┘
              │ poll each frame
              ▼
┌── Main Thread (render loop) ──────────────┐
│  while (running) {                         │
│    process_agent_commands();  // ← NEW     │
│    handle_sdl_events();                    │
│    render_ui();                            │
│    render_bgfx();                          │
│    swap_buffers();                         │
│  }                                         │
└─────────────────────────────────────────────┘
```

**Why main-thread execution?**
- All state (`RenderVoxelList::items`, `VoxelGrid`, `SDFBase`) lives on the main thread
- Modifying it from HTTP threads would require pervasive locking → frame drops
- One frame of latency (~16ms) is negligible for AI interaction
- Queue depth is monitored; if >10 pending, HTTP returns `503 Service Unavailable`

### Request/Response Format

All endpoints accept/return `application/json`.

**Success:**
```json
HTTP 200
{
  "ok": true,
  "data": { ... }
}
```

**Error:**
```json
HTTP 400
{
  "ok": false,
  "error": {
    "code": "NODE_NOT_FOUND",
    "message": "Node id 99 not found"
  }
}
```

**Async operation (202 Accepted):**
```json
HTTP 202
{
  "ok": true,
  "status": "queued",
  "task_id": "segment_2_1712345678",
  "queue_position": 3
}
```

---

## Layer 2: REST API Endpoints

### 2.1 Project

| Method | Path | Body | Response | Description |
|--------|------|------|----------|-------------|
| `GET` | `/api/v1/project` | — | `{path, node_count, memory_mb, dirty}` | Current project info |
| `POST` | `/api/v1/project/open` | `{"path":"C:/proj.kgs"}` | `{node_count}` | Open project |
| `POST` | `/api/v1/project/save` | `{}` | `{}` | Save to current path |
| `POST` | `/api/v1/project/save-as` | `{"path":"C:/other/"}` | `{}` | Save to new path |
| `POST` | `/api/v1/project/create` | `{"path":"C:/new/"}` | `{}` | Create empty project |

### 2.2 Nodes

| Method | Path | Body | Response | Description |
|--------|------|------|----------|-------------|
| `GET` | `/api/v1/nodes` | — | `{"nodes":[{id,title,source_type,segment_mode,children,triangle_count},...]}` | List all |
| `GET` | `/api/v1/nodes/:id` | — | Full node state object | Get one |
| `POST` | `/api/v1/nodes` | `{}` | `{"id":5}` | Create empty |
| `DELETE` | `/api/v1/nodes/:id` | — | `{}` | Delete (cascade) |
| `POST` | `/api/v1/nodes/:id/duplicate` | `{}` | `{"new_id":6}` | Duplicate |
| `PATCH` | `/api/v1/nodes/:id` | `{"title":"new name"}` | `{}` | Update fields |
| `GET` | `/api/v1/nodes/:id/children` | — | `{"children":[3,4]}` | Direct children |
| `GET` | `/api/v1/nodes/:id/bounds` | — | `{"min":{"x","y","z"},"max":{...}}` | World bbox |

**Full node state** (`GET /api/v1/nodes/:id`):
```json
{
  "id": 2,
  "title": "head",
  "source_type": 0,
  "source_node_id": -1,
  "segment_mode": "collision",
  "repair_mode": "fill_holes",
  "alpha_wrap_alpha": 1.0,
  "alpha_wrap_offset": 0.01,
  "subdivide_level": 1,
  "children": [5, 6],
  "root_id": 0,
  "dirty": false,
  "mesh": {
    "triangle_count": 15234,
    "stl_path": "",
    "cached": true
  },
  "voxel": {
    "has_grid": true,
    "grid_size": [128, 128, 128]
  },
  "visibility": {
    "origin_mesh": false,
    "mesh": true,
    "exported_mesh": true,
    "voxel": true,
    "collision": true
  }
}
```

### 2.3 Mesh Operations

| Method | Path | Body | Response | Description |
|--------|------|------|----------|-------------|
| `POST` | `/api/v1/mesh/import` | `{"node_id":2,"path":"C:/in.stl","voxel_size":0.5,"load_mode":"default","load_as_sdf":false}` | `{"triangle_count":15234}` | Import STL into node |
| `POST` | `/api/v1/mesh/export` | `{"node_id":2,"path":"C:/out.stl","mode":"standard","simplify":false}` | `{"file_path":"C:/out.stl"}` | Export node to STL |
| `POST` | `/api/v1/mesh/export-all` | `{"export_dir":"C:/out/","mode":"standard"}` | `{"exported":5}` | Export all nodes |
| `POST` | `/api/v1/mesh/repair` | `{"node_id":2,"method":"alpha_wrap","alpha":1.0,"offset":0.01}` | `{"result_triangle_count":14890}` | Repair mesh |
| `POST` | `/api/v1/mesh/subdivide` | `{"node_id":2,"level":2}` | `{"result_triangle_count":60800}` | Subdivide |
| `POST` | `/api/v1/mesh/simplify` | `{"node_id":2,"ratio":0.5}` | `{"result_triangle_count":7600}` | Simplify |
| `POST` | `/api/v1/mesh/boolean-union` | `{"node_a":2,"node_b":3}` | `{"result_triangle_count":20000}` | Boolean union |
| `GET` | `/api/v1/mesh/:id/is-manifold` | — | `{"is_manifold":true,"issues":""}` | Watertight check |

**Repair methods:** `alpha_wrap`, `fill_holes`, `stitch_borders`, `merge_vertices`, `orient_volume`

**Import load modes:** `default`, `silhouette`, `surface_only`, `mesh_only`, `convex_hull`

### 2.4 Segmentation

| Method | Path | Body | Response | Description |
|--------|------|------|----------|-------------|
| `POST` | `/api/v1/nodes/:id/segment/configure` | `{"mode":"collision","collision":{...}}` | `{}` | Set segment mode + params |
| `POST` | `/api/v1/nodes/:id/segment/execute` | `{}` | `{"child_ids":[5,6]}` | Run segmentation |
| `POST` | `/api/v1/segment/execute-all` | `{}` | `{"results":[{"node_id":2,"child_ids":[5,6]}]}` | Process all pending |
| `GET` | `/api/v1/nodes/:id/segment/children` | — | `{"children":[{id,title,triangle_count}]}` | Get results (non-blocking) |

**Segment mode-specific body** (for `/segment/configure`):

```json
// collision mode
{"mode":"collision","collision":{"group":{...}}}

// plane mode
{"mode":"plane","plane":{"normal":{"x":0,"y":1,"z":0},"offset":2.5}}

// chain mode
{"mode":"chain","chain":{"min_radius":1,"use_cgal_skeleton":true,"picked_points":[...]}}

// repair mesh mode
{"mode":"repair_mesh","repair":{"method":"alpha_wrap","alpha":1.0,"offset":0.01}}

// subdivide mesh mode
{"mode":"subdivide_mesh","subdivide":{"level":2}}
```

### 2.5 SDF

| Method | Path | Body | Response | Description |
|--------|------|------|----------|-------------|
| `GET` | `/api/v1/nodes/:id/sdf` | — | `{"sdf_type":"SDF_Mesh","info":"..."}` | SDF tree info |
| `POST` | `/api/v1/nodes/:id/sdf/from-node` | `{"source_id":1,"subdivisions":2}` | `{}` | Load SDF from another node |
| `POST` | `/api/v1/nodes/:id/sdf/from-stl` | `{"path":"C:/in.stl","precision":"fast"}` | `{}` | Load STL as SDF |
| `POST` | `/api/v1/nodes/:id/sdf/to-voxel` | `{"precision":"precise"}` | `{"voxel_size":[128,128,128]}` | SDF → voxel grid |
| `POST` | `/api/v1/nodes/:id/sdf/boolean` | `{"op":"union","other_node_id":3}` | `{}` | SDF boolean |

### 2.6 Voxel

| Method | Path | Body | Response | Description |
|--------|------|------|----------|-------------|
| `GET` | `/api/v1/nodes/:id/voxel` | — | `{"has_grid":true,"size":[128,128,128]}` | Voxel info |
| `POST` | `/api/v1/nodes/:id/voxel/to-mesh` | `{"isolevel":0.5,"smooth_normals":true}` | `{"triangle_count":12000}` | Voxel → mesh |
| `POST` | `/api/v1/nodes/:id/voxel/brush` | `{"position":{"x":0,"y":0,"z":0},"radius":3.0,"mode":"add"}` | `{}` | Brush voxels |
| `POST` | `/api/v1/nodes/:id/voxel/compute-surface` | `{}` | `{}` | Compute surface cache |

### 2.7 Addon (Hair)

| Method | Path | Body | Response | Description |
|--------|------|------|----------|-------------|
| `GET` | `/api/v1/nodes/:id/addon/strands` | — | `{"strands":[{index,name,guide_point_count,repair_failed}]}` | List strands |
| `GET` | `/api/v1/nodes/:id/addon/strands/:idx` | — | Full strand data | Get one strand |
| `POST` | `/api/v1/nodes/:id/addon/strands` | `{"name":"strand_1"}` | `{"strand_index":0}` | Create strand |
| `DELETE` | `/api/v1/nodes/:id/addon/strands/:idx` | — | `{}` | Delete strand |
| `PATCH` | `/api/v1/nodes/:id/addon/strands/:idx` | `{"repair_alpha":1.5,"guide_samples":64}` | `{}` | Update params |
| `PUT` | `/api/v1/nodes/:id/addon/strands/:idx/guide-points` | `{"points":[{x,y,z},...]}` | `{}` | Set guide curve |
| `POST` | `/api/v1/nodes/:id/addon/strands/:idx/width-points` | `{"world_pos":{"x":0,"y":0,"z":5}}` | `{"width_index":3}` | Add width point |
| `PATCH` | `/api/v1/nodes/:id/addon/strands/:idx/width-points/:widx` | `{"scale":1.5}` | `{}` | Edit width point |
| `PATCH` | `/api/v1/nodes/:id/addon` | `{"base_node_id":0,"center":{"x":0,"y":0,"z":0}}` | `{}` | Set addon config |
| `PATCH` | `/api/v1/nodes/:id/addon/mode` | `{"reveal":true,"split":true,"sdf_boolean":true}` | `{}` | Set boolean mode |
| `GET` | `/api/v1/nodes/:id/addon/bounds` | — | `{"min":{...},"max":{...}}` | Hair world bounds |
| `GET` | `/api/v1/nodes/:id/addon/ready` | — | `{"boolean_ready":false}` | Boolean-ready check |
| `POST` | `/api/v1/nodes/:id/addon/build-sdf` | `{}` | `{}` | Build SDF from strands |

### 2.8 Scene

| Method | Path | Body | Response | Description |
|--------|------|------|----------|-------------|
| `POST` | `/api/v1/scene/raycast` | `{"origin":{x,y,z},"direction":{x,y,z},"max_dist":1000}` | `{"hit":true,"node_id":2,"position":{...},"normal":{...}}` | Raycast |
| `GET` | `/api/v1/scene/bounds` | — | `{"min":{...},"max":{...}}` | Visible bounds |
| `GET` | `/api/v1/scene/mouse` | — | `{"position":{x,y,z},"valid":true}` | Current mouse world pos |
| `PATCH` | `/api/v1/scene/visibility` | `{"mesh":true,"voxel":false}` | `{}` | Global visibility |

### 2.9 System

| Method | Path | Body | Response | Description |
|--------|------|------|----------|-------------|
| `GET` | `/api/v1/system/status` | — | `{"queue_running":true,"queue_progress":0.45,"fps":60,"memory_mb":512}` | Health check |
| `GET` | `/api/v1/system/queue` | — | `{"running":true,"progress":0.45,"status":"segmenting...","pending":3}` | Queue detail |
| `GET` | `/api/v1/system/log` | `?lines=20` | `{"log":"..."}` | Recent log lines |
| `POST` | `/api/v1/system/wait-idle` | `{"timeout_ms":5000}` | `{"timeout":false}` | Block until idle |
| `POST` | `/api/v1/system/toast` | `{"message":"Done!","duration_ms":2000}` | `{}` | Show on-screen toast message |
| `GET` | `/api/v1/system/render-id` | — | `{"render_id":2}` | Currently viewed node |

### 2.10 Flow (Workflow)

| Method | Path | Body | Response | Description |
|--------|------|------|----------|-------------|
| `PUT` | `/api/v1/flow/inputs` | `{"entries":[{node_id,file_path},...]}` | `{}` | Set inputs |
| `PUT` | `/api/v1/flow/outputs` | `{"entries":[{node_id,file_path},...]}` | `{}` | Set outputs |
| `GET` | `/api/v1/flow/order` | — | `{"order":[2,3,5]}` | Execution order |
| `POST` | `/api/v1/flow/execute` | `{}` | `{"results":[{node_id,output_path}]}` | Run workflow |

---

## Layer 3: WebSocket (Progress Streaming)

**Endpoint:** `ws://127.0.0.1:18920/api/v1/ws`

The WebSocket pushes real-time progress to connected clients. No polling needed.

### Server → Client Messages

```json
{"type":"queue.progress","data":{"task":"segment","node_id":2,"percent":0.45,"status":"meshing..."}}
{"type":"queue.complete","data":{"task":"segment","node_id":2,"result":{"child_ids":[5,6]}}}
{"type":"queue.error","data":{"task":"segment","node_id":2,"error":"alpha_wrap returned empty mesh"}}
{"type":"node.created","data":{"id":7,"title":""}}
{"type":"node.deleted","data":{"id":3}}
{"type":"node.updated","data":{"id":2,"changes":{"title":"renamed"}}}
{"type":"project.loaded","data":{"path":"C:/proj.kgs","node_count":8}}
{"type":"toast","data":{"message":"Export complete","duration_ms":2000}}
```

### Client → Server Messages

```json
{"type":"subscribe","data":{"events":["queue.*","node.*"]}}
{"type":"unsubscribe","data":{"events":["queue.*"]}}
```

---

## Layer 4: Implementation Plan

### File Organization

```
dep/cpp-httplib/
└── httplib.h              ← single-header HTTP/WS library (MIT)

src/kigstudio/agent/
├── agent_server.h         // HttpServer wrapper: start/stop/port config
├── agent_server.cpp
├── agent_queue.h          // AgentCommand + thread-safe command queue
├── agent_queue.cpp
├── agent_router.h         // Route registration: maps URL patterns to handlers
├── agent_router.cpp
├── agent_handler_project.h   // project endpoints
├── agent_handler_node.h      // node CRUD endpoints
├── agent_handler_mesh.h      // mesh operation endpoints
├── agent_handler_segment.h   // segmentation endpoints
├── agent_handler_sdf.h       // SDF endpoints
├── agent_handler_voxel.h     // voxel endpoints
├── agent_handler_addon.h     // addon/hair endpoints
├── agent_handler_scene.h     // scene query endpoints
├── agent_handler_system.h    // system/status endpoints
├── agent_handler_flow.h      // workflow endpoints
└── agent_handler_ws.h        // WebSocket event broadcasting

ui/
└── render_voxel_list.h       ← + process_agent_commands() call in render loop
```

### Integration Points

**1. `ui/main.cpp`** — Start HTTP server after SDL/bgfx init:
```cpp
sinriv::kigstudio::agent::AgentServer agent_server;
agent_server.start(18920);  // non-blocking, spawns worker threads
```

**2. Main render loop** — Poll agent commands each frame:
```cpp
while (running) {
    agent_server.process_commands(list);  // execute pending, signal promises
    // ... existing SDL/ImGui/bgfx rendering ...
}
```

**3. `src/kigstudio/agent/agent_queue.h`** — Core thread-safe command queue:
```cpp
struct AgentCommand {
    std::string method;       // e.g. "node.get"
    cJSON* params;            // request body as cJSON
    std::promise<cJSON*> result_promise;  // fulfilled by main thread
    int timeout_ms = 30000;
};

class AgentCommandQueue {
    std::queue<AgentCommand> queue_;
    std::mutex mutex_;
    std::condition_variable cv_;
public:
    void push(AgentCommand&& cmd);    // HTTP thread calls this
    void process_all(RenderVoxelList& list);  // Main thread calls this
};
```

### Dependencies

| Dependency | Current Status | Action |
|------------|---------------|--------|
| cpp-httplib | **Not present** | Add `dep/cpp-httplib/httplib.h` (single file, MIT) |
| cJSON | Already vendored at `dep/cJSON/` | Use as-is for JSON parse/format |
| Boost.Beast | Boost already in dep tree | **Not needed** — cpp-httplib is simpler |

### Key Design Decisions

1. **Single-threaded state, multi-threaded HTTP**: All state changes happen on the main thread. HTTP worker threads only parse requests and format responses — they never touch `RenderVoxelList` directly.

2. **Promise-based synchronization**: Each command carries a `std::promise<cJSON*>`. The main thread fulfills it after execution. HTTP thread awaits with timeout (30s default).

3. **Read-only fast path**: Simple queries (`node.list`, `system.status`) can run directly on the HTTP thread by briefly acquiring the existing `locker` mutex, avoiding one frame of latency. But this is an optimization for later — initially all commands go through the queue for safety.

4. **Event broadcasting**: WebSocket clients receive progress + state change events. The main thread pushes events to a ring buffer; the HTTP thread drains it to all WebSocket connections.

---

## Layer 5: Agent Usage Patterns

### Pattern A: Python Agent (requests library)

```python
import requests
import json

BASE = "http://127.0.0.1:18920/api/v1"

# Open project
requests.post(f"{BASE}/project/open", json={"path": "C:/my_project.kgs"})

# List nodes
nodes = requests.get(f"{BASE}/nodes").json()["data"]["nodes"]

# Repair a node
requests.post(f"{BASE}/mesh/repair", json={
    "node_id": 2,
    "method": "alpha_wrap",
    "alpha": 1.0,
    "offset": 0.01
})

# Wait for completion
requests.post(f"{BASE}/system/wait-idle", json={"timeout_ms": 10000})

# Export
requests.post(f"{BASE}/mesh/export", json={
    "node_id": 2,
    "path": "C:/output/repaired.stl"
})

# Show toast to user
requests.post(f"{BASE}/system/toast", json={
    "message": "Repair complete!",
    "duration_ms": 2000
})
```

### Pattern B: Real-time Progress via WebSocket

```python
import websocket
import json
import threading

def on_message(ws, msg):
    event = json.loads(msg)
    print(f"[{event['type']}] {event['data']}")

ws = websocket.WebSocketApp("ws://127.0.0.1:18920/api/v1/ws",
    on_message=on_message)
threading.Thread(target=ws.run_forever, daemon=True).start()

# Now trigger work and watch progress
requests.post("http://127.0.0.1:18920/api/v1/nodes/2/segment/execute")
# Console shows:
# [queue.progress] {'task': 'segment', 'node_id': 2, 'percent': 0.3, ...}
# [queue.progress] {'task': 'segment', 'node_id': 2, 'percent': 0.7, ...}
# [queue.complete] {'task': 'segment', 'node_id': 2, 'result': {'child_ids': [5, 6]}}
# [node.created] {'id': 5, ...}
# [node.created] {'id': 6, ...}
```

### Pattern C: Claude/GPT Function Calling

The API fits naturally into LLM function-calling schemas. Example system prompt:

```
You control KigStudio via HTTP API at http://127.0.0.1:18920/api/v1.
Available tools:
- get_nodes(): GET /nodes → list all scene nodes
- get_node(id): GET /nodes/:id → full node state
- repair_mesh(node_id, method, alpha?, offset?): POST /mesh/repair
- import_stl(node_id, path, voxel_size?): POST /mesh/import
- segment(node_id): POST /nodes/:id/segment/execute
- export_stl(node_id, path): POST /mesh/export
- wait_idle(timeout_ms): POST /system/wait-idle
- show_toast(message): POST /system/toast

All changes are visible on screen in real time.
```

---

## Implementation Phases

| Phase | Content | Files | Effort |
|-------|---------|-------|--------|
| **Phase 1** | cpp-httplib integration, `AgentCommandQueue`, main loop hook, `GET /system/status` | 4 files | Small |
| **Phase 2** | `project.*` + `node.list/get/create` + `mesh.import/export` | 3 handler files | Medium |
| **Phase 3** | `mesh.repair/subdivide/simplify` + `segment.*` + `node.*` full CRUD | 3 handler files | Medium |
| **Phase 4** | `sdf.*` + `voxel.*` + `addon.*` basic | 3 handler files | Medium |
| **Phase 5** | `addon.*` full + `scene.*` + `flow.*` + `toast` | 3 handler files | Medium |
| **Phase 6** | WebSocket events, read-only fast path, error polish, CORS headers | 2 files | Small |

---

## Future Extensions

- **CORS support**: Allow browser-based AI tools to connect directly
- **Static file serving**: Serve a built-in web dashboard at `http://127.0.0.1:18920/`
- **MCP adapter**: Thin translation layer for Claude Desktop's Model Context Protocol
- **Remote access**: Optional TLS + authentication for remote AI agent control
- **Batch requests**: `POST /api/v1/batch` accepting multiple commands in one round-trip
