# 屏幕底部消息框：  

悬浮于窗口，消息框中心点水平位置在屏幕中心，垂直位置在靠下1/3位置  
窗口没有边框，有一个胶囊形背景，不接受焦点  
全局渲染类中提供一个函数进行管理，调用时显示消息，并在一段时间（默认一秒）后淡出  
所有事件完成时利用这个提示，不只在日志中输出  

# 参考几何体：  

添加参考几何体功能，不属于任何节点，在文件tab中可以选择显示哪个，可以设置当前节点是否允许被鼠标捕获  
刷选参考几何体，自动计算guide_curve和宽度  

# 状态保存文件：  

在user目录中留下状态保存文件  
顶部文件tab中可以看到最近打开过的文件和工程  

# AI agent：

详见 [docs/ai-agent-api.md](docs/ai-agent-api.md)

**方案概要（HTTP + Live GUI）：**
- **运行模式**：kigstudio.exe 正常启动带 GUI，内嵌 HTTP server 监听 `127.0.0.1:18920`
- **传输层**：REST API（cpp-httplib 单头文件库）+ WebSocket 推送进度事件
- **线程模型**：HTTP 线程接收请求 → 推入命令队列 → 主线程每帧 poll 执行 → 返回结果。所有状态修改在主线程完成，操作在屏幕上实时可见
- **API 设计**：RESTful，10 个资源路径覆盖全部功能 —— `/nodes` `/mesh` `/sdf` `/voxel` `/addon` `/scene` `/project` `/segment` `/flow` `/system`
- **AI 集成**：Python `requests` 库即用、WebSocket 实时进度、LLM function-calling schema 直接适配
- **分阶段实现**：Phase 1 搭 HTTP server + 命令队列，Phase 2-3 核心节点/网格 API，Phase 4-6 补齐其余模块 + WebSocket 事件推送

