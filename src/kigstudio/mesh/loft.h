#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include "kigstudio/utils/vec2.h"
#include "kigstudio/utils/vec3.h"
#include "kigstudio/voxel/voxel2mesh.h"

namespace sinriv::kigstudio::mesh::loft {

using vec2f = sinriv::kigstudio::vec2<float>;
using vec3f = sinriv::kigstudio::vec3<float>;
using Triangle = sinriv::kigstudio::voxel::triangle_bvh<float>::triangle;
/*
* 附加件编辑界面：
* 文件Tab加载模式中加入“附加件”选项，可以选择一个有SDF的节点作为底模（不允许直接使用文件）
* 选择此模式时，出现新的“附加件”窗口（Imgui Begin启动新窗口，在render_object_editor_addons中渲染）
* 渲染器中新增RenderMesh的vector addon_renderer，用于管理多个mesh网格，此成员渲染位于鼠标捕获之后，不会参与鼠标拾取
* 操作流程：
* 1.选择一个预设截面样式，允许编辑截面，支持贝塞尔曲线
* 	当鼠标放在宽度编辑器某一栏上时，对应的绿线变成青色(0,1,1)
* 2.使用鼠标在底模上从发根开始依次拾取点，系统通过贝塞尔曲线插值成guide_curve
* 3.生成截面：
* 	在底模上拾取一个点，计算这个点离guide_curve的距离，作为发束的宽度
* 	利用计算的宽度，把预设截面样式复制到这个点，并按宽度进行缩放，作为截面
*    	宽度编辑器加一个参数，可以手动设置截面向量的长度
*    	中心点存在时，添加第一个宽度向量会自动计算截面旋转角度；
*    	每个宽度向量有两个按钮，可沿到中心点的连线移动向量端点
* 4.发根与反翘
*  	中心点：选择一个点作为中心点，一个节点所有发束共享中心点
*   每个关键点新增一组按钮和数字编辑器，可以在关键点到中心点的向量上移动
*   每个截面点新增一个按钮，可以自动计算旋转参数，使中心点位于当前位置曲线的方向与当前截面（在世界坐标系中）的正下方构成的平面上
*	
* 发束有先后顺序，可以手动调节
*
* 每一个参数除了鼠标选择外，还提供文本编辑框和历史记录功能
* 操作时实时显示效果，文件tab中允许设置体素大小(体素化时也用SDF来判断内外)和渲染SDF
* 每根毛发独占一个addon_renderer
* 毛发节点自带sdf，为每根毛发的SDF_Mesh的并集
* 附加件的碰撞编辑器不再是下拉框选择模式，而是两个勾选框：
*	显露：模型的SDF会减去底模（其下“SDF布尔”默认勾选；未勾选时改用CGAL几何布尔减底模，
*	生成的子节点直接渲染布尔后的三角形网格）
*   拆分：点击更新碰撞以后，每根发束会生成独立节点，并且每一根发束还会减去所有排在它前面的发束（勾选显露以后会在之后减去底模）
*	（其下“SDF拆分”默认勾选；未勾选时发束之间改用CGAL几何布尔相减）
* 导出STL时附加件节点可选择“网格（直接导出）”（直接导出loft三角形网格，
* 不经体素/SDF重采样）或“SDF 平滑”（用毛发SDF生成平滑网格）
* 宽度编辑器每一个向量增加独立的截面编辑器，只能拖动顶点，不能新增和删除，不允许自相交，编辑全局的截面后失效
*
* TODO:
* 添加参考几何体功能，不属于任何节点，在文件tab中可以选择显示哪个，可以设置当前节点是否允许被鼠标捕获
* 刷选参考几何体，自动计算guide_curve和宽度
* 
*/
struct LoftSection {
	// Origin is guide_curve[guide_vertex_id].
	size_t guide_vertex_id = 0;

	// Local 2D axes in world space. They are normalized before use; path
	// coordinates keep their own length units.
	vec3f axis_u = {1.0f, 0.0f, 0.0f};
	vec3f axis_v = {0.0f, 1.0f, 0.0f};

	// Closed section path. All sections must have the same vertex count and
	// matching point order.
	std::vector<vec2f> path;
};

struct LoftOptions {
	bool cap_first = true;
	bool cap_last = true;
};

std::vector<Triangle> build_loft_mesh(
    const std::vector<vec3f>& guide_curve,
    const std::vector<LoftSection>& sections,
    const LoftOptions& options = {},
    const std::function<bool()>& should_continue = {});

}  // namespace sinriv::kigstudio::mesh::loft
