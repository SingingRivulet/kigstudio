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
* TODO:
* 在ui中添加附加件编辑界面：
* 文件Tab加载模式中加入“附加件”选项，可以选择一个有SDF的节点作为底模（不允许直接使用文件） 		已完成
* 选择此模式时，出现新的“附加件”窗口（Imgui Begin启动新窗口，在render_object_editor_addons中渲染） 		已完成
* 渲染器中新增RenderMesh的vector addon_renderer，用于管理多个mesh网格，此成员渲染位于鼠标捕获之后，不会参与鼠标拾取 		已完成
* 操作流程：
* 1.选择一个预设截面样式，创建发束（预设截面样式使用txt格式放在程序目录中，每个坐标一行，加载前自动归一化，cmake自动复制）
* 2.使用鼠标在底模上从发根开始依次拾取点，系统通过贝塞尔曲线插值成guide_curve 		已完成
* 3.生成截面：
* 	在底模上拾取一个点，计算这个点离guide_curve的距离，作为发束的宽度 		已完成
* 	利用计算的宽度，把预设截面样式复制到这个点，并按宽度进行缩放，作为截面
*   提供一个宽度极小值作为发梢（因为发梢难以点中）
* 4.发根与反翘
* 	发根：可选项，起点位置往底模法线反方向补一个关键点
*	反翘：可选项，终点位置往底模法线方向补一个关键点
* 发束有先后顺序，可以手动调节 		已完成
*
* 每一个参数除了鼠标选择外，还提供文本编辑框和历史记录功能 		已完成
* 操作时实时显示效果，文件tab中允许设置体素大小(体素化时也用SDF来判断内外)和渲染SDF
* 每根毛发独占一个addon_renderer
* 毛发节点自带sdf，为每根毛发的SDF_Mesh的并集
* 附加件的碰撞编辑器不再是下拉框选择模式，而是两个勾选框：
*	显露：模型的SDF会减去底模
*   拆分：点击更新碰撞以后，每根发束会生成独立节点，并且每一根发束还会减去所有排在它前面的发束（勾选显露以后会在之后减去底模）
*
* 在上面的调试完成以后再做：
* 刷选三角形，自动计算guide_curve和宽度
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
	bool orient_faces = true;
};

std::vector<Triangle> build_loft_mesh(
    const std::vector<vec3f>& guide_curve,
    const std::vector<LoftSection>& sections,
    const LoftOptions& options = {},
    const std::function<bool()>& should_continue = {});

}  // namespace sinriv::kigstudio::mesh::loft
