# 参考几何体：  

添加参考几何体功能，不属于任何节点，在文件tab中可以选择显示哪个，可以设置当前节点是否允许被鼠标捕获  
刷选参考几何体，自动计算guide_curve和宽度  

# 特殊发型：

## 糖葫芦
引导线圆柱放样，每隔一段距离一个椭球，椭球长轴与表面的交点位于引导线上  
可选：在椭球结尾位置（远离起点那一边）加上关节，
关节结构参考`src\kigstudio\sdf\sdf_chain_joint.h`  

## 麻花辫
引导线圆柱放样，周围三根引导线编织
可选：每隔一段距离加上关节，
关节结构参考`src\kigstudio\sdf\sdf_chain_joint.h`  

这两种特殊发型结束点都会带一个桃形几何体

# 修复bug：

宽度编辑器中修改截面后，mesh会直接消失，同时输出异常
[addon_mesh] build failed for strand: Loft sections must have matching path sizes
