#include <Ogre.h>
#include <OgreWindowEventUtilities.h>

#include "kigstudio/voxel/voxelizer_svo.h"

using namespace Ogre;

/* =========================================================
   把 generateMesh() 输出变成 Ogre Mesh
   ========================================================= */
void buildVoxelMeshToOgre(SceneManager* sceneMgr,
                          sinriv::kigstudio::octree::Octree& voxelData)
{
    double isolevel = 0.5;
    int numTriangles = 0;

    ManualObject* manual = sceneMgr->createManualObject("voxelMesh");

    manual->begin("BaseWhite", RenderOperation::OT_TRIANGLE_LIST);

    for (auto [tri, normal] :
        sinriv::kigstudio::voxel::generateMesh(voxelData, isolevel, numTriangles, true))
    {
        auto [v0, v1, v2] = tri;

        manual->position((float)v0.x, (float)v0.y, (float)v0.z);
        manual->normal((float)normal.x, (float)normal.y, (float)normal.z);

        manual->position((float)v1.x, (float)v1.y, (float)v1.z);
        manual->normal((float)normal.x, (float)normal.y, (float)normal.z);

        manual->position((float)v2.x, (float)v2.y, (float)v2.z);
        manual->normal((float)normal.x, (float)normal.y, (float)normal.z);
    }

    manual->end();

    MeshPtr mesh = manual->convertToMesh("voxelMeshConverted");

    Entity* entity = sceneMgr->createEntity(mesh);

    SceneNode* node = sceneMgr->getRootSceneNode()->createChildSceneNode();
    node->attachObject(entity);

    std::cout << "Voxel mesh triangles: " << numTriangles << std::endl;
}

/* =========================================================
   主程序
   ========================================================= */
int main()
{
    std::cout << "Hello, Ogre!" << std::endl;
    Root* root = new Root("plugins.cfg", "ogre.cfg", "ogre.log");

    // OGRE 14 必须带参数
    if (!root->restoreConfig())
    {
        if (!root->showConfigDialog(nullptr))
            return 0;
    }

    RenderWindow* window = root->initialise(true, "Voxel Viewer");
    std::cout << "Window created." << std::endl;

    SceneManager* sceneMgr = root->createSceneManager();

    /* =========================
       创建相机（OGRE14 正确写法）
       ========================= */
    Camera* cam = sceneMgr->createCamera("MainCamera");

    SceneNode* camNode = sceneMgr->getRootSceneNode()->createChildSceneNode();
    camNode->attachObject(cam);

    camNode->setPosition(40, 40, 40);
    camNode->lookAt(Vector3(0, 0, 0), Node::TS_WORLD);

    cam->setNearClipDistance(0.1f);

    Viewport* vp = window->addViewport(cam);
    vp->setBackgroundColour(ColourValue(0.2f, 0.2f, 0.25f));

    cam->setAspectRatio(
        Real(vp->getActualWidth()) / Real(vp->getActualHeight()));

    /* =========================
       创建光照（OGRE14 正确写法）
       ========================= */
    Light* light = sceneMgr->createLight("MainLight");

    SceneNode* lightNode = sceneMgr->getRootSceneNode()->createChildSceneNode();
    lightNode->attachObject(light);

    light->setType(Light::LT_DIRECTIONAL);
    lightNode->setDirection(-1, -1, -1);

    sceneMgr->setAmbientLight(ColourValue(0.3f, 0.3f, 0.3f));

    /* =========================
       构造体素数据
       ========================= */
    sinriv::kigstudio::octree::Octree voxelData(128);

    sinriv::kigstudio::voxel::draw_triangle(
        voxelData,
        sinriv::kigstudio::voxel::Triangle({10,0,0}, {0,10,0}, {0,0,10}),
        sinriv::kigstudio::voxel::vec3f(0,0,0),
        1, 1, 1, 0.05);

    /* =========================
       直接显示体素重建 Mesh
       ========================= */
    buildVoxelMeshToOgre(sceneMgr, voxelData);
    std::cout << "Voxel mesh built." << std::endl;

    /* =========================
       渲染循环
       ========================= */
    while (true)
    {
        WindowEventUtilities::messagePump();

        if (!window->isVisible())
            break;

        if (!root->renderOneFrame())
            break;
    }
    std::cout << "Goodbye, Ogre!" << std::endl;

    delete root;
    return 0;
}