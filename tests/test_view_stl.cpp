#include <OgreApplicationContext.h>
#include <OgreImGuiOverlay.h>
#include <OgreInput.h>
#include <imgui.h>
#include <Ogre.h>
#include <OgreOverlayManager.h>
#include <OgreOverlay.h>

#include "kigstudio/voxel/voxel2mesh.h"
#include "tinyfiledialogs.h"

using namespace Ogre;
using namespace OgreBites;

class STLViewer : public ApplicationContext, public InputListener {
   public:
    STLViewer() : ApplicationContext("STL Viewer") {}

    SceneManager* scnMgr = nullptr;
    Camera* cam = nullptr;
    SceneNode* modelNode = nullptr;

    float yaw = 0;
    float pitch = 0;
    float distance = 200;
    bool mouseDown = false;

    void setup() override {
        ApplicationContext::setup();
        addInputListener(this);

        scnMgr = getRoot()->createSceneManager();

        // ===== 启用 ImGui =====
        auto imgui = new Ogre::ImGuiOverlay();
        imgui->show();
        Ogre::OverlayManager::getSingleton().addOverlay(imgui);

        // ===== Camera =====
        cam = scnMgr->createCamera("cam");
        cam->setNearClipDistance(0.1);
        cam->setAutoAspectRatio(true);

        SceneNode* camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        camNode->attachObject(cam);
        camNode->setPosition(0, 0, distance);

        getRenderWindow()->addViewport(cam);

        // ===== Light =====
        Light* light = scnMgr->createLight();
        SceneNode* lightNode =
            scnMgr->getRootSceneNode()->createChildSceneNode();
        lightNode->setPosition(20, 50, 100);
        lightNode->attachObject(light);

        // 灰色背景（防止误以为程序没运行）
        getRenderWindow()->getViewport(0)->setBackgroundColour(
            ColourValue(0.2f, 0.2f, 0.2f));

        std::cout << "Press O to open STL file\n";
    }


    bool frameRenderingQueued(const FrameEvent& evt) override {
        return true;
    }

    // ===== 按键打开 STL =====
    bool keyPressed(const KeyboardEvent& evt) override {
        // std::cout << "keyPressed: " << evt.keysym.sym << std::endl;
        if (evt.keysym.sym == 'o' || evt.keysym.sym == 'O') {
            const char* file =
                tinyfd_openFileDialog("Open STL", "", 0, NULL, "STL file", 0);
            if (file)
                std::cout << "Open STL file: " << file << std::endl;
                loadSTL(file);
                std::cout << "STL loaded\n";
        }
        return true;
    }

    // ===== 鼠标旋转 =====
    bool mousePressed(const MouseButtonEvent& evt) override {
        if (evt.button == BUTTON_LEFT)
            mouseDown = true;
        return true;
    }

    bool mouseReleased(const MouseButtonEvent& evt) override {
        if (evt.button == BUTTON_LEFT)
            mouseDown = false;
        return true;
    }

    bool mouseMoved(const MouseMotionEvent& evt) override {
        if (!modelNode || !mouseDown)
            return true;

        yaw += evt.xrel * 0.3f;
        pitch += evt.yrel * 0.3f;

        modelNode->setOrientation(Quaternion(Degree(pitch), Vector3::UNIT_X) *
                                  Quaternion(Degree(yaw), Vector3::UNIT_Y));

        return true;
    }

    // ===== 滚轮缩放 =====
    bool mouseWheelRolled(const MouseWheelEvent& evt) override {
        distance -= evt.y * 10;
        if (distance < 10)
            distance = 10;

        cam->getParentSceneNode()->setPosition(0, 0, distance);
        return true;
    }

    // ===== 读取 STL 并显示 =====
    void loadSTL(const std::string& filename) {
        std::vector<Vector3> vertices;
        std::vector<Vector3> normals;

        // 读取 STL
        for (auto [tri, n] : sinriv::kigstudio::voxel::readSTL(filename)) {
            vertices.push_back(Vector3(std::get<0>(tri).x, std::get<0>(tri).y, std::get<0>(tri).z));
            vertices.push_back(Vector3(std::get<1>(tri).x, std::get<1>(tri).y, std::get<1>(tri).z));
            vertices.push_back(Vector3(std::get<2>(tri).x, std::get<2>(tri).y, std::get<2>(tri).z));

            normals.push_back(Vector3(n.x, n.y, n.z));
            normals.push_back(Vector3(n.x, n.y, n.z));
            normals.push_back(Vector3(n.x, n.y, n.z));
        }

        if (vertices.empty()) {
            std::cout << "No vertices loaded from STL.\n";
            return;
        }

        std::cout << "STL has " << vertices.size() << " vertices\n";

        // 创建 Mesh
        MeshPtr mesh = MeshManager::getSingleton().createManual(
            "stlMesh", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

        SubMesh* sub = mesh->createSubMesh();
        sub->useSharedVertices = false;

        // 顶点数据
        sub->vertexData = new VertexData();
        sub->vertexData->vertexCount = vertices.size();

        VertexDeclaration* decl = sub->vertexData->vertexDeclaration;
        size_t offset = 0;
        decl->addElement(0, offset, VET_FLOAT3, VES_POSITION);
        offset += VertexElement::getTypeSize(VET_FLOAT3);
        decl->addElement(0, offset, VET_FLOAT3, VES_NORMAL);

        // 创建顶点缓冲
        auto vbuf = HardwareBufferManager::getSingleton().createVertexBuffer(
            sizeof(Vector3) * 2, vertices.size(), HardwareBuffer::HBU_STATIC_WRITE_ONLY);

        uint8* ptr = static_cast<uint8*>(vbuf->lock(HardwareBuffer::HBL_DISCARD));
        for (size_t i = 0; i < vertices.size(); i++) {
            memcpy(ptr, &vertices[i], sizeof(Vector3));
            ptr += sizeof(Vector3);
            memcpy(ptr, &normals[i], sizeof(Vector3));
            ptr += sizeof(Vector3);
        }
        vbuf->unlock();
        sub->vertexData->vertexBufferBinding->setBinding(0, vbuf);

        // 创建索引数据
        sub->indexData = new IndexData();
        sub->indexData->indexCount = vertices.size();
        sub->indexData->indexBuffer =
            HardwareBufferManager::getSingleton().createIndexBuffer(
                HardwareIndexBuffer::IT_32BIT, vertices.size(),
                HardwareBuffer::HBU_STATIC_WRITE_ONLY);

        uint32* idx = static_cast<uint32*>(sub->indexData->indexBuffer->lock(HardwareBuffer::HBL_DISCARD));
        for (uint32 i = 0; i < vertices.size(); i++)
            idx[i] = i;
        sub->indexData->indexBuffer->unlock();

        // 自动计算包围盒和包围球
        AxisAlignedBox aabb;
        for (auto& v : vertices)
            aabb.merge(v);

        mesh->_setBounds(aabb);
        mesh->_setBoundingSphereRadius(aabb.getMaximum().distance(aabb.getMinimum()) / 2);
        mesh->load();

        // 创建实体
        Entity* ent = scnMgr->createEntity(mesh);

        // 清理旧模型
        if (modelNode) {
            modelNode->removeAndDestroyAllChildren();
        }

        modelNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        modelNode->attachObject(ent);

        // 重置旋转和缩放
        yaw = 0;
        pitch = 0;
        modelNode->setOrientation(Quaternion::IDENTITY);

        std::cout << "STL loaded and entity attached.\n";
    }
};

int main() {
    STLViewer app;
    app.initApp();
    app.getRoot()->startRendering();
    app.closeApp();
}