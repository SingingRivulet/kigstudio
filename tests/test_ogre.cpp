#include <OGRE/Ogre.h>
#include <OGRE/OgreInput.h>
#include <OGRE/OgreCameraMan.h>
#include <OGRE/OgreRTShaderSystem.h>
#include <OGRE/OgreOverlaySystem.h>

class MyApp
    : public OgreBites::ApplicationContext
    , public OgreBites::InputListener
{
public:
    MyApp() : OgreBites::ApplicationContext("Ogre Minimal Demo") {}

    void setup() override
    {
        // 先让基类做基础初始化（窗口、资源定位等）
        OgreBites::ApplicationContext::setup();

        // 输入监听
        addInputListener(this);

        // 获取 Root 与场景管理器
        Ogre::Root* root = getRoot();
        Ogre::SceneManager* scnMgr = root->createSceneManager();

        // Overlay（可用于调试/显示文字）
        auto* overlaySystem = new Ogre::OverlaySystem();
        scnMgr->addRenderQueueListener(overlaySystem);

        // RTShader System：自动为固定管线材质生成着色器
        auto* shaderGen = Ogre::RTShader::ShaderGenerator::getSingletonPtr();
        if (!shaderGen)
        {
            shaderGen = OGRE_NEW Ogre::RTShader::ShaderGenerator();
            Ogre::RTShader::ShaderGenerator::initialize();
        }
        shaderGen->addSceneManager(scnMgr);

        // 创建相机与摄像机控制
        Ogre::Camera* cam = scnMgr->createCamera("MainCam");
        cam->setNearClipDistance(0.5f);
        cam->setAutoAspectRatio(true);

        Ogre::SceneNode* camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        camNode->attachObject(cam);

        // 创建窗口视口
        getRenderWindow()->addViewport(cam);

        // CameraMan 提供 WASD/鼠标 控制
        mCameraMan = std::make_unique<OgreBites::CameraMan>(camNode);
        mCameraMan->setStyle(OgreBites::CS_ORBIT);
        addInputListener(mCameraMan.get());

        // 环境光与主光源
        scnMgr->setAmbientLight(Ogre::ColourValue(0.5f, 0.5f, 0.5f));
        Ogre::Light* light = scnMgr->createLight("MainLight");
        Ogre::SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        lightNode->setPosition(50, 100, 50);
        lightNode->attachObject(light);

        // 资源：OgreBites::ApplicationContext 会自动定位默认媒体目录（包含 ogrehead.mesh）
        // 加载资源组
        Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

        // 创建 ogre 头模型实体
        Ogre::Entity* ent = scnMgr->createEntity("OgreHead", "ogrehead.mesh");
        mModelNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        mModelNode->setPosition(0, 0, -20);
        mModelNode->attachObject(ent);

        // 让相机看向模型
        camNode->setPosition(0, 5, 40);
        camNode->lookAt(mModelNode->getPosition(), Ogre::Node::TS_WORLD);

        // 简单的天空颜色
        scnMgr->setSkyBox(false, "");
        scnMgr->setSkyDome(false, "");
    }

    bool keyPressed(const OgreBites::KeyboardEvent& evt) override
    {
        if (evt.keysym.sym == OgreBites::SDLK_ESCAPE)
        {
            getRoot()->queueEndRendering();
        }
        return true;
    }

    bool frameRenderingQueued(const Ogre::FrameEvent& evt) override
    {
        // 让模型慢慢旋转
        if (mModelNode)
            mModelNode->yaw(Ogre::Degree(15.0f * evt.timeSinceLastFrame));
        return true;
    }

private:
    std::unique_ptr<OgreBites::CameraMan> mCameraMan;
    Ogre::SceneNode* mModelNode{nullptr};
};

int main(int argc, char** argv)
{
    try
    {
        MyApp app;
        app.initApp();
        app.getRoot()->addFrameListener(&app);
        app.getRoot()->startRendering();
        app.closeApp();
    }
    catch (const std::exception& e)
    {
        fprintf(stderr, "Exception: %s\n", e.what());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
