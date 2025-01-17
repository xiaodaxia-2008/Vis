#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/spdlog.h>

#include <Eigen/Dense>

#include <osg/Camera>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Light>
#include <osg/LightModel>
#include <osg/LightSource>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/Point>
#include <osg/PrimitiveSet>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osgGA/MultiTouchTrackballManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/config/SingleScreen>
#include <osgViewer/config/SingleWindow>

#include <atomic>
#include <chrono>
#include <queue>
#include <variant>

enum class CommandType {
    DRAW_POINTS,
    DRAW_AXES,
    SET_CAMERA_POSE,
};

struct DrawPointCommand {
    std::vector<float> xyzs;    // xyz X num
    std::vector<float> colors;  // rgba X num
    CommandType type{CommandType::DRAW_POINTS};
};

struct DrawAxesCommand {
    float axis_len{1.f};
    float axis_size{1.f};
    CommandType type{CommandType::DRAW_AXES};
};

struct SetCameraPoseCommand {
    std::array<double, 3> eye{1, 1, 1};
    std::array<double, 3> target{0, 0, 0};
    std::array<double, 3> up{0, 0, 1};
    CommandType type{CommandType::SET_CAMERA_POSE};
};

osg::ref_ptr<osg::Geometry> DrawAxes(const DrawAxesCommand& cmd,
                                     osgViewer::Viewer* viewer = nullptr) {
    osg::ref_ptr geo = new osg::Geometry();
    auto v = new osg::Vec3Array(6);
    (*v)[0] = osg::Vec3f(0.0f, 0.0f, 0.0f);
    (*v)[1] = osg::Vec3f(cmd.axis_len, 0.0f, 0.0f);
    (*v)[2] = osg::Vec3f(0.0f, 0.0f, 0.0f);
    (*v)[3] = osg::Vec3f(0.0f, cmd.axis_len, 0.0f);
    (*v)[4] = osg::Vec3f(0.0f, 0.0f, 0.0f);
    (*v)[5] = osg::Vec3f(0.0f, 0.0f, cmd.axis_len);

    auto c = new osg::Vec4Array(6);
    (*c)[0] = osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f);
    (*c)[1] = osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f);  // x red
    (*c)[2] = osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f);
    (*c)[3] = osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f);  // y green
    (*c)[4] = osg::Vec4f(0.0f, 0.0f, 1.0f, 1.0f);
    (*c)[5] = osg::Vec4f(0.0f, 0.0f, 1.0f, 1.0f);  // z blue

    geo->setVertexArray(v);
    geo->setColorArray(c);
    geo->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));
    geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 2, 2));
    geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 4, 2));
    geo->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(cmd.axis_size));
    geo->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    return geo;
}

osg::ref_ptr<osg::Geometry> DrawPoints(const DrawPointCommand& cmd,
                                       osgViewer::Viewer* viewer = nullptr) {
    osg::ref_ptr geo = new osg::Geometry;
    const int n = cmd.xyzs.size() / 3;
    geo->setVertexArray(new osg::Vec3Array(n, reinterpret_cast<const osg::Vec3*>(cmd.xyzs.data())));
    geo->setColorArray(new osg::Vec4Array(osg::Array::BIND_PER_VERTEX, n,
                                          reinterpret_cast<const osg::Vec4*>(cmd.colors.data())));
    geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, n));
    return geo;
}

void SetCameraPose(const SetCameraPoseCommand& cmd, osgViewer::Viewer* viewer) {
    auto manip = static_cast<osgGA::TrackballManipulator*>(viewer->getCameraManipulator());
    manip->setTransformation(osg::Vec3d(cmd.eye[0], cmd.eye[1], cmd.eye[2]),
                             osg::Vec3d(cmd.target[0], cmd.target[1], cmd.target[2]),
                             osg::Vec3d(cmd.up[0], cmd.up[1], cmd.up[2]));
}

using ViewerCommand = std::variant<DrawPointCommand, SetCameraPoseCommand, DrawAxesCommand>;

struct ViewerCommandVisitor {
    osgViewer::Viewer* viewer;
    osg::Geode* root_geode;

    void operator()(const DrawPointCommand& cmd) {
        auto geo = DrawPoints(cmd, viewer);
        root_geode->addDrawable(geo);
    }

    void operator()(const DrawAxesCommand& cmd) {
        SPDLOG_DEBUG("draw axes");
        auto geo = DrawAxes(cmd);
        root_geode->addDrawable(geo);
        viewer->home();
    }

    void operator()(const SetCameraPoseCommand& cmd) {
        SPDLOG_DEBUG("set camera pose");
        SetCameraPose(cmd, viewer);
    }
};

class ZenViewer {
public:
    ZenViewer() {
        m_root = new osg::Group;
        m_root_geode = new osg::Geode;
        m_root_geode->addDrawable(DrawAxes(DrawAxesCommand{}));
        m_root->addChild(m_root_geode);
    }

    ~ZenViewer() {
        sm_should_exit = true;
        if (m_thread.joinable())
            m_thread.join();
        SPDLOG_DEBUG("exit zen viewer");
    }

    void Show() { m_thread = std::thread(&ZenViewer::RunViewer, this); }
    osg::ref_ptr<osg::Group> GetSceneRoot() { return m_root; }

    static std::mutex sm_mutex_cmds;
    static std::atomic_bool sm_should_exit;
    static std::queue<ViewerCommand> sm_cmds;

    void RunViewer() {
        sm_should_exit = false;
        osgViewer::Viewer viewer;
        viewer.setUpViewInWindow(30, 30, 800, 800, 0);
        viewer.setCameraManipulator(new osgGA::TrackballManipulator);
        viewer.getCamera()->setClearColor(osg::Vec4(0.f, 0.f, 0.f, 1.f));
        viewer.setSceneData(m_root);
        viewer.home();
        auto tp = std::chrono::system_clock::now();
        while (!sm_should_exit) {
            viewer.frame();
            while (!sm_should_exit) {
                if (!sm_cmds.empty()) {
                    std::visit(ViewerCommandVisitor{&viewer, m_root_geode}, sm_cmds.front());
                    {
                        std::lock_guard<std::mutex> lock{sm_mutex_cmds};
                        sm_cmds.pop();
                    }
                }
                auto tp1 = std::chrono::system_clock::now();
                if (tp1 - tp > std::chrono::milliseconds(33)) {
                    tp = tp1;
                    break;
                }
            }
        }
    }

    // private:
    osg::ref_ptr<osg::Group> m_root;
    osg::ref_ptr<osg::Geode> m_root_geode;
    std::thread m_thread;
};

std::atomic_bool ZenViewer::sm_should_exit;
std::mutex ZenViewer::sm_mutex_cmds;
std::queue<ViewerCommand> ZenViewer::sm_cmds;

class ExitHandler : public osgGA::GUIEventHandler {
public:
    ExitHandler() {}

protected:
    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override {
        if (ea.getEventType() == osgGA::GUIEventAdapter::CLOSE_WINDOW) {
            ZenViewer::sm_should_exit = true;
        }
        return false;
    }
};

int main() {
    spdlog::set_level(spdlog::level::debug);

    ZenViewer zv;
    zv.Show();
    ZenViewer::sm_cmds.push(SetCameraPoseCommand{.eye = {1, 1, 1}, .target = {0, 0, 0}, .up = {0, 0, 1}});
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::this_thread::sleep_for(std::chrono::seconds(5));
    return 0;
}