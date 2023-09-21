#include "ZenVis.h"

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/spdlog.h>

#include <Eigen/Dense>

#include <osg/Camera>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/Point>
#include <osg/PrimitiveSet>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

#include <boost/uuid/uuid_hash.hpp>

#include <unordered_map>

namespace zenvis {

inline osg::Matrix MatrixEigenToOsg(const Eigen::Matrix4d& pose) {
    return osg::Matrix(pose.data());
}

osg::ref_ptr<osg::MatrixTransform> DrawAxes(const DrawAxesCommand& cmd,
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

    osg::ref_ptr mt = new osg::MatrixTransform(MatrixEigenToOsg(cmd.pose));
    mt->addChild(geo);
    return mt;
}

osg::ref_ptr<osg::MatrixTransform> DrawPoints(const DrawPointCommand& cmd,
                                              osgViewer::Viewer* viewer = nullptr) {
    osg::ref_ptr geo = new osg::Geometry;
    const int n = cmd.xyzs.size() / 3;
    geo->setVertexArray(new osg::Vec3Array(n, reinterpret_cast<const osg::Vec3*>(cmd.xyzs.data())));
    if (cmd.colors.size() == n * 4)
        geo->setColorArray(new osg::Vec4Array(
            osg::Array::BIND_PER_VERTEX, n, reinterpret_cast<const osg::Vec4*>(cmd.colors.data())));
    else if (cmd.colors.size() == 4)
        geo->setColorArray(new osg::Vec3Array(
            osg::Array::BIND_OVERALL, 1, reinterpret_cast<const osg::Vec3*>(cmd.colors.data())));
    geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, n));
    // set point size
    if (cmd.point_size != 1.f)
        geo->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(cmd.point_size));

    osg::ref_ptr mt = new osg::MatrixTransform(MatrixEigenToOsg(cmd.pose));
    mt->addChild(geo);
    return mt;
}

void SetCameraPose(const SetCameraPoseCommand& cmd, osgViewer::Viewer* viewer) {
    auto manip = static_cast<osgGA::TrackballManipulator*>(viewer->getCameraManipulator());
    manip->setTransformation(osg::Vec3d(cmd.eye[0], cmd.eye[1], cmd.eye[2]),
                             osg::Vec3d(cmd.target[0], cmd.target[1], cmd.target[2]),
                             osg::Vec3d(cmd.up[0], cmd.up[1], cmd.up[2]));
}

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

struct ZenViewer::Pimpl {
    osg::ref_ptr<osg::Group> root;
    std::unordered_map<boost::uuids::uuid, osg::Node*> node_map;
    std::thread td;
};

struct ViewerCommandVisitor {
    osgViewer::Viewer* viewer;
    ZenViewer::Pimpl& pimpl;
    using RT = void;

    RT operator()(const DrawPointCommand& cmd) {
        auto node = DrawPoints(cmd, viewer);
        pimpl.root->addChild(node);
        pimpl.node_map[cmd.handle] = node;
    }

    RT operator()(const DrawAxesCommand& cmd) {
        auto node = DrawAxes(cmd);
        pimpl.root->addChild(node);
        pimpl.node_map[cmd.handle] = node;
    }

    RT operator()(const SetCameraPoseCommand& cmd) { SetCameraPose(cmd, viewer); }

    RT operator()(const DeleteNodeCommand& cmd) {
        if (pimpl.node_map.contains(cmd.handle)) {
            pimpl.root->removeChild(pimpl.node_map[cmd.handle]);
            pimpl.node_map.erase(cmd.handle);
        } else {
            SPDLOG_WARN("cannot find node with handle {}", cmd.handle);
        }
    }

    RT operator()(const SetNodePoseCommand& cmd) {
        if (pimpl.node_map.contains(cmd.handle)) {
            static_cast<osg::MatrixTransform*>(pimpl.node_map[cmd.handle])
                ->setMatrix(MatrixEigenToOsg(cmd.pose));
        } else {
            SPDLOG_WARN("cannot find node with handle {}", cmd.handle);
        }
    }
};

ZenViewer::ZenViewer() {
    m_pimpl = std::make_unique<Pimpl>();
    m_pimpl->root = new osg::Group;
    sm_should_exit = true;
}

void ZenViewer::PushCommand(ViewerCommand&& cmd) {
    sm_cmds.push(std::move(cmd));
}

void ZenViewer::PushCommand(const ViewerCommand& cmd) {
    sm_cmds.push(cmd);
}

void ZenViewer::Show(bool blocking) {
    if (!sm_should_exit) {
        SPDLOG_DEBUG("a viewer is already running");
        return;
    }

    if (blocking)
        RunViewer();
    else
        m_pimpl->td = std::thread(&ZenViewer::RunViewer, this);
}

void ZenViewer::RunViewer() {
    sm_should_exit = false;
    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow(30, 30, 800, 800, 0);
    viewer.setCameraManipulator(new osgGA::TrackballManipulator);
    viewer.getCamera()->setClearColor(osg::Vec4(0.f, 0.f, 0.f, 1.f));
    viewer.setSceneData(m_pimpl->root);
    viewer.addEventHandler(new ExitHandler);
    viewer.home();
    auto tp = std::chrono::system_clock::now();
    while (!sm_should_exit) {
        viewer.frame();
        while (!sm_should_exit) {
            if (!sm_cmds.empty()) {
                std::visit(ViewerCommandVisitor{&viewer, *m_pimpl}, sm_cmds.front());
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
    SPDLOG_INFO("exit zen viewer");
}

ZenViewer::~ZenViewer() {
    sm_should_exit = true;
    if (m_pimpl->td.joinable())
        m_pimpl->td.join();
    SPDLOG_DEBUG("destruct ZenViewer at {}", fmt::ptr(this));
}

std::atomic_bool ZenViewer::sm_should_exit;
std::mutex ZenViewer::sm_mutex_cmds;
std::queue<ViewerCommand> ZenViewer::sm_cmds;

}  // namespace zenvis