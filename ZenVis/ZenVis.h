#pragma once
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <fmt/ostream.h>

#include <Eigen/Dense>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <queue>
#include <variant>

namespace zenvis {

struct DrawPointCommand {
    std::vector<float> xyzs;    // xyz X num
    std::vector<float> colors;  // optional: rgba X num or rgba
    float point_size{1.f};
    Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};
    boost::uuids::uuid handle = boost::uuids::random_generator()();
};

struct DrawAxesCommand {
    Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};
    float axis_len{1.f};
    float axis_size{1.f};
    boost::uuids::uuid handle = boost::uuids::random_generator()();
};

struct SetCameraPoseCommand {
    std::array<double, 3> eye{1, 1, 1};
    std::array<double, 3> target{0, 0, 0};
    std::array<double, 3> up{0, 0, 1};
};

struct DeleteNodeCommand {
    boost::uuids::uuid handle;
};

struct SetNodePoseCommand {
    boost::uuids::uuid handle;
    Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};
};

using ViewerCommand = std::variant<  //
    DrawPointCommand,                //
    DrawAxesCommand,                 //
    SetCameraPoseCommand,            //
    DeleteNodeCommand,               //
    SetNodePoseCommand               //
    >;

class ZenViewer {
    friend struct ViewerCommandVisitor;

public:
    ZenViewer();
    ~ZenViewer();
    void Show(bool blocking = false);

    static void PushCommand(ViewerCommand&& cmd);
    static void PushCommand(const ViewerCommand& cmd);

    static std::mutex sm_mutex_cmds;
    static std::atomic_bool sm_should_exit;

private:
    void RunViewer();

    static std::queue<ViewerCommand> sm_cmds;
    struct Pimpl;
    std::unique_ptr<Pimpl> m_pimpl;
};
}  // namespace zenvis

template <>
struct fmt::formatter<boost::uuids::uuid> : fmt::ostream_formatter {};