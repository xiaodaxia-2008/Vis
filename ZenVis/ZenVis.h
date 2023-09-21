#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <queue>
#include <variant>

namespace zenvis {

enum class CommandType {
    DRAW_POINTS,
    DRAW_AXES,
    SET_CAMERA_POSE,
};

struct DrawPointCommand {
    std::vector<float> xyzs;    // xyz X num
    std::vector<float> colors;  // optional: rgba X num or rgba
    float point_size{1.f};
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

using ViewerCommand = std::variant<DrawPointCommand, SetCameraPoseCommand, DrawAxesCommand>;

class ZenViewer {
public:
    ZenViewer();
    ~ZenViewer();
    void Show(bool blocking = false);

    static std::mutex sm_mutex_cmds;
    static std::atomic_bool sm_should_exit;
    static std::queue<ViewerCommand> sm_cmds;

private:
    void RunViewer();

    struct Pimpl;
    std::unique_ptr<Pimpl> m_pimpl;
};
}  // namespace zenvis