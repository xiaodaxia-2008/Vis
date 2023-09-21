#include <ZenVis.h>
#include <fmt/ostream.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <random>

int main() {
    using namespace zenvis;
    spdlog::set_level(spdlog::level::debug);

    ZenViewer viewer;
    viewer.Show();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ZenViewer::PushCommand(DrawAxesCommand{.axis_len = 10, .axis_size = 1});

    Eigen::Isometry3d axes_pose(Eigen::Translation3d(5, 0, 0));
    axes_pose.rotate(Eigen::AngleAxisd(EIGEN_PI, Eigen::Vector3d(0, 0, 1)));
    DrawAxesCommand dac{.pose = axes_pose.matrix(), .axis_len = 1, .axis_size = 3};
    ZenViewer::PushCommand(dac);
    auto dac_handle = dac.handle;

    ZenViewer::PushCommand(
        SetCameraPoseCommand{.eye = {1, 1, 1}, .target = {7.5, 7.5, 7.5}, .up = {0, 0, 1}});
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::mt19937 rd;
    std::uniform_real_distribution<float> dist(5, 10);
    std::vector<float> points(10'000'000);
    std::generate(points.begin(), points.end(), [&dist, &rd]() { return dist(rd); });
    std::vector colors{1.f, 0.f, 0.f, 1.f};
    DrawPointCommand cmd{.xyzs = std::move(points), .colors = colors, .point_size = 5.f};
    auto handle = cmd.handle;

    spdlog::stopwatch sw;
    ZenViewer::PushCommand(std::move(cmd));
    spdlog::info("Time: {}", sw);

    points.clear();
    points.resize(10'000);
    std::generate(points.begin(), points.end(), [&dist, &rd]() { return dist(rd); });
    Eigen::Isometry3d pose(Eigen::Translation3d(5, 0, 0));
    DrawPointCommand dpc{
        .xyzs = std::move(points), .colors = {0.f, 1.f, 0.f, 1.f}, .pose = pose.matrix()};

    sw.reset();
    ZenViewer::PushCommand(dpc);
    spdlog::info("Time: {}", sw);

    std::this_thread::sleep_for(std::chrono::seconds(5));
    ZenViewer::PushCommand(DeleteNodeCommand{.handle = handle});

    while (!viewer.sm_should_exit) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        pose.rotate(Eigen::AngleAxisd(0.1, Eigen::Vector3d(0, 0, 1)));
        ZenViewer::PushCommand(SetNodePoseCommand{.handle = dac.handle, .pose = pose.matrix()});
        ZenViewer::PushCommand(SetNodePoseCommand{.handle = dpc.handle, .pose = pose.matrix()});
    }

    return 0;
}