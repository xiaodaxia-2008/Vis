#include <ZenVis.h>
#include <spdlog/spdlog.h>
#include <random>

int main() {
    using namespace zenvis;
    spdlog::set_level(spdlog::level::debug);

    ZenViewer viewer;
    viewer.Show();

    std::this_thread::sleep_for(std::chrono::seconds(5));
    ZenViewer::sm_cmds.push(DrawAxesCommand{.axis_len = 1, .axis_size = 1});
    ZenViewer::sm_cmds.push(
        SetCameraPoseCommand{.eye = {1, 1, 1}, .target = {0, 0, 0}, .up = {0, 0, 1}});
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::vector<float> points, colors{1.f, 0.f, 0.f, 1.f};
    std::mt19937 rd;
    std::uniform_real_distribution<float> dist(0, 10);
    for (int i = 0; i < 10'000'000; i++) {
        points.push_back(dist(rd));
        points.push_back(dist(rd));
        points.push_back(dist(rd));
    }
    ZenViewer::sm_cmds.push(DrawPointCommand{.xyzs = points, .colors = colors, .point_size = 5.f});
    std::this_thread::sleep_for(std::chrono::seconds(30));

    return 0;
}