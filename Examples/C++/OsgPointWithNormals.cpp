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

#include <ranges>

Eigen::MatrixXf ComputeColorFromNormal(Eigen::MatrixXf& normals, const Eigen::Vector3f& light_dir) {
    Eigen::MatrixXf colors(4, normals.cols());
    colors.row(3).fill(1);
    for (int i = 0; i < normals.cols(); ++i) {
        normals.col(i).normalize();
        // colors.block<3, 1>(0, i) = normals.col(i).array() * light_dir.array();
        colors.block<3, 1>(0, i) = normals.col(i);
    }
    return colors;
}

osg::Drawable* DrawAxes(float axis_len = 1.f, float axis_size = 1.f) {
    auto geo = new osg::Geometry();
    auto v = new osg::Vec3Array(6);
    (*v)[0] = osg::Vec3f(0.0f, 0.0f, 0.0f);
    (*v)[1] = osg::Vec3f(axis_len, 0.0f, 0.0f);
    (*v)[2] = osg::Vec3f(0.0f, 0.0f, 0.0f);
    (*v)[3] = osg::Vec3f(0.0f, axis_len, 0.0f);
    (*v)[4] = osg::Vec3f(0.0f, 0.0f, 0.0f);
    (*v)[5] = osg::Vec3f(0.0f, 0.0f, axis_len);

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
    geo->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(axis_size));
    geo->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    return geo;
}

int main() {
    osgViewer::Viewer viewer;
    viewer.apply(new osgViewer::SingleWindow(30, 30, 800, 800, 0));
    osg::ref_ptr root = new osg::Group;
    osg::ref_ptr light_model = new osg::LightModel;
    light_model->setTwoSided(true);
    root->getOrCreateStateSet()->setAttribute(light_model);

    osg::ref_ptr light = new osg::Light;
    light->setLightNum(0);
    light->setPosition(osg::Vec4(1.f, 1.f, 1.f, 0.f));
    light->setDirection(osg::Vec3(0.f, 1.f, 0.f));
    light->setDiffuse(osg::Vec4(1.f, 1.f, 1.f, 1.f));
    light->setConstantAttenuation(1.f);
    light->setLinearAttenuation(0.f);
    light->setQuadraticAttenuation(0.f);

    osg::ref_ptr light_source = new osg::LightSource;
    light_source->setLight(light);
    // root->addChild(light_source);

    auto geode = new osg::Geode;
    root->addChild(geode);

    geode->addDrawable(DrawAxes(1.f, 3.f));

    geode->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(2.f));
    auto material = new osg::Material;
    material->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
    material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(1.f, 0.f, 0.f, 1.f));
    material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1.f, 0.f, 0.f, 1.f));
    material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(1.f, 0.f, 0.f, 1.f));
    // geode->getOrCreateStateSet()->setAttributeAndModes(material);

    const int n = 1'000'000;
    std::vector<Eigen::MatrixXf> points_set, normals_set, colors_set;
    {
        Eigen::MatrixXf points(3, n);  // column major
        points.setZero();
        points.topRows(2).setRandom();
        points_set.push_back(points);

        Eigen::MatrixXf normals(3, n);
        normals.setZero();
        normals.row(2).fill(1);
        normals_set.push_back(normals);
        colors_set.push_back(ComputeColorFromNormal(normals, Eigen::Vector3f::UnitX()));
    }

    {
        Eigen::MatrixXf points(3, n);  // column major
        points.setZero();
        points.bottomRows(2).setRandom();
        points_set.push_back(points);

        Eigen::MatrixXf normals(3, n);
        normals.setRandom();
        normals.row(0) = normals.row(0).cwiseAbs();
        // normalize each column
        for (int i = 0; i < normals.cols(); ++i) {
            normals.col(i).normalize();
        }

        // normals.setZero();
        // normals.row(0).fill(1);
        normals_set.push_back(normals);
        colors_set.push_back(ComputeColorFromNormal(normals, Eigen::Vector3f::UnitX()));
    }

    for (int i = 0; i < points_set.size(); ++i) {
        auto& points = points_set[i];
        auto& normals = normals_set[i];
        auto& colors = colors_set[i];
        auto geo = new osg::Geometry;
        geo->setVertexArray(
            new osg::Vec3Array(points.cols(), reinterpret_cast<osg::Vec3*>(points.data())));
        geo->setNormalArray(
            new osg::Vec3Array(normals.cols(), reinterpret_cast<osg::Vec3*>(normals.data())));
        geo->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
        geo->setColorArray(
            new osg::Vec4Array(colors.cols(), reinterpret_cast<osg::Vec4*>(colors.data())));
        geo->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

        geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, points.cols()));

        geode->addDrawable(geo);
    }

    auto sphere = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.f, 0.f, 0.f), 1.f));
    // geode->addDrawable(sphere);

    viewer.setSceneData(root);
    viewer.setCameraManipulator(new osgGA::MultiTouchTrackballManipulator);
    viewer.getCamera()->setClearColor(osg::Vec4(1.f, 1.f, 1.f, 1.f));
    viewer.home();

    return viewer.run();
}