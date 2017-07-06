#ifndef EVALUATION_CAMERA_POSITION_ALIAS_H_
#define EVALUATION_CAMERA_POSITION_ALIAS_H_

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>

#include <glm/glm.hpp>

#include <Eigen/Dense>

namespace cameval {

    typedef std::vector<std::string> StringList;
    typedef std::vector<int> IntList;
    typedef std::vector<float> FloatList;
    typedef std::vector<double> DoubleList;


    typedef pcl::PointWithViewpoint PCLPoint;
    typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
    typedef pcl::KdTreeFLANN<PCLPoint> PCLKDTree;


    typedef Eigen::Matrix<float, 6, 1> EigVector6;
    typedef std::vector<EigVector6> PoseList;


    typedef Eigen::Vector3f EigVector3;
    typedef Eigen::Matrix3f EigMatrix3;


    typedef glm::vec3 GLMVec3;
    typedef glm::mat3 GLMMat3;


    typedef rapidjson::SizeType JsonSizeT;
    const auto JsonArray = rapidjson::kArrayType;
    const auto JsonObject = rapidjson::kObjectType;
    const auto JsonNumber = rapidjson::kNumberType;

    typedef rapidjson::Document JsonDoc;
    typedef rapidjson::Value JsonValue;
    typedef rapidjson::IStreamWrapper JsonFINWrapper;
    typedef rapidjson::OStreamWrapper JsonFOUTWrapper;
    typedef rapidjson::Writer<JsonFOUTWrapper> JsonWriter;
    typedef rapidjson::PrettyWriter<JsonFOUTWrapper> JsonPrettyWriter;


} // namesapce cameval

#endif // EVALUATION_CAMERA_POSITION_ALIAS_H_
