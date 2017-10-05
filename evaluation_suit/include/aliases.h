#ifndef EVALUATION_CAMERA_POSITION_ALIAS_H_
#define EVALUATION_CAMERA_POSITION_ALIAS_H_

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>


#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>

#include <glm/glm.hpp>

namespace cameval {

    typedef std::vector<std::string> StringList;
    typedef std::unordered_set<std::string> StringUSet;
    typedef std::unordered_map<std::string, int> StringIntUMap;
    typedef std::map<std::string, std::string> StringStringMap;
    typedef std::vector<int> IntList;
    typedef std::vector<float> FloatList;
    typedef std::vector<double> DoubleList;
    typedef std::vector<bool> BoolList;
    typedef std::array<float, 6> Float6Array;
    typedef std::vector<std::string> StringList;
    typedef std::vector<StringList> ArrayStringList;

    typedef std::vector<IntList> IntArrayList;

    typedef std::pair<int, std::string> IntStringPair;
    typedef std::queue<IntStringPair> QueueIntStringPair;

    typedef cv::Mat CVMat;
    typedef cv::Point2f CVPoint2f;
    typedef cv::Point3f CVPoint3f;
    typedef std::vector<CVPoint2f> CVPoint2fList;
    typedef std::vector<cv::Mat> CVMatList;

    typedef pcl::PointXYZ PCLSimplePoint;
    typedef pcl::PointCloud<PCLSimplePoint> PCLSimplePointCloud;
    typedef pcl::KdTreeFLANN<PCLSimplePoint> PCLSimpleKDTree;


    typedef pcl::PointWithViewpoint PCLPoint;
    typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
    typedef pcl::KdTreeFLANN<PCLPoint> PCLKDTree;

    typedef Eigen::Vector3f EigVector3;
    typedef Eigen::Matrix3f EigMatrix3;

    typedef glm::vec2 GLMVec2;
    typedef glm::vec3 GLMVec3;
    typedef glm::vec4 GLMVec4;
    typedef glm::mat3 GLMMat3;
    typedef glm::mat4 GLMMat4;
    
    typedef std::pair<GLMVec3, GLMVec3> AnglePose;
    typedef std::pair<GLMVec3, GLMMat3> Pose;
    typedef std::map<int, glm::vec2> IntGLMVe2Map;
    typedef std::vector<IntGLMVe2Map> IntGLMVe2MapList;

    typedef std::vector<GLMVec2> GLMVec2List;
    typedef std::vector<GLMVec3> GLMVec3List;
    typedef std::vector<GLMMat3> GLMMat3List;
    typedef std::vector<GLMMat4> GLMMat4List;
    typedef std::vector<AnglePose> AnglePoseList;
    typedef std::vector<Pose> PoseList;

    typedef std::map<Float6Array, std::string> FloatArrayStringMap;
    typedef std::map<std::string, Pose> StringPoseMap;
    typedef std::map<Float6Array, Pose> Float6ArrayPoseMap;

    typedef boost::filesystem::path BoostPath;


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

    typedef boost::system::error_code BoostSystemErrorCode;

} // namesapce cameval

#endif // EVALUATION_CAMERA_POSITION_ALIAS_H_
