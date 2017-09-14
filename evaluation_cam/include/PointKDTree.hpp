#ifndef EVALUATION_CAMERA_POSITION_POINT_PointKDTree_H_
#define EVALUATION_CAMERA_POSITION_POINT_PointKDTree_H_

#include <algorithm>
#include <cstdlib>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>

#include <aliases.h>
#include <NoClosePointException.hpp>
#include <utilities.hpp>

namespace cameval {

    class PointKDTree {
    public:
        PointKDTree(std::string &pointCloud);
        ~PointKDTree();

        GLMVec3 searchClosestPoint(GLMVec3 &pose);

        GLMVec3List load(std::string &file);

    private:
        std::string pointCloud;
        PCLSimpleKDTree::Ptr tree;
        PCLSimplePointCloud::Ptr cloud;

        void generateTree();
        PCLSimplePoint getPointFromVector(GLMVec3 &pose);
        void fillPoint(GLMVec3 &pose, PCLSimplePoint &point);
        GLMVec3 getVectorFromPoint(PCLSimplePoint &point);
        void fillVector(GLMVec3 &pose, PCLSimplePoint &point);

    };

    typedef PointKDTree* PointKDTreePtr;


} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_PointKDTree_BUILDER_H_
