#ifndef EVALUATION_CAMERA_POSITION_KDTREE_H_
#define EVALUATION_CAMERA_POSITION_KDTREE_H_

#include <algorithm>
#include <cstdlib>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include <aliases.h>
#include <NoClosePointException.hpp>
#include <utilities.hpp>

namespace cameval {

    class KDTree {
    public:
        KDTree(StringList &database);
        KDTree(AnglePoseList &database);
        ~KDTree();

        AnglePose searchClosestPoint(AnglePose &pose);

    private:
        StringList database;
        PCLKDTree::Ptr tree;
        PCLPointCloud::Ptr cloud;

        void generateTree();
        PCLPoint getPointFromVector(AnglePose &pose);
        void fillPoint(AnglePose &pose, PCLPoint &point);
        AnglePose getVectorFromPoint(PCLPoint &point);
        void fillVector(AnglePose &pose, PCLPoint &point);

    };

    typedef KDTree* KDTReePtr;


} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_KDTREE_BUILDER_H_
