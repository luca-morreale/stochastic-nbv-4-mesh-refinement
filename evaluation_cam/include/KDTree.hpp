#ifndef EVALUATION_CAMERA_POSITION_KDTREE_H_
#define EVALUATION_CAMERA_POSITION_KDTREE_H_

#include <vector>
#include <string>
#include <algorithm>
#include <cstdlib>

#include <boost/algorithm/string.hpp>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <NoClosePointException.hpp>
#include <aliases.h>

namespace cameval {

    class KDTree {
    public:
        KDTree(StringList &database);
        ~KDTree();

        EigVector6 searchClosestPoint(EigVector6 &pose);

    private:
        StringList database;
        PCLKDTree::Ptr tree;
        PCLPointCloud::Ptr cloud;

        void generateTree();
        PCLPoint getPointFromVector(EigVector6 &pose);
        void fillPoint(EigVector6 &pose, PCLPoint &point);
        EigVector6 getVectorFromPoint(PCLPoint &point);
        void fillVector(EigVector6 &pose, PCLPoint &point);
        EigVector6 parseEntry(std::string &entry);

    };

    typedef KDTree* KDTReePtr;


} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_KDTREE_BUILDER_H_
