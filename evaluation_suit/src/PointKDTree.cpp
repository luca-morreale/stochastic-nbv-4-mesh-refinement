#include <PointKDTree.hpp>

namespace cameval {

    PointKDTree::PointKDTree(std::string &pointCloud)
    {
        this->pointCloud = pointCloud;
        tree = PCLSimpleKDTree().makeShared();
        cloud = PCLSimplePointCloud().makeShared();
        this->generateTree();
    }

    PointKDTree::~PointKDTree()
    {
        cloud.reset();
        tree.reset();
    }

    void PointKDTree::generateTree()
    {
        GLMVec3List database = load(pointCloud);
        cloud->width = database.size();
        cloud->height = 1;
        cloud->points.resize(cloud->width * cloud->height);

        #pragma omp parallel for
        for (int i = 0; i < database.size(); i++) {
            fillPoint(database[i], cloud->points[i]);
        }

        tree->setInputCloud(cloud);
    }

    PCLSimplePoint PointKDTree::getPointFromVector(GLMVec3 &pose)
    {
        PCLSimplePoint point;
        fillPoint(pose, point);
        return point;
    }

    void PointKDTree::fillPoint(GLMVec3 &pose, PCLSimplePoint &point)
    {
        point.x = pose.x;
        point.y = pose.y;
        point.z = pose.z;
    }

    GLMVec3 PointKDTree::getVectorFromPoint(PCLSimplePoint &point)
    {
        GLMVec3 pose;
        fillVector(pose, point);
        return pose;
    }

    void PointKDTree::fillVector(GLMVec3 &pose, PCLSimplePoint &point)
    {
        pose.x = point.x;
        pose.y = point.y;
        pose.z = point.z;
    }

    GLMVec3 PointKDTree::searchClosestPoint(GLMVec3 &pose)
    {
        PCLSimplePoint searchPoint = getPointFromVector(pose);
        // K nearest neighbor search
        int K = 1;

        IntList pointIdxNKNSearch(K);
        FloatList pointNKNSquaredDistance(K);

        if (tree->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            return getVectorFromPoint(cloud->points[pointIdxNKNSearch[0]]);
        }
        throw NoClosePointException();
    }

    GLMVec3List PointKDTree::load(std::string &file)
    {
        GLMVec3List list;
        std::ifstream ply(file);
        
        while(!ply.eof()) {
            GLMVec3 point;
            ply >> point.x >> point.y >> point.z;

            if (point.x == 0.0f && point.y == 0.0f && point.z == 0.0f) continue;

            list.push_back(point);
        }
        ply.close();
        return list;
    }

}
