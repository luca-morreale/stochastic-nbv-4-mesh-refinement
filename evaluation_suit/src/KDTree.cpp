#include <KDTree.hpp>

namespace cameval {
    
    KDTree::KDTree(StringList &database)
    {
        this->database = database;
        tree = PCLKDTree().makeShared();
        cloud = PCLPointCloud().makeShared();
        this->generateTree();
    }

    KDTree::KDTree(AnglePoseList &database)
    {
        tree = PCLKDTree().makeShared();
        cloud = PCLPointCloud().makeShared();

        cloud->width = database.size();
        cloud->height = 1;
        cloud->points.resize(cloud->width * cloud->height);

        for (int i = 0; i < database.size(); i++) {
            fillPoint(database[i], cloud->points[i]);
        }

        tree->setInputCloud(cloud);
    }

    KDTree::~KDTree()
    {
        cloud.reset();
        tree.reset();
    }

    void KDTree::generateTree()
    {
        cloud->width = database.size();
        cloud->height = 1;
        cloud->points.resize(cloud->width * cloud->height);

        #pragma omp parallel for
        for (int i = 0; i < database.size(); i++) {
            AnglePose pose = parseEntry(database[i]);
            fillPoint(pose, cloud->points[i]);
        }

        tree->setInputCloud(cloud);
    }

    PCLPoint KDTree::getPointFromVector(AnglePose &pose)
    {
        PCLPoint point;
        fillPoint(pose, point);
        return point;
    }

    void KDTree::fillPoint(AnglePose &pose, PCLPoint &point)
    {
        point.x = pose.first.x;
        point.y = pose.first.y;
        point.z = pose.first.z;
        point.vp_x = pose.second.x;
        point.vp_y = pose.second.y;
        point.vp_z = pose.second.z;
    }

    AnglePose KDTree::getVectorFromPoint(PCLPoint &point)
    {
        AnglePose pose;
        fillVector(pose, point);
        return pose;
    }

    void KDTree::fillVector(AnglePose &pose, PCLPoint &point)
    {
        pose.first.x = point.x;
        pose.first.y = point.y;
        pose.first.z = point.z;
        pose.second.x = point.vp_x;
        pose.second.y = point.vp_y;
        pose.second.z = point.vp_z;
    }

    AnglePose KDTree::searchClosestPoint(AnglePose &pose)
    {
        PCLPoint searchPoint = getPointFromVector(pose);
        // K nearest neighbor search
        int K = 1;

        IntList pointIdxNKNSearch(K);
        FloatList pointNKNSquaredDistance(K);

        if (tree->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            return getVectorFromPoint(cloud->points[pointIdxNKNSearch[0]]);
        }
        throw NoClosePointException();
    }

}
