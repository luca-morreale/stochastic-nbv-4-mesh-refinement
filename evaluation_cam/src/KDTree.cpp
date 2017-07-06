#include <KDTree.hpp>

namespace cameval {
    
    KDTree::KDTree(StringList &database)
    {
        this->database = database;
        tree = PCLKDTree().makeShared();
        cloud = PCLPointCloud().makeShared();
        this->generateTree();
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
            EigVector6 pose = parseEntry(database[i]);
            fillPoint(pose, cloud->points[i]);
        }

        tree->setInputCloud(cloud);
    }

    PCLPoint KDTree::getPointFromVector(EigVector6 &pose)
    {
        PCLPoint point;
        fillPoint(pose, point);
        return point;
    }

    void KDTree::fillPoint(EigVector6 &pose, PCLPoint &point)
    {
        point.x = pose[0];
        point.y = pose[1];
        point.z = pose[2];
        point.vp_x = pose[3];
        point.vp_y = pose[4];
        point.vp_z = pose[5];
    }

    EigVector6 KDTree::getVectorFromPoint(PCLPoint &point)
    {
        EigVector6 pose;
        fillVector(pose, point);
        return pose;
    }

    void KDTree::fillVector(EigVector6 &pose, PCLPoint &point)
    {
        pose[0] = point.x;
        pose[1] = point.y;
        pose[2] = point.z;
        pose[3] = point.vp_x;
        pose[4] = point.vp_y;
        pose[5] = point.vp_z;
    }

    EigVector6 KDTree::searchClosestPoint(EigVector6 &pose)
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

    EigVector6 KDTree::parseEntry(std::string &entry)
    {
        EigVector6 pose;

        entry = entry.substr(0, entry.find_last_of("."));

        StringList blocks;
        std::size_t offset = 0;
        boost::split(blocks, entry, boost::is_any_of("_"));
        
        for (int i = 0; i < 6; i++) {
            pose[i] = std::strtod(blocks[i + 1].c_str(), NULL);
        }
        
        return pose;
    }

}
