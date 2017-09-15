#include <ReportGenerator.hpp>

ReportGenerator::ReportGenerator(meshac::Point3DVarianceEstimatorPtr estimator, std::vector<glm::vec3> points)
{
    this->estimator = estimator;
    this->points = points;
    this->normals.assign(points.size(), glm::vec3());
    computeNormals();
}

ReportGenerator::~ReportGenerator()
{
    points.clear();
    normals.clear();
}

void ReportGenerator::generateReport(std::string outputFile)
{
    std::ofstream out(outputFile);

    #pragma omp parallel for
    for (int i = 0; i < points.size(); i++) {
        std::cout << i << "/" << points.size() << std::endl;
        double acc = estimator->computeSingleVarianceForPoint(points[i]);

        #pragma omp critical
        {
            out << points[i].x << " " << points[i].y << " " << points[i].z << " ";
            out << normals[i].x << " " << normals[i].y << " " << normals[i].z << " ";
            out << acc << std::endl;
        }
    }
    out.close();
}

void ReportGenerator::computeNormals() 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = points.size();
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    #pragma omp parallel for
    for (int i = 0; i < points.size(); i++) {
        fillPoint(cloud->points[i], points[i]);
    }

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Compute the features
    ne.compute(*cloud_normals);

    #pragma omp parallel for
    for (int i = 0; i < points.size(); i++) {
        fillPoint(normals[i], cloud->points[i]);
    }
}

void ReportGenerator::fillPoint(pcl::PointXYZ &point, glm::vec3 &pose)
{
    point.x = pose.x;
    point.y = pose.y;
    point.z = pose.z;
}

void ReportGenerator::fillPoint(glm::vec3 &pose, pcl::PointXYZ &point)
{
    pose.x = point.x;
    pose.y = point.y;
    pose.z = point.z;
}
