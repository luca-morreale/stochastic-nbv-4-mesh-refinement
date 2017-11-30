#ifndef REPORT_GENERATOR_H_
#define REPORT_GENERATOR_H_

#include <glm/glm.hpp>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <meshac/Point3DVarianceEstimator.hpp>

#include <aliases.hpp>

class ReportGenerator {
public:
    ReportGenerator(meshac::Point3DVarianceEstimatorPtr estimator, std::vector<glm::vec3> points);
    ~ReportGenerator();

    void generateReport(std::string outputFile);

protected:
    void computeNormals();
    void fillPoint(pcl::PointXYZ &point, glm::vec3 &pose);
    void fillPoint(glm::vec3 &pose, pcl::PointXYZ &point);


private:
    meshac::Point3DVarianceEstimatorPtr estimator;
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> normals;

};



#endif // REPORT_GENERATOR_H_
