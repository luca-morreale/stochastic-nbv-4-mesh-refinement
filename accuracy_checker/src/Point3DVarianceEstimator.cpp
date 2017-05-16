
#include <meshac/Point3DVarianceEstimator.hpp>

namespace meshac {

    Point3DVarianceEstimator::Point3DVarianceEstimator(PointAccuracyModelPtr accuracyModel, GLMVec3List &points)
    {
        this->accuracyModel = accuracyModel;
        this->points = points;
    }

    Point3DVarianceEstimator::~Point3DVarianceEstimator()
    {
        delete accuracyModel;
        points.clear();
    }

    EigMatrix Point3DVarianceEstimator::computeVariaceMatrixForPoint(GLMVec3 &point) 
    { 
        GLMVec3List points = this->get3DPoints(); 
        EigMatrixList variances; 
         
        for (int i = 0; i<points.size(); i++) {
            if (glm::epsilonEqual(points[i], point, EPSILON)[0]) {
                return this->computeVariaceMatrixForPoint(i);
            }
        }
        return EigMatrix(); 
    }

    EigMatrix Point3DVarianceEstimator::computeVariaceMatrixForPoint(int pointIndex)
    {
        EigMatrixList variances = this->getAccuracyModel()->getAccuracyForPoint(pointIndex);
        return selectVarianceMatrix(variances);
    }    

    double Point3DVarianceEstimator::computeTotalVarianceForPoint(GLMVec3 &point)
    {
        GLMVec3List points = this->get3DPoints();

        for (int i = 0; i<points.size(); i++) {
            if (glm::all(glm::epsilonEqual(points[i], point, EPSILON))) {
                return this->computeTotalVarianceForPoint(i);
            }
        }
        return -1.0;
    }

    double Point3DVarianceEstimator::computeTotalVarianceForPoint(int pointIndex)
    {
        EigMatrix variance = this->getAccuracyModel()->getCompleteAccuracyForPoint(pointIndex);
        return this->computeVarianceFromMatrix(variance);
    }

    double Point3DVarianceEstimator::computeSingleVarianceForPoint(GLMVec3 &point)
    {
        GLMVec3List points = this->get3DPoints();

        for (int i = 0; i<points.size(); i++) {
            if (glm::all(glm::epsilonEqual(points[i], point, EPSILON))) {
                return this->computeSingleVarianceForPoint(i);
            }
        }
        return -1.0;
    }

    double Point3DVarianceEstimator::computeSingleVarianceForPoint(int pointIndex)
    {
        EigMatrixList varianceList = this->getAccuracyModel()->getAccuracyForPoint(pointIndex);
        EigMatrix variance = this->selectVarianceMatrix(varianceList);
        std::cout << "variance matrix " << variance << std::endl;
        return this->computeVarianceFromMatrix(variance);
    }


    void Point3DVarianceEstimator::setAccuracyModel(PointAccuracyModelPtr accuracyModel)
    {
        this->accuracyModel = accuracyModel;
    }

    PointAccuracyModelPtr Point3DVarianceEstimator::getAccuracyModel()
    {
        return this->accuracyModel;
    }

    GLMVec3List Point3DVarianceEstimator::get3DPoints()
    {
        return this->points;
    }
    
    void Point3DVarianceEstimator::set3DPoints(GLMVec3List &points)
    {
        this->points = points;
    }

    void Point3DVarianceEstimator::append3DPoints(GLMVec3List &newPoints)
    {
        this->points.insert(this->points.end(), newPoints.begin(), newPoints.end());
    }

    void Point3DVarianceEstimator::append3DPoints(GLMVec3 &newPoint)
    {
        this->points.push_back(newPoint);
    }


} // namespace meshac
