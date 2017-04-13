
#include <meshac/Point3DVarianceEstimator.hpp>

namespace meshac {

    Point3DVarianceEstimator::Point3DVarianceEstimator(AccuracyModelPtr accuracyModel, GLMListVec3 &points)
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
        GLMListVec3 points = this->get3DPoints(); 
        EigMatrixList variances; 
         
        for (int i = 0; i<points.size(); i++) {
            if (glm::epsilonEqual(points[i], point, EPSILON)[0]) {
                return this->getVariaceMatrixForPoint(i);
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
        GLMListVec3 points = this->get3DPoints();

        for (int i = 0; i<points.size(); i++) {
            if (glm::all(glm::epsilonEqual(points[i], point, EPSILON))) {
                return this->computeVarianceForPoint(i);
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
        GLMListVec3 points = this->get3DPoints();

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
        return this->computeVarianceFromMatrix(variance);
    }


    void Point3DVarianceEstimator::setAccuracyModel(AccuracyModelPtr accuracyModel)
    {
        this->accuracyModel = accuracyModel;
    }

    AccuracyModelPtr Point3DVarianceEstimator::getAccuracyModel()
    {
        return this->accuracyModel;
    }

    GLMListVec3 Point3DVarianceEstimator::get3DPoints()
    {
        return this->points;
    }
    
    void Point3DVarianceEstimator::set3DPoints(GLMListVec3 &points)
    {
        this->points = points;
    }

    void Point3DVarianceEstimator::append3DPoints(GLMListVec3 &newPoints)
    {
        this->points.insert(this->points.end(), newPoints.begin(), newPoints.end());
    }

    void Point3DVarianceEstimator::append3DPoints(GLMVec3 &newPoint)
    {
        this->points.push_back(newPoint);
    }


} // namespace meshac
