
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

    EigMatrix Point3DVarianceEstimator::computeVariaceForPoint(GLMVec3 &point)
    {
        GLMListVec3 points = this->get3DPoints();

        EigMatrixList variances;
        
        for (int i = 0; i<points.size(); i++) {
            if (glm::epsilonEqual(points[i], point, EPSILON)[0]) {
                variances = this->getAccuracyModel()->getAccuracyForPoint(i);
            }
        }

        return selectVarianceMatrix(variances);
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
