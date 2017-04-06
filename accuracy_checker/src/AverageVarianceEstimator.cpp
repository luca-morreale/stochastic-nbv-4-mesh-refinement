
#include <meshac/AverageVarianceEstimator.hpp>

namespace meshac {

    AverageVarianceEstimator::AverageVarianceEstimator(AccuracyModelPtr accuracyModel, GLMListVec3 &points) : Point3DVarianceEstimator(accuracyModel, points)
    { /*    */ }
    
    AverageVarianceEstimator::~AverageVarianceEstimator()
    { /*    */ }

    double AverageVarianceEstimator::computeVarianceFromMatrix(EigMatrix &varianceMatrix)
    {
        double sum = varianceMatrix.sum();
        return sum / varianceMatrix.size();
    }


} // namespace meshac


