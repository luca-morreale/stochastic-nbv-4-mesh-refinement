
#include <meshac/DeterminantVarianceEstimator.hpp>

namespace meshac {

    DeterminantVarianceEstimator::DeterminantVarianceEstimator(AccuracyModelPtr accuracyModel, GLMListVec3 &points) : Point3DVarianceEstimator(accuracyModel, points)
    { /*    */ }
    
    DeterminantVarianceEstimator::~DeterminantVarianceEstimator()
    { /*    */ }

    double DeterminantVarianceEstimator::computeVarianceFromMatrix(EigMatrix &varianceMatrix)
    {
        return varianceMatrix.determinant();
    }

} // namespace meshac
