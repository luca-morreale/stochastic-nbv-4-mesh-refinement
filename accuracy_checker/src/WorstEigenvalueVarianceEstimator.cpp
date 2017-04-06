
#include <meshac/WorstEigenvalueVarianceEstimator.hpp>

namespace meshac {

    WorstEigenvalueVarianceEstimator::WorstEigenvalueVarianceEstimator(AccuracyModelPtr accuracyModel, GLMListVec3 &points) : Point3DVarianceEstimator(accuracyModel, points)
    { /*    */ }
    
    WorstEigenvalueVarianceEstimator::~WorstEigenvalueVarianceEstimator()
    { /*    */ }

    double WorstEigenvalueVarianceEstimator::computeVarianceFromMatrix(EigMatrix &varianceMatrix)
    {
        EigVector eivals = varianceMatrix.eigenvalues().real();
        return eivals.maxCoeff();

    }

} // namespace meshac
