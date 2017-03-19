
#include <meshac/AverageVarianceEstimator.hpp>

namespace meshac {

    AverageVarianceEstimator::AverageVarianceEstimator(AccuracyModelPtr accuracyModel, GLMListVec3 points) : Point3DVarianceEstimator(accuracyModel, points)
    { /*    */ }
    
    AverageVarianceEstimator::~AverageVarianceEstimator()
    { /*    */ }

    EigMatrix AverageVarianceEstimator::selectVarianceMatrix(EigMatrixList &mat)
    {
        EigMatrix variance = EigZeros(mat[0].rows());
        for (EigMatrix matrix : mat) {
            variance += matrix;
        }

        return variance / mat.size();
    }


} // namespace meshac


