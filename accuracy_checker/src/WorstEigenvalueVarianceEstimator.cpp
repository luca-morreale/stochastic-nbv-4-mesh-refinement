
#include <WorstEigenvalueVarianceEstimator.hpp>

namespace meshac {

    WorstEigenvalueVarianceEstimator::WorstEigenvalueVarianceEstimator(AccuracyModelPtr accuracyModel, GLMListVec3 points) : Point3DVarianceEstimator(accuracyModel, points)
    { /*    */ }
    
    WorstEigenvalueVarianceEstimator::~WorstEigenvalueVarianceEstimator()
    { /*    */ }

    EigMatrix WorstEigenvalueVarianceEstimator::selectVarianceMatrix(EigMatrixList &mat)
    {
        EigMatrix worst = mat[0];
        for (EigMatrix matrix : mat) {
            if (matrix(matrix.rows(), matrix.cols()) > worst(worst.rows(), worst.cols())) {
                worst = matrix;
            }
        }
        
        return worst;
    }

} // namespace meshac
