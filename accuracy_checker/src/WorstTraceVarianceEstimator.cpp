
#include <meshac/WorstTraceVarianceEstimator.hpp>

namespace meshac {

    WorstTraceVarianceEstimator::WorstTraceVarianceEstimator(AccuracyModelPtr accuracyModel, GLMListVec3 points) : Point3DVarianceEstimator(accuracyModel, points)
    { /*    */ }
    
    WorstTraceVarianceEstimator::~WorstTraceVarianceEstimator()
    { /*    */ }

    EigMatrix WorstTraceVarianceEstimator::selectVarianceMatrix(EigMatrixList &mat)
    {
        EigMatrix worst = mat[0];
        for (EigMatrix matrix : mat) {
            if (matrix.trace() > worst.trace()) {
                worst = matrix;
            }
        }
        
        return worst;
    }


} // namespace meshac
