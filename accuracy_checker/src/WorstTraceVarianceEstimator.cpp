
#include <meshac/WorstTraceVarianceEstimator.hpp>

namespace meshac {

    WorstTraceVarianceEstimator::WorstTraceVarianceEstimator(AccuracyModelPtr accuracyModel, GLMListVec3 &points) : Point3DVarianceEstimator(accuracyModel, points)
    { /*    */ }
    
    WorstTraceVarianceEstimator::~WorstTraceVarianceEstimator()
    { /*    */ }

    double WorstTraceVarianceEstimator::computeVarianceFromMatrix(EigMatrix &varianceMatrix)
    {
        return varianceMatrix.trace();
    }


} // namespace meshac
