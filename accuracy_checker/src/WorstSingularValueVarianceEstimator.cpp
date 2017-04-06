
#include <meshac/WorstSingularValueVarianceEstimator.hpp>

namespace meshac {

    WorstSingularValueVarianceEstimator::WorstSingularValueVarianceEstimator(AccuracyModelPtr accuracyModel, GLMListVec3 &points) : Point3DVarianceEstimator(accuracyModel, points)
    { /*    */ }
    
    WorstSingularValueVarianceEstimator::~WorstSingularValueVarianceEstimator()
    { /*    */ }

    double WorstSingularValueVarianceEstimator::computeVarianceFromMatrix(EigMatrix &varianceMatrix)
    {
        EigSVD svd(varianceMatrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
        return svd.singularValues().maxCoeff();
    }

} // namespace meshac
