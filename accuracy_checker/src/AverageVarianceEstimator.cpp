
#include <meshac/AverageVarianceEstimator.hpp>

namespace meshac {

    AverageVarianceEstimator::AverageVarianceEstimator(PointAccuracyModelPtr accuracyModel, GLMVec3List &points)
                                                             : Point3DVarianceEstimator(accuracyModel, points)
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

    double AverageVarianceEstimator::computeVarianceFromMatrix(EigMatrix &varianceMatrix)
    {
        double sum = varianceMatrix.sum();
        return sum / varianceMatrix.size();
    }


} // namespace meshac


