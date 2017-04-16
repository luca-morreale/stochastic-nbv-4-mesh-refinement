
#include <meshac/DeterminantVarianceEstimator.hpp>

namespace meshac {

    DeterminantVarianceEstimator::DeterminantVarianceEstimator(AccuracyModelPtr accuracyModel, GLMListVec3 &points) : Point3DVarianceEstimator(accuracyModel, points)
    { /*    */ }
    
    DeterminantVarianceEstimator::~DeterminantVarianceEstimator()
    { /*    */ }

    EigMatrix DeterminantVarianceEstimator::selectVarianceMatrix(EigMatrixList &mat) 
    { 
        EigMatrix worst = mat[0];
        double worstDet = this->computeVarianceFromMatrix(worst);
        for (EigMatrix matrix : mat) { 
            double det = this->computeVarianceFromMatrix(matrix);
            if (det > worstDet) { 
                worst = matrix; 
                worstDet = det;
            } 
        }        
        return worst;
    } 

    double DeterminantVarianceEstimator::computeVarianceFromMatrix(EigMatrix &varianceMatrix)
    {
        return varianceMatrix.determinant();
    }

} // namespace meshac
