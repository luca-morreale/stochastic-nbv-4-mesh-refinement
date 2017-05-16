
#include <meshac/WorstEigenvalueVarianceEstimator.hpp>

namespace meshac {

    WorstEigenvalueVarianceEstimator::WorstEigenvalueVarianceEstimator(PointAccuracyModelPtr accuracyModel, GLMVec3List &points)
                                                             : Point3DVarianceEstimator(accuracyModel, points)
    { /*    */ }
    
    WorstEigenvalueVarianceEstimator::~WorstEigenvalueVarianceEstimator()
    { /*    */ }

    EigMatrix WorstEigenvalueVarianceEstimator::selectVarianceMatrix(EigMatrixList &mat) 
    { 
        EigMatrix worst = mat[0];
        double worstEig = this->computeVarianceFromMatrix(worst);
        for (EigMatrix matrix : mat) { 
            double eig = this->computeVarianceFromMatrix(matrix);
            if (eig > worstEig) { 
                worst = matrix; 
                worstEig = eig;
            } 
        }
        return worst;
    } 

    double WorstEigenvalueVarianceEstimator::computeVarianceFromMatrix(EigMatrix &varianceMatrix)
    {
        EigVector eivals = varianceMatrix.eigenvalues().real();
        return eivals.maxCoeff();

    }

} // namespace meshac
