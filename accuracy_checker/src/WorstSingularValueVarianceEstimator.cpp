
#include <meshac/WorstSingularValueVarianceEstimator.hpp>

namespace meshac {

    WorstSingularValueVarianceEstimator::WorstSingularValueVarianceEstimator(AccuracyModelPtr accuracyModel, GLMVec3List &points) : Point3DVarianceEstimator(accuracyModel, points)
    { /*    */ }
    
    WorstSingularValueVarianceEstimator::~WorstSingularValueVarianceEstimator()
    { /*    */ }

    EigMatrix WorstSingularValueVarianceEstimator::selectVarianceMatrix(EigMatrixList &mat) 
    { 
        EigMatrix worst = mat[0];
        double worstSV = this->computeVarianceFromMatrix(worst);
        for (EigMatrix matrix : mat) { 
            double sv = this->computeVarianceFromMatrix(matrix);
            if (sv > worstSV) { 
                worst = matrix; 
                worstSV = sv;
            } 
        }        
        return worst;
    }

    double WorstSingularValueVarianceEstimator::computeVarianceFromMatrix(EigMatrix &varianceMatrix)
    {
        EigSVD svd(varianceMatrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
        return svd.singularValues().maxCoeff();
    }

} // namespace meshac
