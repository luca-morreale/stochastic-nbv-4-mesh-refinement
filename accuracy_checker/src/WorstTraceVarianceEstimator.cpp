
#include <meshac/WorstTraceVarianceEstimator.hpp>

namespace meshac {

    WorstTraceVarianceEstimator::WorstTraceVarianceEstimator(AccuracyModelPtr accuracyModel, GLMVec3List &points) : Point3DVarianceEstimator(accuracyModel, points)
    { /*    */ }
    
    WorstTraceVarianceEstimator::~WorstTraceVarianceEstimator()
    { /*    */ }

    EigMatrix WorstTraceVarianceEstimator::selectVarianceMatrix(EigMatrixList &mat) 
    { 
        EigMatrix worst = mat[0];
        double maxTrace = this->computeVarianceFromMatrix(worst);
        for (EigMatrix matrix : mat) { 
            double trace = this->computeVarianceFromMatrix(matrix);
            if (trace > maxTrace) { 
                worst = matrix; 
                maxTrace = trace;
            } 
        }        
        return worst;
    } 

    double WorstTraceVarianceEstimator::computeVarianceFromMatrix(EigMatrix &varianceMatrix)
    {
        return varianceMatrix.trace();
    }


} // namespace meshac
