#ifndef MESH_ACCURACY_WORST_EIGENVALUE_VARIANCE_ESTIMATOR_H
#define MESH_ACCURACY_WORST_EIGENVALUE_VARIANCE_ESTIMATOR_H

#include <meshac/alias_definition.hpp>
#include <meshac/Point3DVarianceEstimator.hpp>


namespace meshac {
    

    class WorstEigenvalueVarianceEstimator : public Point3DVarianceEstimator {
    public:
        WorstEigenvalueVarianceEstimator(AccuracyModelPtr accuracyModel, GLMVec3List &points);
        virtual ~WorstEigenvalueVarianceEstimator();

    protected:
        virtual EigMatrix selectVarianceMatrix(EigMatrixList &mat); 
        /*
         * Computes the variance as the biggest eigenvalue of the matrix.
         */
        double computeVarianceFromMatrix(EigMatrix &varianceMatrix);

    };

} // namespace meshac

#endif // MESH_ACCURACY_WORST_VARIANCE_ESTIMATOR_H
