#ifndef MESH_ACCURACY_WORST_SINGULAR_VALUE_VARIANCE_ESTIMATOR_H
#define MESH_ACCURACY_WORST_SINGULAR_VALUE_VARIANCE_ESTIMATOR_H

#include <meshac/alias_definition.hpp>
#include <meshac/Point3DVarianceEstimator.hpp>


namespace meshac {
    

    class WorstSingularValueVarianceEstimator : public Point3DVarianceEstimator {
    public:
        WorstSingularValueVarianceEstimator(AccuracyModelPtr accuracyModel, GLMVec3List &points);
        ~WorstSingularValueVarianceEstimator();

    protected:
        virtual EigMatrix selectVarianceMatrix(EigMatrixList &mat); 
        /*
         * Computes the variance as the maximum singular value of the matrix.
         */
        double computeVarianceFromMatrix(EigMatrix &varianceMatrix);

    };

} // namespace meshac

#endif // MESH_ACCURACY_WORST_SINGULAR_VALUE_VARIANCE_ESTIMATOR_H
