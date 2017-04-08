#ifndef MESH_ACCURACY_AVERAGE_VARIANCE_ESTIMATOR_H
#define MESH_ACCURACY_AVERAGE_VARIANCE_ESTIMATOR_H

#include <meshac/alias_definition.hpp>
#include <meshac/Point3DVarianceEstimator.hpp>


namespace meshac {


    class AverageVarianceEstimator : public Point3DVarianceEstimator {
    public:
        AverageVarianceEstimator(AccuracyModelPtr accuracyModel, GLMListVec3 &points);
        ~AverageVarianceEstimator();

    protected:
        virtual EigMatrix selectVarianceMatrix(EigMatrixList &mat); 
        /*
         * Computes the variance as the average of the elements in the matrix.
         */
        double computeVarianceFromMatrix(EigMatrix &varianceMatrix);

    };

} // namespace meshac

#endif // MESH_ACCURACY_AVERAGE_VARIANCE_ESTIMATOR_H
