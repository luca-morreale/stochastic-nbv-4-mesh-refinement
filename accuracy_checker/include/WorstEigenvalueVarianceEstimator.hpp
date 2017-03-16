#ifndef MESH_ACCURACY_WORST_EIGENVALUE_VARIANCE_ESTIMATOR_H
#define MESH_ACCURACY_WORST_EIGENVALUE_VARIANCE_ESTIMATOR_H

#include <alias_definition.hpp>
#include <Point3DVarianceEstimator.hpp>


namespace meshac {

    extern const float SENSIBILITY;


    class WorstEigenvalueVarianceEstimator : public Point3DVarianceEstimator {
    public:
        WorstEigenvalueVarianceEstimator(AccuracyModelPtr accuracyModel, GLMListVec3 points);
        ~WorstEigenvalueVarianceEstimator();

    protected:
        EigMatrix selectVarianceMatrix(EigMatrixList &mat);

    };

} // namespace meshac

#endif // MESH_ACCURACY_WORST_VARIANCE_ESTIMATOR_H
