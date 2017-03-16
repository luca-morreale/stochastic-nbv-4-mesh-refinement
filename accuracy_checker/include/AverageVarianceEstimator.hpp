
#ifndef MESH_ACCURACY_AVERAGE_VARIANCE_ESTIMATOR_H
#define MESH_ACCURACY_AVERAGE_VARIANCE_ESTIMATOR_H

#include <alias_definition.hpp>
#include <Point3DVarianceEstimator.hpp>


namespace meshac {

    extern const float SENSIBILITY;


    class AverageVarianceEstimator : public Point3DVarianceEstimator {
    public:
        AverageVarianceEstimator(AccuracyModelPtr accuracyModel, GLMListVec3 points);
        ~AverageVarianceEstimator();

    protected:
        EigMatrix selectVarianceMatrix(EigMatrixList &mat);

    };

} // namespace meshac

#endif // MESH_ACCURACY_AVERAGE_VARIANCE_ESTIMATOR_H
