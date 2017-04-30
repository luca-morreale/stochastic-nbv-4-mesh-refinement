#ifndef CAM_POSITION_GENERATOR_VON_MISES_VIEW_ESTIMATOR_H
#define CAM_POSITION_GENERATOR_VON_MISES_VIEW_ESTIMATOR_H

#include <cmath>

#include <opview/alias_definition.h>
#include <opview/DistanceCameraViewEstimator.hpp>
#include <opview/CplexVectorOperations.hpp>

namespace opview {

    class VonMisesViewEstimator : public DistanceCameraViewEstimator {
    public:
        VonMisesViewEstimator(GLMVec3List &cams);
        ~VonMisesViewEstimator();

    protected:
        virtual IloExpr createGoalExpression(CGALVec3 &targetDirection, CGALVec3 &centroid, IloEnv env);

    };

    typedef VonMisesViewEstimator* VonMisesViewEstimatorPtr;

} // namespace opview

#endif // CAM_POSITION_GENERATOR_VON_MISES_VIEW_ESTIMATOR_H