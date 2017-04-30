#ifndef CAM_POSITION_GENERATOR_DISTANCE_CAMERA_VIEW_ESTIMATOR_H
#define CAM_POSITION_GENERATOR_DISTANCE_CAMERA_VIEW_ESTIMATOR_H

#include <cmath>

#include <opview/alias_definition.h>
#include <opview/CplexViewEstimator.hpp>
#include <opview/CplexVectorOperations.hpp>

namespace opview {

    #define BD_RAPPORT 0.7

    class DistanceCameraViewEstimator : public CplexViewEstimator {
    public:
        DistanceCameraViewEstimator(GLMVec3List &cams);
        ~DistanceCameraViewEstimator();

    protected:
        virtual IloExpr createGoalExpression(CGALVec3 &targetDirection, CGALVec3 &centroid, IloEnv env) = 0;
        
        virtual IloConstraintList createConstraints(CGALVec3 &centroid, IloEnv env);
    private:
        CGALVec3List cams;

        IloExpr middleToCenterDistanceExpr(CGALVec3 &cam, CGALVec3 &center, IloEnv env);
        IloConstraint createBDConstraint(CGALVec3 &cam, CGALVec3 &center, IloEnv env);
    };

    typedef DistanceCameraViewEstimator* DistanceCameraViewEstimatorPtr;

} // namespace opview

#endif // CAM_POSITION_GENERATOR_DISTANCE_CAMERA_VIEW_ESTIMATOR_H