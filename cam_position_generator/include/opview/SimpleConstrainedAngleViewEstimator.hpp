#ifndef CAM_POSITION_GENERATOR_CONSTRAINED_ANGLE_VIEW_ESTIMATOR_H
#define CAM_POSITION_GENERATOR_CONSTRAINED_ANGLE_VIEW_ESTIMATOR_H

#include <CGAL/barycenter.h>

#include <ilconcert/iloenv.h>
#include <ilconcert/ilomodel.h>


#include <opview/alias_definition.h>
#include <opview/DistanceCameraViewEstimator.hpp>
#include <opview/CplexVectorOperations.hpp>

namespace opview {

    class SimpleConstrainedAngleViewEstimator : public DistanceCameraViewEstimator {
    public:
        SimpleConstrainedAngleViewEstimator(GLMVec3List &cams);
        ~SimpleConstrainedAngleViewEstimator();

    protected:
        virtual IloExpr createGoalExpression(CGALVec3 &targetDirection, CGALVec3 &centroid, IloEnv env);
        
    };

    typedef SimpleConstrainedAngleViewEstimator* SimpleConstrainedAngleViewEstimatorPtr;

}   // namespace opview

#endif // CAM_POSITION_GENERATOR_CONSTRAINED_ANGLE_VIEW_ESTIMATOR_H
