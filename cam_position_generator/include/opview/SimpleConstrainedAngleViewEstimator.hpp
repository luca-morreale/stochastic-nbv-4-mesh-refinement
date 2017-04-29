#ifndef CAM_POSITION_GENERATOR_CONSTRAINED_ANGLE_VIEW_ESTIMATOR_H
#define CAM_POSITION_GENERATOR_CONSTRAINED_ANGLE_VIEW_ESTIMATOR_H

#include <CGAL/barycenter.h>

#include <ilconcert/iloenv.h>
#include <ilconcert/ilomodel.h>


#include <opview/alias_definition.h>
#include <opview/OptimalViewEstimator.hpp>
#include <opview/CplexVectorOperations.hpp>

namespace opview {

    #define BD_RAPPORT 0.7

    class SimpleConstrainedAngleViewEstimator : public OptimalViewEstimator {
    public:
        SimpleConstrainedAngleViewEstimator(GLMVec3List &cams);
        ~SimpleConstrainedAngleViewEstimator();

    protected:
        virtual GLMVec3 determingBestPositionForFace(Face &face);

        virtual void buildModel(IloModel model, Face &face);
        virtual void setUpGoalVariables(IloEnv env);
        virtual void addGoal(IloModel model, CGALVec3 &normVector, CGALVec3 &centroid);
        virtual IloExpr createGoalExpression(CGALVec3 &targetDirection, CGALVec3 &centroid, IloEnv env);
        virtual void addConstraints(IloModel model, CGALVec3 &barycenter);
        virtual IloConstraintList createConstraints(CGALVec3 &centroid, IloEnv env);
        virtual GLMVec3 extractSolutions(IloCplex &cplex);


        IloNumVarList getGoalPoint();
        void setGoalPoint(IloNumVarList &point);

    private:
        IloNumVarList goalPoint;
        CGALVec3List cams;

        IloExpr middleToCenterDistanceExpr(CGALVec3 &cam, CGALVec3 &center, IloEnv env);
        IloConstraint createBDConstraint(CGALVec3 &cam, CGALVec3 &center, IloEnv env);
        
    };

    typedef SimpleConstrainedAngleViewEstimator* SimpleConstrainedAngleViewEstimatorPtr;

}   // namespace opview

#endif // CAM_POSITION_GENERATOR_CONSTRAINED_ANGLE_VIEW_ESTIMATOR_H
