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
        SimpleConstrainedAngleViewEstimator(Delaunay3 &dt, GLMVec3List &cams);
        ~SimpleConstrainedAngleViewEstimator();

    protected:
        virtual GLMVec3 determingBestPositionForFace(Face face);

        virtual void buildModel(IloModel &model, IloEnv env, Face &face);
        virtual void addGoal(IloModel &model, PointD3 &barycenter, CGALVec3 &normVector, IloEnv env);
        virtual IloExpr createGoalExpression(PointD3 &centroid, CGALVec3 &targetDirection, IloEnv env);
        virtual void addConstraints(IloModel &model, PointD3 &barycenter);
        virtual IloRangeList createConstraints(PointD3 &centroid);
        virtual GLMVec3 extractSolutions(IloCplex &cplex);


        IloNumVarList getGoalPoint();
        void setGoalPoint(IloNumVarList point);

    private:
        IloNumVarList goalPoint;
        GLMVec3List cams;
        
    };   
}   // namespace opview

#endif // CAM_POSITION_GENERATOR_CONSTRAINED_ANGLE_VIEW_ESTIMATOR_H
