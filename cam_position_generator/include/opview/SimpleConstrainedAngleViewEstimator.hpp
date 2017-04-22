#ifndef CAM_POSITION_GENERATOR_CONSTRAINED_ANGLE_VIEW_ESTIMATOR_H
#define CAM_POSITION_GENERATOR_CONSTRAINED_ANGLE_VIEW_ESTIMATOR_H

#include <CGAL/barycenter.h>

#include <ilconcert/iloenv.h>
#include <ilconcert/ilomodel.h>


#include <opview/alias_definition.h>
#include <opview/OptimalViewEstimator.hpp>
#include <opview/CplexVectorOperations.hpp>

namespace opview {

    class SimpleConstrainedAngleViewEstimator : public OptimalViewEstimator {
    public:
        SimpleConstrainedAngleViewEstimator(Delaunay3 &dt);
        ~SimpleConstrainedAngleViewEstimator();

    protected:
        virtual GLMVec3 determingBestPositionForFace(Face face);

        virtual void buildModel(IloModel &model, IloEnv env, Face &face);
        virtual IloExpr createProblem(PointD3 &centroid, CGALVec3 &targetDirection, IloEnv env);
        virtual GLMVec3 extractSolutions(IloCplex &cplex);


        IloNumVarList getGoalPoint();
        void setGoalPoint(IloNumVarList point);

    private:
        IloNumVarList goalPoint;
        
    };   
}   // namespace opview

#endif // CAM_POSITION_GENERATOR_CONSTRAINED_ANGLE_VIEW_ESTIMATOR_H
