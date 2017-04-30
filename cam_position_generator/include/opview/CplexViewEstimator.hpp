#ifndef CAM_POSITION_GENERATOR_CPLEX_VIEW_ESTIMATOR_H
#define CAM_POSITION_GENERATOR_CPLEX_VIEW_ESTIMATOR_H

#include <cmath>

#include <opview/alias_definition.h>
#include <opview/OptimalViewEstimator.hpp>
#include <opview/CplexVectorOperations.hpp>

namespace opview {

    class CplexViewEstimator : public OptimalViewEstimator {
    public:
        CplexViewEstimator();
        ~CplexViewEstimator();

    protected:
        virtual GLMVec3 determingBestPositionForFace(Face &face);

        virtual void buildModel(IloModel model, Face &face);
        virtual void setUpGoalVariables(IloEnv env);
        
        virtual void addGoal(IloModel model, CGALVec3 &normVector, CGALVec3 &centroid);
        virtual IloExpr createGoalExpression(CGALVec3 &targetDirection, CGALVec3 &centroid, IloEnv env) = 0;
        
        virtual void addConstraints(IloModel model, CGALVec3 &barycenter);
        virtual IloConstraintList createConstraints(CGALVec3 &centroid, IloEnv env) = 0;
        
        virtual GLMVec3 extractSolutions(IloCplex &cplex);
        
        IloNumVarList getGoalPoint();
        void setGoalPoint(IloNumVarList &point);

    private:
        IloNumVarList goalPoint;

        void setupCplex(IloCplex &cplex, IloEnv env);
    };

    typedef CplexViewEstimator* CplexViewEstimatorPtr;

} // namespace opview

#endif // CAM_POSITION_GENERATOR_CPLEX_VIEW_ESTIMATOR_H