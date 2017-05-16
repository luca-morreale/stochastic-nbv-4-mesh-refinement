#include <opview/AngleDifferenceViewEstimator.hpp>


namespace opview {

    AngleDifferenceViewEstimator::AngleDifferenceViewEstimator(GLMVec3List &cams) : DistanceCameraViewEstimator(cams)
    { /*    */ }

    AngleDifferenceViewEstimator::~AngleDifferenceViewEstimator()
    { /*    */ }

    void AngleDifferenceViewEstimator::addGoal(IloModel model, CGALVec3 &normVector,CGALVec3 &centroid)
    {
        IloExpr goalExp = createGoalExpression(normVector, centroid, model.getEnv());
        model.add(IloMinimize(model.getEnv(), goalExp));
    }

    IloExpr AngleDifferenceViewEstimator::createGoalExpression(CGALVec3 &targetDirection, CGALVec3 &centroid, IloEnv env)
    {
        IloNumVarList goalPoint = this->getGoalPoint();
        //CGALVec3 pointB = centroid + MIN_DISTANCE * targetDirection;
        IloNumExprArray v = differenceExpr(goalPoint, centroid, env);
        double fixedAngle = 1;

        return dotExpr(targetDirection, goalPoint, env) * dotExpr(targetDirection, goalPoint, env) - fixedAngle * sqrtNormExpr(v, env);
    }


} // namespace opview
