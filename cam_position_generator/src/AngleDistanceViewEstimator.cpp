#include <opview/AngleDistanceViewEstimator.hpp>

namespace opview {

    AngleDistanceViewEstimator::AngleDistanceViewEstimator(GLMVec3List &cams) : DistanceCameraViewEstimator(cams)
    { /*    */ }

    AngleDistanceViewEstimator::~AngleDistanceViewEstimator()
    { /*    */ }

    void AngleDistanceViewEstimator::addGoal(IloModel model, CGALVec3 &normVector,CGALVec3 &centroid)
    {
        IloExpr goalExp = createGoalExpression(normVector, centroid, model.getEnv());
        model.add(IloMinimize(model.getEnv(), goalExp));
    }

    IloExpr AngleDistanceViewEstimator::createGoalExpression(CGALVec3 &targetDirection, CGALVec3 &centroid, IloEnv env)
    {
        IloNumVarList goalPoint = this->getGoalPoint();
        CGALVec3 pointB = centroid + MIN_DISTANCE * targetDirection;

        auto denominator = distanceCGALVec3(centroid, pointB);
        auto numerator = manhattanDistanceExpr(goalPoint, centroid, env) * manhattanDistanceExpr(goalPoint, pointB, env);

        //return numerator / denominator;
        return numerator;
    }

} // namespace opview
