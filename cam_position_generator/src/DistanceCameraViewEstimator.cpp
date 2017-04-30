#include <opview/DistanceCameraViewEstimator.hpp>

namespace opview {

    DistanceCameraViewEstimator::DistanceCameraViewEstimator(GLMVec3List &cams) : CplexViewEstimator()
    {
        this->cams = convertListToCGALVecList(cams);
    }

    DistanceCameraViewEstimator::~DistanceCameraViewEstimator()
    {
        this->cams.clear();
    }

    IloConstraintList DistanceCameraViewEstimator::createConstraints(CGALVec3 &centroid, IloEnv env)
    {
        IloConstraintList constraintList;

        for (int i = 0; i < cams.size(); i++) {
            constraintList.push_back(createBDConstraint(cams[i], centroid, env));
        }

        IloNumVarList goalPoint = this->getGoalPoint();
        // constraint that limit the point in space, otherwise goes to the infinite
        //constraintList.push_back(pyramidDistanceExpr(goalPoint, centroid, env) <= 100.0);
        constraintList.push_back(manhattanDistanceExpr(goalPoint, centroid, env) <= 100.0);
        

        return constraintList;
    }

    IloConstraint DistanceCameraViewEstimator::createBDConstraint(CGALVec3 &cam, CGALVec3 &center, IloEnv env)
    {
        IloNumVarList goalPoint = this->getGoalPoint();
        //IloExpr distanceToCam = pyramidDistanceExpr(goalPoint, cam, env);
        IloExpr distanceToCam = manhattanDistanceExpr(goalPoint, cam, env);
        IloExpr rightSide = BD_RAPPORT * BD_RAPPORT * middleToCenterDistanceExpr(cam, center, env);

        return distanceToCam >= rightSide;
    }

    IloExpr DistanceCameraViewEstimator::middleToCenterDistanceExpr(CGALVec3 &cam, CGALVec3 &center, IloEnv env)
    {
        IloNumVarList goalPoint = this->getGoalPoint();
        IloExprList mp = middlePoint(goalPoint, cam, env);
        //return pyramidDistanceExpr(mp, center, env);
        return manhattanDistanceExpr(mp, center, env);
    }

    
} // namespace opview
