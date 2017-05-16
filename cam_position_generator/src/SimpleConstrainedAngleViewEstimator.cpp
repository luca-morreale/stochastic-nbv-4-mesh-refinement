#include <opview/SimpleConstrainedAngleViewEstimator.hpp>

namespace opview {

    SimpleConstrainedAngleViewEstimator::SimpleConstrainedAngleViewEstimator(GLMVec3List &cams) : DistanceCameraViewEstimator(cams)
    { /*    */ }

    SimpleConstrainedAngleViewEstimator::~SimpleConstrainedAngleViewEstimator()
    { /*    */ }

    IloExpr SimpleConstrainedAngleViewEstimator::createGoalExpression(CGALVec3 &targetDirection, CGALVec3 &centroid, IloEnv env)
    {
        IloNumVarList goalPoint = this->getGoalPoint();
        
        IloNumExprArray vectorPointCenter = differenceExpr(goalPoint, centroid, env);
        IloExpr numerator = dotExpr(targetDirection, vectorPointCenter, env);
        IloExpr denominator = sqrtNormExpr(vectorPointCenter, env);

        //return IloSquare(numerator) / denominator;
        //return IloSquare(numerator);
        return numerator * numerator;

        //return goalPoint[0] * goalPoint[0] / (goalPoint[1] * goalPoint[1]);
    }



} // namespace opview
