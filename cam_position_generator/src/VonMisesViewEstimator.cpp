#include <opview/VonMisesViewEstimator.hpp>

namespace opview {

    VonMisesViewEstimator::VonMisesViewEstimator(GLMVec3List &cams) : DistanceCameraViewEstimator(cams)
    { /*    */ }

    VonMisesViewEstimator::~VonMisesViewEstimator()
    { /*    */ }

    IloExpr VonMisesViewEstimator::createGoalExpression(CGALVec3 &targetDirection, CGALVec3 &centroid, IloEnv env)
    {
        IloNumVarList goalPoint = this->getGoalPoint();

        double bessel = 1.0;
        double k = 1.0;

        auto v = differenceExpr(goalPoint, centroid, env);
        auto dot = dotExpr(targetDirection, v, env);
        auto norm = sqrtNormExpr(v, env);

        //auto numerator = IloExponent(k * dot / norm);
        auto numerator = IloExponent(k * dot);
        //auto numerator = IloExponent(k * IloSquare(dot));

        auto denominator = 2 * M_PI * bessel;

        //return numerator / denominator;
        return numerator;
    }

} // namespace opview
