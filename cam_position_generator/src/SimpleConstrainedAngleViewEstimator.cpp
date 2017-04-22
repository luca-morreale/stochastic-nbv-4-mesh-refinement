
#include <opview/SimpleConstrainedAngleViewEstimator.hpp>

namespace opview {

    SimpleConstrainedAngleViewEstimator::SimpleConstrainedAngleViewEstimator(Delaunay3 &dt) : OptimalViewEstimator(dt)
    {
        //optimizationEnv = new IloEnvironment();
        //cplex = new IloCplex(optimizationEnv);
    }

    SimpleConstrainedAngleViewEstimator::~SimpleConstrainedAngleViewEstimator()
    {
        //optimizationEnv->end();

        //delete cplex;
        //delete optimizationEnv;
    }
    /*
    GLMVec3 SimpleConstrainedAngleViewEstimator::determingBestPositionForFace(Face face)
    {
        IloEnv optimizationEnv;
        IloCplex cplex;
        IloModel model(optimizationEnv);
        IloNumVar x1(optimizationEnv, -500.0, 500.0, ILOFLOAT);     // lowerbound 0, upperbound 40 and type continuous





        model.add(IloMaximize(optimizationEnv, logVonMises(0, 40)));      // added goal to minimize
        //model.add(-x1 + x2 + x3 <= 20);     // create constraint

        
        //IloCplex cplex(model);
        cplex.extract(model);
        cplex.solve();

        //IloNum val1 = cplex.getValue(x1);

        //IloNum objval = cplex.getObjValue();

        //cplex.clearModel();

        optimizationEnv.end();
    }

    IloExpr logBessel0(IloExpr x)   // log of I0(x) as approximately x − 1/2 log(2 *pi * x)     https://math.stackexchange.com/questions/376758/exponential-approximation-of-the-modified-bessel-function-of-first-kind-equatio
    {
        IloExpr logArg = 2 * M_PI * x;
        return x - 0.5 * IloLog(logArg);
    }

    IloExpr logVonMises(IloNumVar alpha, IloNumVar kappa)   // alpha should be for sure an expression, kappa can even be a number!
    {
        return alpha * kappa - log(2 * M_PI) - logBessel0(kappa);
    }

    IloExpr logVonMises(EigVector n, EigVector v, EigVector goalDirection)
    {
        IloNumVar x1(optimizationEnv, -500.0, 500.0, ILOFLOAT);     // lowerbound 0, upperbound 40 and type continuous
        IloNumVar x1(optimizationEnv, -500.0, 500.0, ILOFLOAT);     // lowerbound 0, upperbound 40 and type continuous
        IloNumVar x1(optimizationEnv, -500.0, 500.0, ILOFLOAT);     // lowerbound 0, upperbound 40 and type continuous
    }


    IloExpr logVonMises(EigVector n, EigVector v, EigVector goalDirection)
    {
        double dotProduct = n.dot(v);
        double normProduct = n.norm() * v.norm();
        double alpha = dotProduct / normProduct;

        dotProduct = n.dot(goalDirection);
        normProduct = n.norm() * goalDirection.norm();
        double k = dotProduct / normProduct;

        return logVonMises(alpha, k);
    }

    IloExpr logVonMises(EigVector n, EigVector v, double k)
    {
        auto dotProduct = n.dot(v);
        auto normProduct = n.norm() * v.norm();
        double alpha = dotProduct / normProduct;

        return logVonMises(alpha, k);
    }
    */

    GLMVec3 SimpleConstrainedAngleViewEstimator::determingBestPositionForFace(Face face)
    {
        IloEnv optimizationEnv;
        IloCplex cplex;
        IloModel model(optimizationEnv);

        this->buildModel(model, optimizationEnv, face);

        cplex.extract(model);
        cplex.solve();

        GLMVec3 solution = this->extractSolutions(cplex);

        optimizationEnv.end();
    }

    void SimpleConstrainedAngleViewEstimator::buildModel(IloModel &model, IloEnv env, Face &face)
    {
        CGALVec3 normVector = this->normalVectorToFace(face);
        PointD3 barycenter = this->barycenterVectorToFace(face.face);

        IloExpr goalExp = createProblem(barycenter, normVector, env);
        model.add(IloMaximize(env, goalExp));      // added goal to minimize
    }


    IloExpr SimpleConstrainedAngleViewEstimator::createProblem(PointD3 &centroid, CGALVec3 &targetDirection, IloEnv env)
    {
        IloNumVarList point = convertVector(3, env);
        this->setGoalPoint(point);

        IloExpr numerator = dot(targetDirection, point);
        IloExpr denominator = norm(point);

        return numerator / denominator;
    }

    GLMVec3 SimpleConstrainedAngleViewEstimator::extractSolutions(IloCplex &cplex)
    {
        GLMVec3 solution;
        for (int i = 0; i < glm::length(solution); i++) {
            solution[i] = cplex.getValue(this->goalPoint[i]);
        }
        return solution;
    }

    IloNumVarList SimpleConstrainedAngleViewEstimator::getGoalPoint()
    {
        return this->goalPoint;
    }

    void SimpleConstrainedAngleViewEstimator::setGoalPoint(IloNumVarList point)
    {
        this->goalPoint = point;
    }


} // namespace opview
