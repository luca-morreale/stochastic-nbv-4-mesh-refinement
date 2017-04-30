#include <opview/CplexViewEstimator.hpp>

namespace opview {

    CplexViewEstimator::CplexViewEstimator() : OptimalViewEstimator()
    { /*    */ }

    CplexViewEstimator::~CplexViewEstimator()
    { /*    */ }

    GLMVec3 CplexViewEstimator::determingBestPositionForFace(Face &face)
    {

        IloEnv optimizationEnv;
        IloModel model(optimizationEnv);

        this->buildModel(model, face);
        IloCplex cplex(optimizationEnv);
        this->setupCplex(cplex, optimizationEnv);

        try {
            cplex.extract(model);
            cplex.solve();
        }catch (IloException& e) {
            std::cout << e << std::endl;
            return GLMVec3();
        }

        GLMVec3 solution = this->extractSolutions(cplex);

        optimizationEnv.end();
        return solution;
    }

    void CplexViewEstimator::buildModel(IloModel model, Face &face)
    {
        CGALVec3 normVector = this->normalVectorToFace(face);
        PointD3 barycenter = this->barycenterVectorToFace(face.face);
        CGALVec3 centroid = convertPoinToCGALVec(barycenter);

        this->setUpGoalVariables(model.getEnv());

        this->addConstraints(model, centroid);
        this->addGoal(model, normVector, centroid);
    }

    void CplexViewEstimator::setUpGoalVariables(IloEnv env)
    {
        IloNumVarList point = createCplexVariables(3, env);
        this->setGoalPoint(point);
    }

    void CplexViewEstimator::addGoal(IloModel model, CGALVec3 &normVector,CGALVec3 &centroid)
    {
        IloExpr goalExp = createGoalExpression(normVector, centroid, model.getEnv());
        model.add(IloMaximize(model.getEnv(), goalExp));
    }

    void CplexViewEstimator::addConstraints(IloModel model, CGALVec3 &barycenter)
    {
        IloConstraintList constraintList = this->createConstraints(barycenter, model.getEnv());
        for (IloConstraint constraint : constraintList) {
            model.add(constraint);
        }
    }

    GLMVec3 CplexViewEstimator::extractSolutions(IloCplex &cplex)
    {
        std::cout << "trying to extract the solution\n";
        GLMVec3 solution;
        for (int i = 0; i < this->goalPoint.size(); i++) {
            std::cout << "value " << cplex.getValue(this->goalPoint[i]) << std::endl;
            solution[i] = cplex.getValue(this->goalPoint[i]);
        }
        std::cout << "extracted solution\n";
        return solution;
    }

    void CplexViewEstimator::setupCplex(IloCplex &cplex, IloEnv env)
    {
        cplex.setParam(IloCplex::Threads, 4);
        cplex.setOut(env.getNullStream());
    }

    IloNumVarList CplexViewEstimator::getGoalPoint()
    {
        return this->goalPoint;
    }

    void CplexViewEstimator::setGoalPoint(IloNumVarList &point)
    {
        this->goalPoint = point;
    }

} // namespace opview
