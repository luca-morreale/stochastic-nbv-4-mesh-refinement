
#include <opview/SimpleConstrainedAngleViewEstimator.hpp>

namespace opview {

    SimpleConstrainedAngleViewEstimator::SimpleConstrainedAngleViewEstimator(Delaunay3 &dt, GLMVec3List &cams) : OptimalViewEstimator(dt)
    {
        this->cams = cams;
    }

    SimpleConstrainedAngleViewEstimator::~SimpleConstrainedAngleViewEstimator()
    {
        this->cams.clear();
    }
    

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

        this->addConstraints(model, barycenter);
        this->addGoal(model, barycenter, normVector, env);
    }

    void SimpleConstrainedAngleViewEstimator::addGoal(IloModel &model, PointD3 &barycenter, CGALVec3 &normVector, IloEnv env)
    {
        IloExpr goalExp = createGoalExpression(barycenter, normVector, env);
        model.add(IloMaximize(env, goalExp));
    }

    IloExpr SimpleConstrainedAngleViewEstimator::createGoalExpression(PointD3 &centroid, CGALVec3 &targetDirection, IloEnv env)
    {
        IloNumVarList point = convertVector(3, env);
        this->setGoalPoint(point);

        IloExpr numerator = dot(targetDirection, point);
        IloExpr denominator = norm(point);

        return numerator / denominator;
    }

    void SimpleConstrainedAngleViewEstimator::addConstraints(IloModel &model, PointD3 &barycenter)
    {
        IloRangeList constraintList = this->createConstraints(barycenter);
        for (IloRange constraint : constraintList) {
            model.add(constraint);
        }
    }

    IloRangeList SimpleConstrainedAngleViewEstimator::createConstraints(PointD3 &centroid)
    {
        IloRangeList constraintList;
        for (int i = 0; i < cams.size(); i++) {
            IloExpr pointDistance = sqrtDistance(goalPoint, cams[i]);
            constraintList.push_back(pointDistance >= BD_RAPPORT * BD_RAPPORT);
        }
        return constraintList;
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
