
#include <opview/SimpleConstrainedAngleViewEstimator.hpp>

namespace opview {

    SimpleConstrainedAngleViewEstimator::SimpleConstrainedAngleViewEstimator(GLMVec3List &cams) : OptimalViewEstimator()
    {
        this->cams = convertListToCGALVecList(cams);
        
    }

    SimpleConstrainedAngleViewEstimator::~SimpleConstrainedAngleViewEstimator()
    {
        this->cams.clear();
    }

    GLMVec3 SimpleConstrainedAngleViewEstimator::determingBestPositionForFace(Face &face)
    {
        IloEnv optimizationEnv;
        IloModel model(optimizationEnv);

        std::cout << "starting building model\n";
        this->buildModel(model, face);
        std::cout << "model built\n";
        IloCplex cplex(optimizationEnv);
        
        std::cout << "created solver\n";
        cplex.setParam(IloCplex::Threads, 4);
        cplex.setOut(optimizationEnv.getNullStream());

        try {
            cplex.extract(model);
            std::cout << "added model\n";
            cplex.solve();
        }catch (IloException& e) {
            std::cout << e << std::endl;
            return GLMVec3();
        }

        std::cout << "solution " << cplex.getStatus() << std::endl;

        GLMVec3 solution = this->extractSolutions(cplex);

        optimizationEnv.end();
        return solution;
    }

    void SimpleConstrainedAngleViewEstimator::buildModel(IloModel model, Face &face)
    {
        CGALVec3 normVector = this->normalVectorToFace(face);
        PointD3 barycenter = this->barycenterVectorToFace(face.face);
        CGALVec3 centroid = convertPoinToCGALVec(barycenter);

        this->setUpGoalVariables(model.getEnv());

        this->addConstraints(model, centroid);
        this->addGoal(model, normVector, centroid);
    }

    void SimpleConstrainedAngleViewEstimator::setUpGoalVariables(IloEnv env)
    {
        IloNumVarList point = convertVector(3, env);
        this->setGoalPoint(point);
    }

    void SimpleConstrainedAngleViewEstimator::addGoal(IloModel model, CGALVec3 &normVector,CGALVec3 &centroid)
    {
        IloExpr goalExp = createGoalExpression(normVector, centroid, model.getEnv());
        model.add(IloMaximize(model.getEnv(), goalExp));
    }

    IloExpr SimpleConstrainedAngleViewEstimator::createGoalExpression(CGALVec3 &targetDirection, CGALVec3 &centroid, IloEnv env)
    {
        IloNumExprArray vectorPointCenter = differenceExpr(goalPoint, centroid, env);
        IloExpr numerator = dotExpr(targetDirection, vectorPointCenter, env);
        IloExpr denominator = sqrtNormExpr(vectorPointCenter, env);

        //return IloSquare(numerator) / denominator;
        //return IloSquare(numerator);
        return numerator;
    }

    void SimpleConstrainedAngleViewEstimator::addConstraints(IloModel model, CGALVec3 &barycenter)
    {
        IloConstraintList constraintList = this->createConstraints(barycenter, model.getEnv());
        for (IloConstraint constraint : constraintList) {
            model.add(constraint);
        }
    }

    IloConstraintList SimpleConstrainedAngleViewEstimator::createConstraints(CGALVec3 &centroid, IloEnv env)
    {
        IloConstraintList constraintList;

        for (int i = 0; i < cams.size(); i++) {
            constraintList.push_back(createBDConstraint(cams[i], centroid, env));
        }
        // constraint that limit the point in space, otherwise goes to the infinite
        //constraintList.push_back(pyramidDistanceExpr(goalPoint, centroid, env) <= 100.0);
        constraintList.push_back(manhattanDistanceExpr(goalPoint, centroid, env) <= 100.0);
        

        return constraintList;
    }

    IloConstraint SimpleConstrainedAngleViewEstimator::createBDConstraint(CGALVec3 &cam, CGALVec3 &center, IloEnv env)
    {
        //IloExpr distanceToCam = pyramidDistanceExpr(goalPoint, cam, env);
        IloExpr distanceToCam = manhattanDistanceExpr(goalPoint, cam, env);
        IloExpr rightSide = BD_RAPPORT * BD_RAPPORT * middleToCenterDistanceExpr(cam, center, env);

        return distanceToCam >= rightSide;
    }

    IloExpr SimpleConstrainedAngleViewEstimator::middleToCenterDistanceExpr(CGALVec3 &cam, CGALVec3 &center, IloEnv env)
    {
        IloExprList mp = middlePoint(goalPoint, cam, env);
        //return pyramidDistanceExpr(mp, center, env);
        return manhattanDistanceExpr(mp, center, env);
    }


    GLMVec3 SimpleConstrainedAngleViewEstimator::extractSolutions(IloCplex &cplex)
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

    IloNumVarList SimpleConstrainedAngleViewEstimator::getGoalPoint()
    {
        return this->goalPoint;
    }

    void SimpleConstrainedAngleViewEstimator::setGoalPoint(IloNumVarList &point)
    {
        this->goalPoint = point;
    }


} // namespace opview
