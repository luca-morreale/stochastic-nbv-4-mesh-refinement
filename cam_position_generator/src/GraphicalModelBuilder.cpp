#include <opview/GraphicalModelBuilder.hpp>

#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/graphicalmodel/space/simplediscretespace.hxx>
#include <opengm/functions/potts.hxx>
#include <opengm/operations/adder.hxx>
#include <opengm/inference/messagepassing/messagepassing.hxx>

namespace opview {

    GraphicalModelBuilder::GraphicalModelBuilder(SolverGeneratorPtr solver)
    {
        this->solver = solver;
    }

    GraphicalModelBuilder::~GraphicalModelBuilder()
    {
        delete solver;
    }

    LabelList GraphicalModelBuilder::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        SimpleSpace space(shape.begin(), shape.end());
        GraphicalModelAdder model(space);

        this->fillModel(model, centroid, normVector);

        AdderInferencePtr algorithm = solver->getOptimizerAlgorithm(model, VarIndexList(), numVariables());
        algorithm->infer();

        LabelList x = this->extractResults(algorithm);
        return x;
    }

    LabelList GraphicalModelBuilder::extractResults(AdderInferencePtr algorithm)
    {
        VarIndexList x;
        algorithm->arg(x);
        
        // std::cout << "Value obtained: " << algorithm->value() << std::endl;
        
        LabelList optima;
        optima.insert(optima.end(), x.begin(), x.end());

        // std::cout << "Optimal solution: ";
        // for (size_t j = 0; j < x.size(); ++j) {
        //     std::cout << x[j] << ' ';
        // }
        
        // std::cout << std::endl << std::endl;
        return optima;
    }

    void GraphicalModelBuilder::addFunctionTo(GMExplicitFunction &fun, GraphicalModelAdder &model, VarIndexList &variableIndices)
    {
        auto fid = model.addFunction(fun);
        model.addFactor(fid, variableIndices.begin(), variableIndices.end());
    }

    void GraphicalModelBuilder::addFunctionTo(GMSparseFunction &fun, GraphicalModelAdder &model, VarIndexList &variableIndices)
    {
        auto fid = model.addFunction(fun);
        model.addFactor(fid, variableIndices.begin(), variableIndices.end());
    }

    SolverGeneratorPtr GraphicalModelBuilder::solverGenerator()
    {
        return solver;
    }

    void GraphicalModelBuilder::setSolverGenerator(SolverGeneratorPtr solver)
    {
        delete this->solver;
        this->solver = solver;
    }

} // namespace opview
