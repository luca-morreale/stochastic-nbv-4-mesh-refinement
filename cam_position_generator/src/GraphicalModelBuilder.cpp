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
    { /*    */ }

    void GraphicalModelBuilder::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        SimpleSpace space(numVariables(), numLabels());
        GraphicalModelAdder model(space);

        this->fillModel(model, centroid, normVector);

        AdderInferencePtr algorithm = solver->getOptimizerAlgorithm(model, numVariables());
        algorithm->infer();

        LabelList x = this->extractResults(algorithm);
    }

    LabelList GraphicalModelBuilder::extractResults(AdderInferencePtr algorithm)
    {
        LabelList x;
        algorithm->arg(x);
        
        std::cout << algorithm->value() << std::endl;
        
        for (size_t j = 0; j < x.size(); ++j) {
            std::cout << x[j] << ' ';
        }
        
        std::cout << std::endl << std::endl;
        return x;
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

} // namespace opview
