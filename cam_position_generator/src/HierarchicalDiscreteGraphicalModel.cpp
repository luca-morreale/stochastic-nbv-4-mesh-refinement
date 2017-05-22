#include <opview/HierarchicalDiscreteGraphicalModel.hpp>

namespace opview {

    HierarchicalDiscreteGraphicalModel::HierarchicalDiscreteGraphicalModel(SolverGeneratorPtr solver, size_t depth, GLMVec3List &cams, 
                                                                            double goalAngle, double dispersion)
                                                                            : BasicGraphicalModel(solver, cams, goalAngle, dispersion)
    {
        this->depth = depth; 
    }

    HierarchicalDiscreteGraphicalModel::~HierarchicalDiscreteGraphicalModel()
    { /*    */ }

    void HierarchicalDiscreteGraphicalModel::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        this->resetPosition();

        for (int d = 0; d < this->depth; d++) {
            SimpleSpace space(numVariables(), numLabels());
            GraphicalModelAdder model(space);

            this->fillModel(model, centroid, normVector);

            AdderInferencePtr algorithm = solverGenerator()->getOptimizerAlgorithm(model, numVariables());
            algorithm->infer();

            LabelList x = this->extractResults(algorithm);

            this->reduceScale(x);
        }
    }

    void HierarchicalDiscreteGraphicalModel::reduceScale(LabelList currentOptimal)
    {
        float currentScale = scale();
        float half_size = scale() / 2.0f;

        offsetX = [currentOptimal, half_size](){ return currentOptimal[0] - half_size; };
        offsetY = [currentOptimal, half_size](){ return currentOptimal[1] - half_size; };
        offsetZ = [currentOptimal, half_size](){ return currentOptimal[2] - half_size; };

        scale = [currentScale, this](){ return currentScale / (float)numLabels(); };
    }

    void HierarchicalDiscreteGraphicalModel::resetPosition()
    {
        scale = [this](){ return (float)ORIGINAL_SIDE_SIZE / (float)numLabels(); };

        offsetX = [](){ return (float)MIN_COORDINATE; };
        offsetY = [](){ return (float)MIN_COORDINATE; };
        offsetZ = [](){ return (float)MIN_COORDINATE; };
    }

    size_t HierarchicalDiscreteGraphicalModel::numVariables()
    {
        return VARS;
    }

    size_t HierarchicalDiscreteGraphicalModel::numLabels()
    {
        return DISCRETE_LABELS;
    }



} // namespace opview
