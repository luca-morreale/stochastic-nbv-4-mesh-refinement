#include <opview/HierarchicalDiscreteGraphicalModel.hpp>

namespace opview {

    HierarchicalDiscreteGraphicalModel::HierarchicalDiscreteGraphicalModel(SolverGeneratorPtr solver, size_t depth, size_t labels,
                                                                            GLMVec3List &cams, double goalAngle, double dispersion)
                                                                            : BasicGraphicalModel(solver, cams, goalAngle, dispersion)
    {
        this->depth = depth;
        this->labels = labels;
    }

    HierarchicalDiscreteGraphicalModel::~HierarchicalDiscreteGraphicalModel()
    { /*    */ }

    void HierarchicalDiscreteGraphicalModel::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        this->resetPosition();

        LabelList currentOptimal;
        for (int d = 0; d < this->depth; d++) {
            SimpleSpace space(numVariables(), numLabels());
            GraphicalModelAdder model(space);

            this->fillModel(model, centroid, normVector);

            AdderInferencePtr algorithm = solverGenerator()->getOptimizerAlgorithm(model, currentOptimal, numVariables());
            algorithm->infer();

            currentOptimal = this->extractResults(algorithm);
            this->reduceScale(currentOptimal);
        }
    }

    void HierarchicalDiscreteGraphicalModel::reduceScale(LabelList currentOptimal)
    {
        float currentScale = scale();
        float halfSize = scale() / 2.0f;
        float halfNextScale = halfSize / (float)numLabels();

        offsetX = [currentOptimal, halfSize, halfNextScale](){ return currentOptimal[0] - halfSize - halfNextScale; };
        offsetY = [currentOptimal, halfSize, halfNextScale](){ return currentOptimal[1] - halfSize - halfNextScale; };
        offsetZ = [currentOptimal, halfSize, halfNextScale](){ return currentOptimal[2] - halfSize - halfNextScale; };

        scale = [halfNextScale](){ return halfNextScale * 2.0f; };
    }

    void HierarchicalDiscreteGraphicalModel::resetPosition()
    {
        scale = [this](){ return (float)ORIGINAL_SIDE_SIZE / (float)numLabels(); };

        offsetX = [](){ return (float)MIN_COORDINATE; };
        offsetY = [](){ return (float)MIN_COORDINATE; };
        offsetZ = [](){ return (float)MIN_COORDINATE; };
    }

    size_t HierarchicalDiscreteGraphicalModel::numLabels()
    {
        return this->labels;
    }



} // namespace opview
