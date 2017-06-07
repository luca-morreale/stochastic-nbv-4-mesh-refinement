#include <opview/HierarchicalDiscreteGraphicalModel.hpp>

namespace opview {

    HierarchicalDiscreteGraphicalModel::HierarchicalDiscreteGraphicalModel(SolverGeneratorPtr solver, 
                                                                HierarchicalDiscretizationConfiguration &config, GLMVec3List &cams, 
                                                                double goalAngle, double dispersion)
                                                                : BasicGraphicalModel(solver, cams, goalAngle, dispersion)
    {
        this->config = config;
    }

    HierarchicalDiscreteGraphicalModel::~HierarchicalDiscreteGraphicalModel()
    { /*    */ }

    void HierarchicalDiscreteGraphicalModel::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        this->resetPosition();

        LabelList currentOptima = {MIN_COORDINATE, MIN_COORDINATE, MIN_COORDINATE, 0.0, 0.0};
        
        for (int d = 0; d < this->getDepth(); d++) {
            std::cout << "Current depth: " << d << std::endl;
            SimpleSpace space(numVariables(), numLabels());
            GraphicalModelAdder model(space);

            this->fillModel(model, centroid, normVector);
            
            // get optimal solution, than scale it
            // before give it to the solver descale it in the current space

            auto discreteOptima = getOptimaForDiscreteSpace(currentOptima);
            AdderInferencePtr algorithm = solverGenerator()->getOptimizerAlgorithm(model, discreteOptima, numVariables());
            algorithm->infer();

            currentOptima = this->extractResults(algorithm);

            this->reduceScale(currentOptima);
        }
    }

    LabelList HierarchicalDiscreteGraphicalModel::extractResults(AdderInferencePtr algorithm)
    {
        LabelList x;
        algorithm->arg(x);

        std::cout << "Value obtained: " << algorithm->value() << std::endl;

        GLMVec3 realOptima = scalePoint(GLMVec3(x[0], x[1], x[2]));
        x[0] = realOptima.x;
        x[1] = realOptima.y;
        x[2] = realOptima.z;

        std::cout << "Optimal solution: " << x[0] << ' ' << x[1] << ' ' << x[2] << std::endl;
        std::cout << std::endl << std::endl;

        return x;
    }

    LabelList HierarchicalDiscreteGraphicalModel::getOptimaForDiscreteSpace(LabelList &currentOptima)
    {
        GLMVec3 spaceOptima = unscalePoint(GLMVec3(currentOptima[0], currentOptima[1], currentOptima[2]));

        return {spaceOptima.x, spaceOptima.y, spaceOptima.z};
    }

    void HierarchicalDiscreteGraphicalModel::reduceScale(LabelList &currentOptimal)
    {
        float currentSize = scale() * (float)numLabels();
        float currentScale = scale();

        float nextSize = currentSize * 0.75f;
        float halfNextSize = nextSize / 2.0f;

        float nextScale = currentScale * 0.75f;
        float halfNextScale = nextScale / 2.0f;

        offsetX = [currentOptimal, halfNextSize, halfNextScale](){ return currentOptimal[0] - halfNextSize - halfNextScale; };
        offsetY = [currentOptimal, halfNextSize, halfNextScale](){ return currentOptimal[1] - halfNextSize - halfNextScale; };
        offsetZ = [currentOptimal, halfNextSize, halfNextScale](){ return currentOptimal[2] - halfNextSize - halfNextScale; };

        scale = [nextScale](){ return nextScale; };
    }

    void HierarchicalDiscreteGraphicalModel::resetPosition()
    {
        scale = [this](){ return (float)ORIGINAL_SIDE_SIZE / (float)numLabels(); };

        offsetX = [](){ return (float)MIN_COORDINATE; };
        offsetY = [](){ return (float)MIN_COORDINATE; };
        offsetZ = [](){ return (float)MIN_COORDINATE; };
    }

    size_t HierarchicalDiscreteGraphicalModel::setNumLabels(size_t labels)
    {
        this->config.labels = labels;
    }

    size_t HierarchicalDiscreteGraphicalModel::numLabels()
    {
        return this->config.labels;
    }

    size_t HierarchicalDiscreteGraphicalModel::getDepth()
    {
        return config.depth;
    }

    void HierarchicalDiscreteGraphicalModel::setDepth(size_t depth)
    {
        this->config.depth = depth;
    }



} // namespace opview
