#include <opview/HierarchicalDiscreteGraphicalModel.hpp>

namespace opview {

    HierarchicalDiscreteGraphicalModel::HierarchicalDiscreteGraphicalModel(SolverGeneratorPtr solver, 
                                                                HierarchicalDiscretizationConfiguration &config, GLMVec3List &cams, 
                                                                double goalAngle, double dispersion)
                                                                : BasicGraphicalModel(solver, cams, goalAngle, dispersion)
    {
        this->config = config;
        this->initShapes();
    }

    HierarchicalDiscreteGraphicalModel::~HierarchicalDiscreteGraphicalModel()
    { /*    */ }

    void HierarchicalDiscreteGraphicalModel::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        this->resetPosition();

        LabelList currentOptima = {MIN_COORDINATE, MIN_COORDINATE, MIN_COORDINATE, 0.0, 0.0};
        
        for (int d = 0; d < this->getDepth(); d++) {
            std::cout << "Current depth: " << d << std::endl;
            SimpleSpace space(shape.begin(), shape.end());
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
        VarIndexList x;
        algorithm->arg(x);

        std::cout << "Value obtained: " << algorithm->value() << std::endl;

        LabelList convertedOpt(x.size());
        GLMVec3 realOptima = scalePoint(GLMVec3(x[0], x[1], x[2]));
        convertedOpt[0] = realOptima.x;
        convertedOpt[1] = realOptima.y;
        convertedOpt[2] = realOptima.z;

        std::cout << "Optimal solution: " << convertedOpt[0] << ' ' << convertedOpt[1] << ' ' << convertedOpt[2] << std::endl;
        std::cout << std::endl << std::endl;

        return convertedOpt;
    }

    VarIndexList HierarchicalDiscreteGraphicalModel::getOptimaForDiscreteSpace(LabelList &currentOptima)
    {
        GLMVec3 spaceOptima = unscalePoint(GLMVec3(currentOptima[0], currentOptima[1], currentOptima[2]));

        return {(VariableIndexType)spaceOptima.x, (VariableIndexType)spaceOptima.y, (VariableIndexType)spaceOptima.z};
    }

    void HierarchicalDiscreteGraphicalModel::reduceScale(LabelList &currentOptimal)
    {
        float currentSize = scale() * (float)numLabels();
        float currentScale = scale();

        float nextSize = currentSize * 0.75f;

        float nextScale = nextSize / (float)numLabels(); // 3/4 of the current scale
        float halfNextSize = (float)numLabels() * nextScale / 2.0f;

        // offset = optima - halfNextSize + x * scale - y * scale
        float tmpOffsetX = currentOptimal[0] - halfNextSize + getXScalingFactor(currentOptimal[0], halfNextSize, nextScale) * nextScale 
                                                            - getYScalingFactor(currentOptimal[0], halfNextSize, nextScale) * nextScale;
        float tmpOffsetY = currentOptimal[1] - halfNextSize + getXScalingFactor(currentOptimal[1], halfNextSize, nextScale) * nextScale 
                                                            - getYScalingFactor(currentOptimal[1], halfNextSize, nextScale) * nextScale;
        float tmpOffsetZ = currentOptimal[2] - halfNextSize + getXScalingFactor(currentOptimal[2], halfNextSize, nextScale) * nextScale 
                                                            - getYScalingFactor(currentOptimal[2], halfNextSize, nextScale) * nextScale;

        offsetX = [tmpOffsetX](){ return tmpOffsetX; };
        offsetY = [tmpOffsetY](){ return tmpOffsetY; };
        // std::max to force z to stay in the positive space
        offsetZ = [tmpOffsetZ](){ return tmpOffsetZ; };

        // offsetX = [currentOptimal, halfNextSize](){ return std::max(std::min(currentOptimal[0] - halfNextSize, 3.0-halfNextSize), -3.0+halfNextSize); };
        // offsetY = [currentOptimal, halfNextSize](){ return std::max(std::min(currentOptimal[1] - halfNextSize, 3.0-halfNextSize), -3.0+halfNextSize); };
        // // std::max to force z to stay in the positive space
        // offsetZ = [currentOptimal, halfNextSize](){ return std::max(std::min(currentOptimal[2] - halfNextSize, 3.0-halfNextSize), -3.0+halfNextSize); };

        scale = [nextScale](){ return nextScale; };
    }

    int HierarchicalDiscreteGraphicalModel::getXScalingFactor(float currentOptima, float halfNextSize, float scale)
    {
        int x = 0;
        while (currentOptima - halfNextSize + x * scale <  -0.5) {
            x++;
        }
        return x;
    }

    int HierarchicalDiscreteGraphicalModel::getYScalingFactor(float currentOptima, float halfNextSize, float scale)
    {
        int y = 0;
        while (currentOptima - halfNextSize - y * scale >  1.0) {
            y++;
        }
        return y;
    }

    void HierarchicalDiscreteGraphicalModel::resetPosition()
    {
        scale = [this](){ return (float)ORIGINAL_SIDE_SIZE / (float)numLabels(); };

        offsetX = [](){ return (float)MIN_COORDINATE; };
        offsetY = [](){ return (float)-0.5; };
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
