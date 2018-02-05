#include <opview/HierarchicalDiscreteGraphicalModel.hpp>

namespace opview {

    HierarchicalDiscreteGraphicalModel::HierarchicalDiscreteGraphicalModel(SolverGeneratorPtr solver, 
                                                                HierarchicalDiscretizationConfiguration &config, GLMVec3List &cams, 
                                                                double goalAngle, double dispersion)
                                                                : BasicGraphicalModel(solver, cams, goalAngle, dispersion)
    {
        this->config = config;
        this->Y = config.bounds.upper.y;
        this->initShapes();
        this->resetPosition();
    }

    HierarchicalDiscreteGraphicalModel::~HierarchicalDiscreteGraphicalModel()
    { /*    */ }

    LabelList HierarchicalDiscreteGraphicalModel::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        this->resetPosition();

        LabelList currentOptima = {config.bounds.lower.x, this->Y, config.bounds.lower.y, 0.0, 0.0};
        
        for (int d = 0; d < this->getDepth(); d++) {
            
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
        return currentOptima;
    }

    LabelList HierarchicalDiscreteGraphicalModel::extractResults(AdderInferencePtr algorithm)
    {
        VarIndexList x;
        algorithm->arg(x);

        LabelList convertedOpt(x.size());
        GLMVec3 realOptima = scalePoint(GLMVec3(x[0], x[1], x[2]));
        convertedOpt[0] = realOptima.x;
        convertedOpt[1] = realOptima.y;
        convertedOpt[2] = realOptima.z;
        
        return convertedOpt;
    }

    VarIndexList HierarchicalDiscreteGraphicalModel::getOptimaForDiscreteSpace(LabelList &currentOptima)
    {
        GLMVec3 spaceOptima = unscalePoint(GLMVec3(currentOptima[0], currentOptima[1], currentOptima[2]));

        return {(VariableIndexType)spaceOptima.x, (VariableIndexType)spaceOptima.y, (VariableIndexType)spaceOptima.z};
    }

    void HierarchicalDiscreteGraphicalModel::reduceScale(LabelList &currentOptimal)
    {
        GLMVec3 currentSize = scale() * (float)numLabels();
        GLMVec3 currentScale = scale();

        GLMVec3 nextSize = currentSize * 0.75f;

        GLMVec3 nextScale = nextSize / (float)numLabels(); // 3/4 of the current scale
        GLMVec3 halfNextSize = nextSize / 2.0f;
                                                            
        float tmpOffsetX = currentOptimal[0] - halfNextSize.x;
        float tmpOffsetY = currentOptimal[1] - halfNextSize.y;
        float tmpOffsetZ = currentOptimal[2] - halfNextSize.z;

        offsetX = [tmpOffsetX, this](){ return std::max(tmpOffsetX, lowerBounds().x); };
        // offsetY = [tmpOffsetY, this](){ return std::max(tmpOffsetY, lowerBounds().y); };
        offsetY = [tmpOffsetZ, this](){ return std::max(tmpOffsetZ, lowerBounds().z); };

        scale = [nextScale](){ return nextScale; };
    }

    int HierarchicalDiscreteGraphicalModel::getXScalingFactor(float currentOptima, float halfNextSize, float scale)
    {
        int x = 0;
        while (currentOptima - halfNextSize + x * scale <  lowerBounds().x) {
            x++;
        }
        return x;
    }

    int HierarchicalDiscreteGraphicalModel::getYScalingFactor(float currentOptima, float halfNextSize, float scale)
    {
        int y = 0;
        while (currentOptima - halfNextSize - y * scale >  lowerBounds().y) {
            y++;
        }
        return y;
    }

    void HierarchicalDiscreteGraphicalModel::resetPosition()
    {
        scale = [this](){ return GLMVec3(std::fabs(config.bounds.upper.x - config.bounds.lower.x),
                                        0, // std::fabs(config.bounds.upper.y - config.bounds.lower.y),
                                        std::fabs(config.bounds.upper.z - config.bounds.lower.z)) / (float)numLabels(); };

        offsetX = [this](){ return config.bounds.lower.x; };
        // offsetY = [this](){ return config.bounds.lower.y; };
        offsetZ = [this](){ return config.bounds.lower.z; };
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

    GLMVec3 HierarchicalDiscreteGraphicalModel::lowerBounds()
    {
        return config.bounds.lower;
    }
    

    GLMVec3 HierarchicalDiscreteGraphicalModel::upperBounds()
    {
        return config.bounds.upper;
    }



} // namespace opview
