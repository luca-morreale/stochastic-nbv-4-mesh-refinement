#include <opview/HeavyOrientationHierarchicalGraphicalModel.hpp>

namespace opview {

    HeavyOrientationHierarchicalGraphicalModel::HeavyOrientationHierarchicalGraphicalModel(SolverGeneratorPtr solver, 
                                OrientationHierarchicalConfiguration &config, CameraGeneralConfiguration &camConfig, std::string meshFile, 
                                GLMVec3List &cams, size_t spaceBlock, size_t offsetBlock, double goalAngle, double dispersion)
                                : OrientationHierarchicalGraphicalModel(solver, config, camConfig, meshFile, cams, goalAngle, dispersion)
    {
        this->spaceBlock = spaceBlock;      // NOTE has to be a power of 2
        this->offsetBlock = offsetBlock;
        this->initShapes();
        this->getLogger()->resetFile("heavy_orientation.json");
    }

    HeavyOrientationHierarchicalGraphicalModel::~HeavyOrientationHierarchicalGraphicalModel()
    { /*    */ }

    void HeavyOrientationHierarchicalGraphicalModel::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        size_t iterations = std::pow(spaceBlock, 3);
        for (size_t i = offsetBlock; i < iterations; i++) {
            std::cout << "Current block: " << i << std::endl;
            changeBlockSettings(i);

            this->resetPosition();
            LabelList currentOptima = {MIN_COORDINATE, MIN_COORDINATE, MIN_COORDINATE, 0, 0};

            SimpleSpace space(shape.begin(), shape.end());
            GraphicalModelAdder model(space);

            this->fillModel(model, centroid, normVector);

            auto discreteOptima = getOptimaForDiscreteSpace(currentOptima);
            AdderInferencePtr algorithm = solverGenerator()->getOptimizerAlgorithm(model, discreteOptima, numVariables());
            algorithm->infer();

            currentOptima = this->extractResults(algorithm);
        }
    }

    void HeavyOrientationHierarchicalGraphicalModel::changeBlockSettings(size_t blockNumber)
    {
        size_t mask = spaceBlock-1;
        size_t shift = popcount(mask);

        float blockX = std::fabs(MIN_COORDINATE) / (float)spaceBlock;
        float blockY = std::fabs(-0.5f) / (float)spaceBlock;
        float blockZ = std::fabs(MIN_COORDINATE) / (float)spaceBlock;

        float tmpOffsetX = MIN_COORDINATE + (blockNumber & mask) * blockX;
        mask <<= shift;
        float tmpOffsetY = -0.5 + ((blockNumber & mask) % spaceBlock) * blockY;
        mask <<= shift;
        float tmpOffsetZ = MIN_COORDINATE + ((blockNumber & mask) % spaceBlock) * blockZ;

        std::cout << tmpOffsetX << " " << tmpOffsetY << " " << tmpOffsetZ << std::endl;

        offsetX = [tmpOffsetX](){ return tmpOffsetX; };
        offsetY = [tmpOffsetY](){ return tmpOffsetY; };
        offsetZ = [tmpOffsetZ](){ return tmpOffsetZ; };
    }

    size_t HeavyOrientationHierarchicalGraphicalModel::popcount(size_t n) 
    {
        std::bitset<sizeof(size_t) * CHAR_BIT> b(n);
        return b.count();
    }

    size_t HeavyOrientationHierarchicalGraphicalModel::numLabels()
    {
        return (size_t)(std::pow(super::numLabels(), getDepth()) / (double)spaceBlock);
    }

} // namespace opview
