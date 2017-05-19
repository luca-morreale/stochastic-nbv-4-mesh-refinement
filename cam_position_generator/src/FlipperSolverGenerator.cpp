#include <opview/FlipperSolverGenerator.hpp>

namespace opview {
    
    AdderInferencePtr FlipperSolverGenerator::getOptimizerAlgorithm(GraphicalModelAdder &model, size_t numVariables)
    {
        size_t maxSubgraphSize = 1; // works only if it is 1

        LabelList startPoint = genStartPoint(numVariables);
        LazyFlipperParameter para(maxSubgraphSize, startPoint.begin(), startPoint.end());
        LazyFlipperPtr algorithm = new LazyFlipper(model, para);
        return algorithm;
    }
    
    MultiplierInferencePtr FlipperSolverGenerator::getOptimizerAlgorithm(GraphicalModelMultiplier &model, size_t numVariables)
    {
        size_t maxSubgraphSize = 1; // works only if it is 1

        LabelList startPoint = genStartPoint(numVariables);
        LazyFlipperParameter para(maxSubgraphSize, startPoint.begin(), startPoint.end());
        //LazyFlipperPtr algorithm = new LazyFlipper(model, para);

        return NULL;
    }

} // namespace opview
