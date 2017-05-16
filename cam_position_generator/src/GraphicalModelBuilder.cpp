#include <opview/GraphicalModelBuilder.hpp>

#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/graphicalmodel/space/simplediscretespace.hxx>
#include <opengm/functions/potts.hxx>
#include <opengm/operations/adder.hxx>
#include <opengm/inference/messagepassing/messagepassing.hxx>

namespace opview {

    GraphicalModelBuilder::GraphicalModelBuilder(GLMVec3List &cams, double goalAngle, double dispersion)
    {
        this->cams = cams;
        this->vonMisesConfig = {goalAngle, dispersion};
        this->initShapes();
    }

    GraphicalModelBuilder::~GraphicalModelBuilder()
    { /*    */ }

    void GraphicalModelBuilder::initShapes()
    {
        for (size_t i = 0; i < numVariables; i++) this->variableIndices.push_back(i);
        for (size_t i = 0; i < numVariables; i++) this->shape.push_back(100);
    }

    void GraphicalModelBuilder::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        SimpleSpace space(numVariables, numLabels);
        GraphicalModelAdder model(space);

        GMExplicitFunction vonMises(shape.begin(), shape.end());
        GMSparseFunction constraints(shape.begin(), shape.end(), -10.0);
        GMSparseFunction distances(shape.begin(), shape.end(), -100.0);

        buildModel(model, vonMises, constraints, distances, centroid, normVector);

        LabelList startPoint(numVariables);
        for (int i = 0; i < numVariables; i++) {
            startPoint[i] = static_cast<LabelType>(std::rand()) / RAND_MAX + 1;
            std::cout << "startPoint[i]: " << startPoint[i] << std::endl;
        }

        //MinAlphaExpansion algorithm(model); // OpenGM error: This implementation of Alpha-Expansion supports only factors of order <= 2
        //MinAlphaBetaSwap algorithm(model); // OpenGM error: This implementation of Alpha-Beta-Swap supports only factors of order <= 2.
        //algorithm.infer();

        //ICM::Parameter para(startPoint);
        //ICM algorithm(model, para);
        //algorithm.infer();

        size_t maxSubgraphSize = 1; // works only if it is 1

        LazyFlipper::Parameter para(maxSubgraphSize, startPoint.begin(), startPoint.end());
        LazyFlipper algorithm(model, para);
        algorithm.infer();

        /*
        const std::string solver="ad3",
        const double phi = 0.3,
        onst size_t maxBlockRadius  = 50,
        const size_t maxTreeRadius = 50,
        const double pFastHeuristic = 0.9,
        const size_t maxIterations = 100000,
        const size_t stopAfterNBadIterations=10000,
        const size_t maxBlockSize = 0,
        const size_t maxTreeSize     =0,
        const int treeRuns =1
        */

        /*
        LOC::Parameter parameter(
            "ad3",
            0.5,
            1,
            1,
            0.9,
            1000,
            1000,
            0,
            0,
            1
        );
        //LOC::Parameter parameter();
        LOC algorithm(model, parameter);
        // set starting point
        // assuming startingPoint has been filled
        // with meaningful labels
        //algorithm.setStartingPoint(startPoint.begin());
        // optimize (approximately)
        algorithm.infer();
        */        

        LabelList x;
        algorithm.arg(x);
        std::cout << algorithm.value() << std::endl;
        for (size_t j = 0; j < x.size(); ++j) {
            std::cout << x[j] << ' ';
        }
        std::cout << std::endl;

    }

    void GraphicalModelBuilder::buildModel(GraphicalModelAdder &model, GMExplicitFunction &vonMises, GMSparseFunction &constraints, GMSparseFunction &distances, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        fillObjectiveFunction(vonMises, centroid, normVector);
        addFunctionTo(vonMises, model);
        
        fillConstraintFunction(constraints, distances, centroid);
        addFunctionTo(constraints, model);
        addFunctionTo(distances, model);
    }

    void GraphicalModelBuilder::fillObjectiveFunction(GMExplicitFunction &vonMises, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        #pragma omp parallel for collapse(3)
        coordinatecycles(startX, endX, startY, endY, startZ, endZ) {
            double xd = static_cast<double>(x) * 0.25 + 0.25;
            double yd = static_cast<double>(y) * 0.25 + 0.25;
            double zd = static_cast<double>(z) * 0.25 + 0.25;
            LabelType val = logVonMises(GLMVec3(xd, yd, zd), centroid, normVector, &vonMisesConfig);
            #pragma omp critical
            vonMises(xd, yd, zd) = val;
        }
    }

    void GraphicalModelBuilder::fillConstraintFunction(GMSparseFunction &constraints, GMSparseFunction &distances, GLMVec3 &centroid)
    {
        for (GLMVec3 cam : cams) {
            #pragma omp parallel for collapse(3)
            coordinatecycles(startX, endX, startY, endY, startZ, endZ) {
                double xd = static_cast<double>(x) * 0.25 + 0.25;
                double yd = static_cast<double>(y) * 0.25 + 0.25;
                double zd = static_cast<double>(z) * 0.25 + 0.25;
                addValueToConstraintFunction(constraints, GLMVec3(xd, yd, zd), cam, centroid);
            }
            LabelType vals[] = {cam.x, cam.y, cam.z};
            distances.insert(vals, 0.0);
        }
    }

    void GraphicalModelBuilder::addValueToConstraintFunction(GMSparseFunction &function, GLMVec3 point, GLMVec3 &cam, GLMVec3 &centroid)
    {
        double B = glm::distance(point, cam);
        
        GLMVec3 midPoint = (point + cam) / 2.0f;
        GLMVec3 center = GLMVec3(centroid[0], centroid[1], centroid[2]);

        double D = glm::distance(midPoint, center);

        if (B / D > 0.7f) {
            LabelType vals[] = {point.x, point.y, point.z};
            #pragma omp critical
            function.insert(vals, 1.0);
        }
    }

    void GraphicalModelBuilder::addFunctionTo(GMExplicitFunction &fun, GraphicalModelAdder &model)
    {
        auto fid = model.addFunction(fun);
        model.addFactor(fid, variableIndices.begin(), variableIndices.end());
    }

    void GraphicalModelBuilder::addFunctionTo(GMSparseFunction &fun, GraphicalModelAdder &model)
    {
        auto fid = model.addFunction(fun);
        model.addFactor(fid, variableIndices.begin(), variableIndices.end());
    }

    
    LabelType GraphicalModelBuilder::logVonMises(GLMVec3 point, GLMVec3 &centroid, GLMVec3 &normalVector, VonMisesConfigurationPtr config)
    {
        GLMVec3 v = point - centroid;
        return logVonMises(v, normalVector, config);
    }

    LabelType GraphicalModelBuilder::logVonMises(GLMVec3 &point, GLMVec3 &centroid, GLMVec3 &normalVector, VonMisesConfigurationPtr config)
    {
        GLMVec3 v = point - centroid;
        return logVonMises(v, normalVector,config);
    }

    LabelType GraphicalModelBuilder::logVonMises(GLMVec3 &v, GLMVec3 &normalVector, VonMisesConfigurationPtr config)
    { 
        double dotProduct = glm::dot(normalVector, v);
        double normProduct = glm::l2Norm(normalVector) * glm::l2Norm(v);
        double angle = dotProduct / normProduct;
 
        return logVonMises(angle, config);
    }

    LabelType GraphicalModelBuilder::logVonMises(double angle, VonMisesConfigurationPtr config)
    { 
        return std::cos(angle - config->goalAngle) * config->dispersion - std::log(2 * M_PI) - logBessel0(config->dispersion); 
    }

    LabelType GraphicalModelBuilder::logBessel0(double k)   // log of I0(x) as approximately x − 1/2 log(2 *pi * x)     https://math.stackexchange.com/questions/376758/exponential-approximation-of-the-modified-bessel-function-of-first-kind-equatio 
    { 
        double logArg = 2.0f * M_PI * k; 
        return k - 0.5f * std::log(logArg);
    }

} // namespace opview
