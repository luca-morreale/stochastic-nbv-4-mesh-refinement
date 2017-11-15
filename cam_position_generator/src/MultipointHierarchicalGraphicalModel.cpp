#include <opview/MultipointHierarchicalGraphicalModel.hpp>

namespace opview {
    
    MultipointHierarchicalGraphicalModel::MultipointHierarchicalGraphicalModel(SolverGeneratorPtr solver, 
                                            OrientationHierarchicalConfiguration &config, CameraGeneralConfiguration &camConfig,
                                            std::string meshFile, GLMVec3List &cams, double goalAngle, double dispersion)
                                            : OrientationHierarchicalGraphicalModel(solver, config, camConfig, meshFile, cams, goalAngle, dispersion)
    {
        getLogger()->resetFile("multipoint.json");
    }
        
    MultipointHierarchicalGraphicalModel::~MultipointHierarchicalGraphicalModel()
    { /*    */ }


    LabelList MultipointHierarchicalGraphicalModel::estimateBestCameraPosition(GLMVec3List &centroids, GLMVec3List &normVectors)
    {
        if (centroids.size() != normVectors.size()) {
            throw DimensionDisagreementLists(centroids.size(), normVectors.size());
        }

        this->resetPosition();

        LabelList currentOptima = {lowerBounds().x, lowerBounds().y, lowerBounds().z, 0.0, 0.0};
        
        for (int d = 0; d < this->getDepth(); d++) {
            std::cout << "Current depth: " << d << std::endl;
            SimpleSpace space(shape.begin(), shape.end());
            GraphicalModelAdder model(space);

            this->fillModel(model, centroids, normVectors);

            auto discreteOptima = getOptimaForDiscreteSpace(currentOptima);
            AdderInferencePtr algorithm = solverGenerator()->getOptimizerAlgorithm(model, discreteOptima, numVariables());
            algorithm->infer();

            currentOptima = this->extractResults(algorithm);

            this->reduceScale(currentOptima, d+1);
        }
        
        currentOptima[3] = deg2rad(currentOptima[3]);
        currentOptima[4] = deg2rad(currentOptima[4]);
        return currentOptima;
    }

    
    void MultipointHierarchicalGraphicalModel::fillModel(GraphicalModelAdder &model, GLMVec3List &centroids, GLMVec3List &normVectors)
    {
        GMExplicitFunction vonMises(shape.begin(), shape.end());
        GMExplicitFunction visibility(shape.begin(), shape.end());
        GMExplicitFunction projectionWeight(shape.begin(), shape.end());
        GMExplicitFunction constraints(shape.begin(), shape.end());

        #pragma omp parallel sections 
        {
            #pragma omp section
            fillExplicitOrientationFunction(visibility, boost::bind(&Formulation::visibilityDistribution, _1, _2, _3, _4, _5), centroids, normVectors);
            #pragma omp section
            fillExplicitOrientationFunction(projectionWeight, boost::bind(&Formulation::imageProjectionDistribution, _1, _2, _3, _4, _5), centroids, normVectors);
            #pragma omp section
            fillObjectiveFunction(vonMises, centroids, normVectors);
            #pragma omp section
            fillConstraintFunction(constraints, centroids);
        }

        addFunctionTo(vonMises, model, variableIndices);
        addFunctionTo(visibility, model, variableIndices);
        addFunctionTo(projectionWeight, model, variableIndices);
        addFunctionTo(constraints, model, variableIndices);
    }

    void MultipointHierarchicalGraphicalModel::fillExplicitOrientationFunction(GMExplicitFunction &modelFunction, BoostObjFunction evals, GLMVec3List &centroids, GLMVec3List &normVectors) 
    {
        #pragma omp parallel for collapse(5)
        coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) { 
            orientationcycles(0, orientationLabels(), 0, orientationLabels()) {
                size_t coord[] = {(size_t)x, (size_t)y, (size_t)z, (size_t)ptc, (size_t)yaw};
                computeDistribution(modelFunction, evals, coord, centroids, normVectors);
            }
        }
    }

    void MultipointHierarchicalGraphicalModel::fillObjectiveFunction(GMExplicitFunction &objFunction, GLMVec3List &centroids, GLMVec3List &normVectors)
    {
        VonMisesConfigurationPtr vmConfig = vonMisesConfiguration();
        #pragma omp parallel for collapse(3)
        coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) { 

            GLMVec3 scaledPos = scalePoint(GLMVec3(x, y, z));

            LabelType val = 0.0;

            #pragma omp parallel for reduction(+:val)
            for (int p = 0; p < centroids.size(); p++) {
                val += Formulation::logVonMisesWrapper(scaledPos, centroids[p], normVectors[p], *vmConfig);
            }

            fillExplicitFunction(objFunction, val, x, y, z);
        }
    }

    void MultipointHierarchicalGraphicalModel::computeDistribution(GMExplicitFunction &modelFunction, BoostObjFunction &evals, size_t coord[], GLMVec3List &centroids, GLMVec3List &normVectors)
    {
        CameraGeneralConfigPtr camConfig = getCamConfig();
        TreePtr tree = getTree();

        GLMVec3 scaledPos = scalePoint(GLMVec3(coord[0], coord[1], coord[2]));
        GLMVec2 scaledOri = scaleOrientation(GLMVec2(coord[3], coord[4]));

        EigVector5 pose = getPose(scaledPos, scaledOri);

        LabelType val = 0.0;

        #pragma omp parallel for reduction(+:val)
        for (int p = 0; p < centroids.size(); p++) {
            val += evals(pose, centroids[p], normVectors[p], *camConfig, tree);
        }
        modelFunction(coord) = val;
    }

    void MultipointHierarchicalGraphicalModel::fillConstraintFunction(GMExplicitFunction &constraints, GLMVec3List &centroids)
    {
        GLMVec3List cams = getCams();

        #pragma omp parallel for collapse(3)
        coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) {
            size_t coords[] = {(size_t)x, (size_t)y, (size_t)z};
            GLMVec3 pos = scalePoint(GLMVec3(x, y, z));
            
            LabelType val = 0.0;

            #pragma omp parallel for reduction(+:val)
            for (int i = 0; i < centroids.size(); i++) {
                val += Formulation::computeBDConstraint(pos, centroids[i], cams);
            }

            fillExplicitFunction(constraints, val, x, y, z);
        }
    }

    void MultipointHierarchicalGraphicalModel::fillExplicitFunction(GMExplicitFunction &function, LabelType val, size_t x, size_t y, size_t z)
    {
        orientationcycles(0, orientationLabels(), 0, orientationLabels()) {
            #pragma omp critical
            function(x, y, z, ptc, yaw) = val;
        }
    }

} // namespace opview
