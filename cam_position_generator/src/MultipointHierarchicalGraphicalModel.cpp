#include <opview/MultipointHierarchicalGraphicalModel.hpp>

namespace opview {
    
    MultipointHierarchicalGraphicalModel::MultipointHierarchicalGraphicalModel(SolverGeneratorPtr solver, 
                                            OrientationHierarchicalConfiguration &config, CameraGeneralConfiguration &camConfig,
                                            std::string meshFile, GLMVec3List &cams, double goalAngle, double dispersion)
                                            : OrientationHierarchicalGraphicalModel(solver, config, camConfig, meshFile, cams, goalAngle, dispersion)
    { /*    */ }

    MultipointHierarchicalGraphicalModel::MultipointHierarchicalGraphicalModel(SolverGeneratorPtr solver, 
                                            OrientationHierarchicalConfiguration &config, CameraGeneralConfiguration &camConfig, 
                                            MeshConfiguration &meshConfig, size_t maxPoints, long double maxUncertainty, double goalAngle, double dispersion)
                                            : OrientationHierarchicalGraphicalModel(solver, config, camConfig, meshConfig.filename, meshConfig.cams, goalAngle, dispersion)
    {
        this->points = meshConfig.points;
        this->maxPoints = maxPoints;
        this->maxUncertainty = maxUncertainty;
        this->uncertainty = meshConfig.uncertainty;
        for (int p = 0; p < uncertainty.size(); p++) {
            SUM_UNCERTAINTY += uncertainty[p];
        }
    }
        
    MultipointHierarchicalGraphicalModel::~MultipointHierarchicalGraphicalModel()
    { /*    */ }


    void MultipointHierarchicalGraphicalModel::estimateBestCameraPosition(GLMVec3List &centroids, GLMVec3List &normVectors)
    {
        if (centroids.size() != normVectors.size()) {
            throw DimensionDisagreementLists(centroids.size(), normVectors.size());
        }

        this->resetPosition();

        LabelList currentOptima = {MIN_COORDINATE, MIN_COORDINATE, MIN_COORDINATE, 0.0, 0.0};
        
        for (int d = 0; d < this->getDepth(); d++) {
            std::cout << "Current depth: " << d << std::endl;
            SimpleSpace space(numVariables(), numLabels());
            GraphicalModelAdder model(space);

            this->fillModel(model, centroids, normVectors);

            auto discreteOptima = getOptimaForDiscreteSpace(currentOptima);
            AdderInferencePtr algorithm = solverGenerator()->getOptimizerAlgorithm(model, discreteOptima, numVariables());
            algorithm->infer();

            currentOptima = this->extractResults(algorithm);

            this->reduceScale(currentOptima);
        }
    }

    
    void MultipointHierarchicalGraphicalModel::fillModel(GraphicalModelAdder &model, GLMVec3List &centroids, GLMVec3List &normVectors)
    {
        GMExplicitFunction vonMises(coordinateShape.begin(), coordinateShape.end());
        GMSparseFunction visibility(shape.begin(), shape.end(), -1.0);
        GMSparseFunction projectionWeight(shape.begin(), shape.end(), -1.0);
        GMSparseFunction constraints(coordinateShape.begin(), coordinateShape.end(), 0.0);

        GMSparseFunctionList modelFunctions = {visibility, projectionWeight};
        BoostObjFunctionList evals = {
                                    boost::bind(&MultipointHierarchicalGraphicalModel::parentCallToVisibilityEstimation, this, _1, _2, _3),
                                    boost::bind(&MultipointHierarchicalGraphicalModel::parentCallToPlaneWeight, this, _1, _2, _3)
                                };
        fillSparseFunctions(modelFunctions, evals, centroids, normVectors);
        fillObjectiveFunction(vonMises, centroids, normVectors);
        fillConstraintFunction(constraints, centroids);

        addFunctionTo(vonMises, model, coordinateIndices);
        addFunctionTo(visibility, model, variableIndices);
        addFunctionTo(projectionWeight, model, variableIndices);
        addFunctionTo(constraints, model, coordinateIndices);
    }

    void MultipointHierarchicalGraphicalModel::fillSparseFunctions(GMSparseFunctionList &modelFunctions, BoostObjFunctionList &evals, GLMVec3List &centroids, GLMVec3List &normVectors) 
    {
        #pragma omp parallel for collapse(5)
        coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) { 
            orientationcycles(0, orientationLabels(), 0, orientationLabels()) {
                size_t coord[] = {(size_t)x, (size_t)y, (size_t)z, (size_t)ptc, (size_t)yaw};

                computeDistributionForList(modelFunctions, evals, coord, centroids, normVectors);
            }
        }
    }

    void MultipointHierarchicalGraphicalModel::fillObjectiveFunction(GMExplicitFunction &objFunction, GLMVec3List &centroids, GLMVec3List &normVectors)
    {
        #pragma omp parallel for collapse(3)
        coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) { 

            GLMVec3 scaledPos = scalePoint(GLMVec3(x, y, z));

            LabelType val = 0.0;
            for (int p = 0; p < centroids.size(); p++) {
                val += -logVonMises(scaledPos, centroids[p], normVectors[p]);
            }
            #pragma omp critical
            objFunction(x, y, z) = val; 
        }
    }

    void MultipointHierarchicalGraphicalModel::computeDistributionForList(GMSparseFunctionList &modelFunctions, BoostObjFunctionList &evals, size_t coord[], GLMVec3List &centroids, GLMVec3List &normVectors)
    {
        GLMVec3 scaledPos = scalePoint(GLMVec3(coord[0], coord[1], coord[2]));
        GLMVec2 scaledOri = scaleOrientation(GLMVec2(coord[3], coord[4]));

        EigVector5 pose = getPose(scaledPos, scaledOri);

        #pragma omp parallel for
        for (int f = 0; f < modelFunctions.size(); f++) {
            LabelType val = 0.0;
            for (int p = 0; p < centroids.size(); p++) {
                val += evals[f](pose, centroids[p], normVectors[p]);
            }
            #pragma omp critical
            modelFunctions[f].insert(coord, val);
        }
    }

    void MultipointHierarchicalGraphicalModel::fillConstraintFunction(GMSparseFunction &constraints, GLMVec3List &centroids)
    {
        for (GLMVec3 cam : this->getCams()) {
            #pragma omp parallel for collapse(3)
            coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) {
                GLMVec3 pos = scalePoint(GLMVec3(x, y, z));
                addValueToConstraintFunction(constraints, pos, cam, centroids, GLMVec3(x, y, z));
            }
        }
    }

    void MultipointHierarchicalGraphicalModel::addValueToConstraintFunction(GMSparseFunction &function, GLMVec3 &point, GLMVec3 &cam, GLMVec3List &centroids, GLMVec3 spacePos)
    {
        double B = glm::distance(point, cam);
        double val = 0.0;

        // #pragma omp parallel for    can not perform reduction because of floating point variable
        for (int i = 0; i < centroids.size(); i++) {
            double D = std::min(glm::distance(cam, centroids[i]), glm::distance(point, centroids[i]));
            val += (B / D <= BD_TERRESTRIAL_ARCHITECTURAL) ? -1.0 : 0.0;
        }

        LabelType coords[] = {spacePos.x, spacePos.y, spacePos.z};
        #pragma omp critical
        function.insert(coords, val);        
    }

    LabelType MultipointHierarchicalGraphicalModel::visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        return estimateForWorstPointSeen(pose, boost::bind(&MultipointHierarchicalGraphicalModel::parentCallToVisibilityEstimation, this, _1, _2, _3));
    }

    LabelType MultipointHierarchicalGraphicalModel::imagePlaneWeight(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        return estimateForWorstPointSeen(pose, boost::bind(&MultipointHierarchicalGraphicalModel::parentCallToPlaneWeight, this, _1, _2, _3));
    }

    double MultipointHierarchicalGraphicalModel::estimateForWorstPointSeen(EigVector5 &pose, BoostObjFunction function)
    {
        DoubleList uncertaintyList;
        #pragma omp parallel for
        for (int p = 0; p < points.size(); p++) {
            if (uncertainty[p] >= this->maxUncertainty) {    // TODO look at definition of UNCERTAINTY_THRESHOLD
                double normalizedWeight = computeWeightForPoint(p);     // more the uncertainty is high more important it is seen
                LabelType local_val = function(pose, points[p], normals[p]);

                #pragma omp critical
                uncertaintyList.push_back(local_val * normalizedWeight);
            }
        }

        std::sort(uncertaintyList.rbegin(), uncertaintyList.rend());

        LabelType val = 0.0;
        for (int i = 0; i < std::min(this->maxPoints, uncertaintyList.size()); i++) {
            val += uncertaintyList[i];
        }
        return val;
    }

    double MultipointHierarchicalGraphicalModel::computeWeightForPoint(int pointIndex)
    {
        return uncertainty[pointIndex] / SUM_UNCERTAINTY;
    }

    // private utils, used to indirectly call the parent and cheat boost
    LabelType MultipointHierarchicalGraphicalModel::parentCallToVisibilityEstimation(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        return super::visibilityDistribution(pose, centroid, normalVector);
    }

    LabelType MultipointHierarchicalGraphicalModel::parentCallToPlaneWeight(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        return super::imagePlaneWeight(pose, centroid, normalVector);
    }

    void MultipointHierarchicalGraphicalModel::updateMeshInfo(int pointIndex, GLMVec3 point, GLMVec3 normal, double uncertainty)
    {
        this->points[pointIndex] = point;
        updateMeshInfo(pointIndex, normal, uncertainty);
    }

    void MultipointHierarchicalGraphicalModel::updateMeshInfo(int pointIndex, GLMVec3 normal, double uncertainty)
    {
        this->normals[pointIndex] = normal;
        updateMeshInfo(pointIndex, uncertainty);
    }

    void MultipointHierarchicalGraphicalModel::updateMeshInfo(int pointIndex, double uncertainty)
    {
        SUM_UNCERTAINTY += this->uncertainty[pointIndex] - uncertainty;
        this->uncertainty[pointIndex] = uncertainty;
    }

    void MultipointHierarchicalGraphicalModel::addPoint(GLMVec3 point, GLMVec3 normal, double uncertainty)
    {
        this->points.push_back(point);
        this->normals.push_back(normal);
        this->uncertainty.push_back(uncertainty);
        SUM_UNCERTAINTY += uncertainty;
    }

} // namespace opview
