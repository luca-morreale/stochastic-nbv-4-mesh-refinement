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
        this->normals = meshConfig.normals;
        this->uncertainty = meshConfig.uncertainty;

        this->maxPoints = maxPoints;
        this->maxUncertainty = maxUncertainty;

        getLogger()->resetFile("multipoint.json");
        
        precomputeSumUncertainty();
        setupWorstPoints();
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
            SimpleSpace space(shape.begin(), shape.end());
            GraphicalModelAdder model(space);

            this->fillModel(model, centroids, normVectors);

            auto discreteOptima = getOptimaForDiscreteSpace(currentOptima);
            AdderInferencePtr algorithm = solverGenerator()->getOptimizerAlgorithm(model, discreteOptima, numVariables());
            algorithm->infer();

            currentOptima = this->extractResults(algorithm);

            this->reduceScale(currentOptima, d+1);
        }
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
            fillExplicitOrientationFunction(visibility, boost::bind(&MultipointHierarchicalGraphicalModel::parentCallToVisibilityEstimation, this, _1, _2, _3), centroids, normVectors);
            #pragma omp section
            fillExplicitOrientationFunction(projectionWeight, boost::bind(&MultipointHierarchicalGraphicalModel::parentCallToPlaneWeight, this, _1, _2, _3), centroids, normVectors);
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
        #pragma omp parallel for collapse(3)
        coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) { 

            GLMVec3 scaledPos = scalePoint(GLMVec3(x, y, z));

            LabelType val = 0.0;
            for (int p = 0; p < centroids.size(); p++) {
                val += -super::logVonMisesWrapper(scaledPos, centroids[p], normVectors[p]);
            }
            orientationcycles(0, orientationLabels(), 0, orientationLabels()) {
                #pragma omp critical
                objFunction(x, y, z, ptc, yaw) = val;
            }
        }
    }

    void MultipointHierarchicalGraphicalModel::computeDistribution(GMExplicitFunction &modelFunction, BoostObjFunction &evals, size_t coord[], GLMVec3List &centroids, GLMVec3List &normVectors)
    {
        GLMVec3 scaledPos = scalePoint(GLMVec3(coord[0], coord[1], coord[2]));
        GLMVec2 scaledOri = scaleOrientation(GLMVec2(coord[3], coord[4]));

        EigVector5 pose = getPose(scaledPos, scaledOri);

        LabelType val = 0.0;
        for (int p = 0; p < centroids.size(); p++) {
            val += evals(pose, centroids[p], normVectors[p]);
        }
        modelFunction(coord) = val;
    }

    void MultipointHierarchicalGraphicalModel::fillConstraintFunction(GMExplicitFunction &constraints, GLMVec3List &centroids)
    {
        for (GLMVec3 cam : this->getCams()) {
            #pragma omp parallel for collapse(3)
            coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) {
                GLMVec3 pos = scalePoint(GLMVec3(x, y, z));
                addValueToConstraintFunction(constraints, pos, cam, centroids, GLMVec3(x, y, z));
            }
        }
    }

    void MultipointHierarchicalGraphicalModel::addValueToConstraintFunction(GMExplicitFunction &function, GLMVec3 &point, GLMVec3 &cam, GLMVec3List &centroids, GLMVec3 spacePos)
    {
        double B = glm::distance(point, cam);
        double val = 0.0;

        // #pragma omp parallel for    can not perform reduction because of floating point variable
        #pragma omp parallel for
        for (int i = 0; i < centroids.size(); i++) {
            double D = std::min(glm::distance(cam, centroids[i]), glm::distance(point, centroids[i]));
            LabelType local_val = (B / D <= BD_TERRESTRIAL_ARCHITECTURAL) ? -1.0 : 0.0;
            #pragma omp critical
            val += local_val;
        }

        orientationcycles(0, orientationLabels(), 0, orientationLabels()) {
            #pragma omp critical
            function(spacePos.x, spacePos.y, spacePos.z, ptc, yaw) = val;
        }
    }

    LabelType MultipointHierarchicalGraphicalModel::logVonMisesWrapper(GLMVec3 &point, GLMVec3 &centroid, GLMVec3 &normal)
    {
        EigVector5 pose;
        pose << point.x, point.y, point.z, 0.0, 0.0;
        return estimateForWorstPointSeen(pose, boost::bind(&MultipointHierarchicalGraphicalModel::parentCallToVonMisesWrapper, this, _1, _2, _3))
                + parentCallToVonMisesWrapper(pose, centroid, normal);
    }

    LabelType MultipointHierarchicalGraphicalModel::visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        return estimateForWorstPointSeen(pose, boost::bind(&MultipointHierarchicalGraphicalModel::parentCallToVisibilityEstimation, this, _1, _2, _3))
                + parentCallToVisibilityEstimation(pose, centroid, normalVector);
    }

    LabelType MultipointHierarchicalGraphicalModel::imagePlaneWeight(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        return estimateForWorstPointSeen(pose, boost::bind(&MultipointHierarchicalGraphicalModel::parentCallToPlaneWeight, this, _1, _2, _3))
                + parentCallToPlaneWeight(pose, centroid, normalVector);
    }

    double MultipointHierarchicalGraphicalModel::estimateForWorstPointSeen(EigVector5 &pose, BoostObjFunction function)
    {
        LabelType val = 0.0;
        #pragma omp parallel for
        for (int i = 0; i < worstPointsList.size(); i++) {
            DoubleIntPair el = worstPointsList[i];
            double normalizedWeight = computeWeightForPoint(el.second);     // more the uncertainty is high more important it is seen
            LabelType local_val = function(pose, points[el.second], normals[el.second]);
            
            #pragma omp critical
            val += local_val * normalizedWeight;
        }
        
        return val;
    }

    double MultipointHierarchicalGraphicalModel::computeWeightForPoint(int pointIndex)
    {
        return (double)uncertainty[pointIndex] / (double)SUM_UNCERTAINTY;
    }

    // private utils, used to indirectly call the parent and cheat boost
    LabelType MultipointHierarchicalGraphicalModel::parentCallToVonMisesWrapper(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        GLMVec3 scaledPos = GLMVec3(pose[0], pose[1], pose[2]);
        return -super::logVonMisesWrapper(scaledPos, centroid, normalVector);
    }

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
        updateWorstPoints(pointIndex, uncertainty);
    }

    void MultipointHierarchicalGraphicalModel::addPoint(GLMVec3 point, GLMVec3 normal, double uncertainty)
    {
        this->points.push_back(point);
        this->normals.push_back(normal);
        this->uncertainty.push_back(uncertainty);
        SUM_UNCERTAINTY += uncertainty;
    }

    void MultipointHierarchicalGraphicalModel::precomputeSumUncertainty()
    {
        SUM_UNCERTAINTY = 0.0;
        for (int p = 0; p < this->uncertainty.size(); p++) {
            SUM_UNCERTAINTY += uncertainty[p];
        }
    }

    void MultipointHierarchicalGraphicalModel::setupWorstPoints()
    {
        worstPointsList.clear();
        #pragma omp parallel for        // not much to gain using parallelism
        for (int p = 0; p < points.size(); p++) {
            if (uncertainty[p] > this->maxUncertainty) {    // NOTE look at definition of UNCERTAINTY_THRESHOLD
                #pragma omp critical
                worstPointsList.push_back(std::make_pair(uncertainty[p], p));
            }
        }
        retainWorst();
    }

    void MultipointHierarchicalGraphicalModel::updateWorstPoints(int index, long double uncertainty)
    {
        worstPointsList.push_back(std::make_pair(uncertainty, index));
        retainWorst();
    }

    void MultipointHierarchicalGraphicalModel::retainWorst()
    {
        std::sort(worstPointsList.rbegin(), worstPointsList.rend());
        int eraseFrom = std::min(this->maxPoints, worstPointsList.size());
        worstPointsList.erase(worstPointsList.begin() + eraseFrom, worstPointsList.end());
    }

    DoubleIntList MultipointHierarchicalGraphicalModel::getWorstPointsList()
    {
        return worstPointsList;
    }

    GLMVec3List MultipointHierarchicalGraphicalModel::getPoints()
    {
        return points;
    }
    GLMVec3List MultipointHierarchicalGraphicalModel::getNormals()
    {
        return normals;
    }
    DoubleList MultipointHierarchicalGraphicalModel::getUncertainties()
    {
        return uncertainty;
    }

} // namespace opview
