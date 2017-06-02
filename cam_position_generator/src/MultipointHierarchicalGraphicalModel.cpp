#include <opview/MultipointHierarchicalGraphicalModel.hpp>

namespace opview {
    
    MultipointHierarchicalGraphicalModel::MultipointHierarchicalGraphicalModel(SolverGeneratorPtr solver, 
                                            OrientationHierarchicalConfiguration &config, CameraGeneralConfiguration &camConfig,
                                            std::string meshFile, GLMVec3List &cams, double goalAngle, double dispersion)
                                            : OrientationHierarchicalGraphicalModel(solver, config, camConfig, meshFile, cams, goalAngle, dispersion)
    { /*    */ }
        
    MultipointHierarchicalGraphicalModel::~MultipointHierarchicalGraphicalModel()
    { /*    */ }


    void MultipointHierarchicalGraphicalModel::estimateBestCameraPosition(GLMVec3List &centroids, GLMVec3List &normVectors)
    {
        if (centroids.size() != normVectors.size()) {
            throw DimensionDiagreementLists(centroids.size(), normVectors.size());
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
        GMSparseFunction vonMises(shape.begin(), shape.end(), 0.0);
        GMSparseFunction constraints(shape.begin(), shape.end(), 0.0);
        GMSparseFunction distances(shape.begin(), shape.end(), 0.0);

        fillObjectiveFunction(vonMises, centroids, normVectors);
        fillConstraintFunction(constraints, distances, centroids);

        addFunctionTo(vonMises, model, variableIndices);
        addFunctionTo(constraints, model, variableIndices);
        addFunctionTo(distances, model, variableIndices);
    }

    void MultipointHierarchicalGraphicalModel::fillObjectiveFunction(GMSparseFunction &vonMises, GLMVec3List &centroids, GLMVec3List &normVectors)
    {

        #pragma omp parallel for collapse(5)
        coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) { 
            orientationcycles(0, orientationLabels(), 0, orientationLabels()) { 
                GLMVec3 scaledPos = scalePoint(GLMVec3(x, y, z));
                GLMVec2 scaledOri = scaleOrientation(GLMVec2(ptc, yaw));

                EigVector5 pose = getPose(scaledPos, scaledOri);

                LabelType val = computeObjectiveValueForList(pose, centroids, normVectors);

                size_t coord[] = {(size_t)x, (size_t)y, (size_t)z, (size_t)ptc, (size_t)yaw};
                
                #pragma omp critical
                vonMises.insert(coord, val);
            }
        }
    }

    LabelType MultipointHierarchicalGraphicalModel::computeObjectiveValueForList(EigVector5 &pose, GLMVec3List &centroids, GLMVec3List &normVectors)
    {
        LabelType val = 0.0;
        #pragma omp parallel for    // can not perform reduction because of floating point variable
        for (int i = 0; i < centroids.size(); i++) {
            LabelType local_val = computeObjectiveFunction(pose, centroids[i], normVectors[i]);
            #pragma omp critical
            val += local_val;
        }
        return val;
    }

    void MultipointHierarchicalGraphicalModel::fillConstraintFunction(GMSparseFunction &constraints, GMSparseFunction &distances, GLMVec3List &centroids)
    {
        for (GLMVec3 cam : this->getCams()) {
            #pragma omp parallel for collapse(3)
            coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) {
                GLMVec3 pos = scalePoint(GLMVec3(x, y, z));

                addValueToConstraintFunction(constraints, pos, cam, centroids, GLMVec3(x, y, z));
            }
            addCameraPointConstrain(distances, cam);
        }
    }

    void MultipointHierarchicalGraphicalModel::addValueToConstraintFunction(GMSparseFunction &function, GLMVec3 &point, GLMVec3 &cam, GLMVec3List &centroids, GLMVec3 spacePos)
    {
        double B = glm::distance(point, cam);
        double val = 0;

        #pragma omp parallel for reduction (+:val)
        for (int i = 0; i < centroids.size(); i++) {
            double D = std::min(glm::distance(cam, centroids[i]), glm::distance(point, centroids[i]));
            val += (B / D <= BD_TERRESTRIAL_ARCHITECTURAL) ? -1.0 : 0.0;
        }

        LabelType coords[] = {spacePos.x, spacePos.y, spacePos.z};
        function.insert(coords, val);        
    }

} // namespace opview
