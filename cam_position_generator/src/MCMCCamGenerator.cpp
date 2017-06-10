#include <opview/MCMCCamGenerator.hpp>


namespace opview {
    
    MCMCCamGenerator::MCMCCamGenerator(MultiBruteForceSolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config, 
                                            CameraGeneralConfiguration &camConfig, std::string meshFile, GLMVec3List &cams, 
                                            MCConfiguration &mcConfig, double goalAngle, double dispersion) 
                                            // : OrientationHierarchicalGraphicalModel(SolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config,
                                            //         CameraGeneralConfiguration &camConfig, std::string meshFile, 
                                            //         GLMVec3List &cams, double goalAngle=55, double dispersion=5);
                                            : OrientationHierarchicalGraphicalModel((SolverGeneratorPtr)solver, config, camConfig, meshFile, cams, goalAngle, dispersion)
    {
        this->sampler = new MCMCSamplerGenerator();
        this->mcConfig = mcConfig;

        this->initLambdas();
    }

    void MCMCCamGenerator::initLambdas()
    {
        this->uniformPointGetter = [this](OrderedStates &currentOptima) 
                { 
                    return sampler->getUniformSamples({std::make_pair(offsetX(), 2.0), std::make_pair(offsetY(), 2.0), std::make_pair(offsetZ(), 2.0)}, PARTICLES); 
                };
        this->resamplingPointGetter = [this](OrderedStates &currentOptima)
                { 
                    GLMVec3List centers = getCentersFromOptima(currentOptima);
                    DoubleList weights = getWeightsFromOptima(currentOptima);
                    return sampler->getWeightedSamples(centers, weights, PARTICLES); 
                };
    }

    MCMCCamGenerator::~MCMCCamGenerator()
    {
        delete sampler;
    }
    
    void MCMCCamGenerator::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        OrderedStates currentOptima = uniformMCStep(centroid, normVector);

        for (int d = 0; d < 10; d++) {
            currentOptima = resamplingMCStep(centroid, normVector, currentOptima);
        }

    }

    OrderedStates MCMCCamGenerator::uniformMCStep(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        SimpleSpace space(numVariables(), numLabels());
        GraphicalModelAdder model(space);

        OrderedStates emptyStates;
        generalStep(model, centroid, normVector, emptyStates, uniformPointGetter);

        AdderInferencePtr algorithm = solverGenerator()->getOptimizerAlgorithm(model, LabelList(), numVariables());
        ((MultiBruteforcePtr)algorithm)->setLimitQueue((size_t) (0.1*(double)PARTICLES));
        algorithm->infer();

        return this->extractBestResults(algorithm);
    }

    OrderedStates MCMCCamGenerator::resamplingMCStep(GLMVec3 &centroid, GLMVec3 &normVector, OrderedStates &currentOptima)
    {
        SimpleSpace space(numVariables(), numLabels());
        GraphicalModelAdder model(space);

        generalStep(model, centroid, normVector, currentOptima, resamplingPointGetter);

        AdderInferencePtr algorithm = solverGenerator()->getOptimizerAlgorithm(model, LabelList(), numVariables());
        ((MultiBruteforcePtr)algorithm)->setLimitQueue((size_t) (0.1*(double)PARTICLES));
        algorithm->infer();

        return this->extractBestResults(algorithm);
    }

    void MCMCCamGenerator::generalStep(GraphicalModelAdder &model, GLMVec3 &centroid, GLMVec3 &normVector, OrderedStates &currentOptima, LambdaGLMPointsList &getPoints)
    {
        GMSparseFunction vonMises(shape.begin(), shape.end(), 0.0);
        GMSparseFunction visibility(shape.begin(), shape.end(), -1.0);
        GMSparseFunction projectionWeight(shape.begin(), shape.end(), 0.0);
        GMSparseFunction constraints(shape.begin(), shape.end(), 0.0);
        
        GLMVec3List points = getPoints(currentOptima);

        std::vector<std::map<double, int>> mapping = getPointMapping(points);

        #pragma omp parallel for
        for (int p = 0; p < points.size(); p++) {
            GLMVec3 glmPoint = GLMVec3(points[p].x, points[p].y, points[p].z);

            orientationcycles(0, orientationLabels(), 0, orientationLabels()) {
                size_t coord[] = {(size_t)mapping[0][points[p].x], (size_t)mapping[1][points[p].y], (size_t)mapping[2][points[p].z], (size_t)ptc, (size_t)yaw};
                
                GLMVec2 scaledOri = scaleOrientation(GLMVec2(ptc, yaw));

                EigVector5 pose = getPose(glmPoint, scaledOri);
                
                LabelType val = estimateObjDistribution(pose, centroid, normVector);
                
                #pragma omp critical
                vonMises.insert(coord, val);

                val = imagePlaneWeight(pose, centroid, normVector);
                #pragma omp critical
                projectionWeight.insert(coord, val);

                for (GLMVec3 cam : getCams()) {
                    addValueToConstraintFunction(constraints, glmPoint, cam, centroid, coord);
                }
            }
        }

        addFunctionTo(vonMises, model, variableIndices);
        addFunctionTo(projectionWeight, model, variableIndices);
        addFunctionTo(constraints, model, variableIndices);
    }
    

    OrderedStates MCMCCamGenerator::extractBestResults(AdderInferencePtr algorithm)
    {
        OrderedStates x;
        ((MultiBruteforcePtr)algorithm)->argAll(x);

        // std::cout << "Value obtained: " << algorithm->value() << std::endl;

        // GLMVec3 realOptima = scalePoint(GLMVec3(x[0], x[1], x[2]));
        // x[0] = realOptima.x;
        // x[1] = realOptima.y;
        // x[2] = realOptima.z;

        // GLMVec2 orientOptima = scaleOrientation(GLMVec2(x[3], x[4]));
        // x[3] = orientOptima.x;
        // x[4] = orientOptima.y;

        // std::cout << "Optimal solution: " << x[0] << ' ' << x[1] << ' ' << x[2] << ' ';
        // std::cout << x[3] << ' ' << x[4] << std::endl << std::endl;

        return x;
    }

    DoubleIntMapList MCMCCamGenerator::getPointMapping(GLMVec3List &points)
    {
        DoubleIntMapList mappingCoordinates;
        DoubleSetList sets;

        for (int p = 0; p < points.size(); p++) {
            for (int i = 0; i < 3; i++) {
                sets[i].insert(points[p][i]);
            }
        }

        #pragma omp parallel for 
        for (int s = 0; s < sets.size(); s++) {
            DoubleSetIterator it = sets[s].begin();
            mappingCoordinates.push_back(std::map<double, int>());

            for (int p = 0; p < sets[s].size(); p++) {
                mappingCoordinates[p][(*it)] = p;
            }
        }

        return mappingCoordinates;
    }

    
    GLMVec3List MCMCCamGenerator::getCentersFromOptima(OrderedStates currentOptima) // not by refernce otherwise changes also the original
    {
        GLMVec3List points;
        while(!currentOptima.empty()) {
            DoubleDoubleListPair solution = currentOptima.top();
            DoubleList point = solution.second;

            GLMVec3 pointSpace = scalePoint(GLMVec3(point[0], point[1], point[2]));
            points.push_back(pointSpace);

            currentOptima.pop();
        }


    }

    DoubleList MCMCCamGenerator::getWeightsFromOptima(OrderedStates currentOptima) // not by refernce otherwise changes also the original
    {
        DoubleList weights;
        while(!currentOptima.empty()) {
            DoubleDoubleListPair solution = currentOptima.top();
            weights.push_back(solution.first);

            currentOptima.pop();
        }
        return weights;
    }

}
