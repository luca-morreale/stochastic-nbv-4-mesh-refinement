#include <opview/MCMCCamGenerator.hpp>


namespace opview {
    
    MCMCCamGenerator::MCMCCamGenerator(MultiBruteForceSolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config, 
                                            CameraGeneralConfiguration &camConfig, std::string meshFile, GLMVec3List &cams, 
                                            MCConfiguration &mcConfig, double goalAngle, double dispersion) 
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
                    return sampler->getUniformSamples({std::make_pair(offsetX(), 5.0f), std::make_pair(offsetY(), 5.0f), std::make_pair(offsetZ(), 5.0f)}, mcConfig.particles); 
                };
        this->resamplingPointGetter = [this](OrderedStates &currentOptima)
                { 
                    GLMVec3List centers = getCentersFromOptima(currentOptima);
                    DoubleList weights = getWeightsFromOptima(currentOptima);
                    return sampler->getWeightedSamples(centers, weights, mcConfig.particles); 
                };
    }

    MCMCCamGenerator::~MCMCCamGenerator()
    {
        delete sampler;
    }
    
    void MCMCCamGenerator::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        std::cout << "doing uniform step" << std::endl;
        OrderedStates currentOptima = uniformMCStep(centroid, normVector);

        std::cout << "starting with resampling steps" << std::endl;        
        for (int d = 0; d < mcConfig.resamplingNum; d++) {
            currentOptima = resamplingMCStep(centroid, normVector, currentOptima);
        }

    }

    OrderedStates MCMCCamGenerator::uniformMCStep(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        SimpleSpace space(numVariables(), mcConfig.particles);  // if got wrong numLabels everything crashes
        GraphicalModelAdder model(space);

        OrderedStates emptyStates;
        generalStep(model, centroid, normVector, emptyStates, uniformPointGetter);
        std::cout << "infer now\n";
        AdderInferencePtr algorithm = solverGenerator()->getOptimizerAlgorithm(model, LabelList(), numVariables());
        ((MultiBruteforcePtr)algorithm)->setLimitQueue((size_t) (0.1*(double)mcConfig.particles));
        algorithm->infer();

        return this->extractBestResults(algorithm);
    }

    OrderedStates MCMCCamGenerator::resamplingMCStep(GLMVec3 &centroid, GLMVec3 &normVector, OrderedStates &currentOptima)
    {
        SimpleSpace space(numVariables(), mcConfig.particles);  // if got wrong numLabels everything crashes
        GraphicalModelAdder model(space);

        generalStep(model, centroid, normVector, currentOptima, resamplingPointGetter);
        std::cout << "infer now\n";
        AdderInferencePtr algorithm = solverGenerator()->getOptimizerAlgorithm(model, LabelList(), numVariables());
        ((MultiBruteforcePtr)algorithm)->setLimitQueue((size_t) (0.1*(double)mcConfig.particles));
        algorithm->infer();

        return this->extractBestResults(algorithm);
    }

    void MCMCCamGenerator::generalStep(GraphicalModelAdder &model, GLMVec3 &centroid, GLMVec3 &normVector, OrderedStates &currentOptima, LambdaGLMPointsList &getPoints)
    {
        std::cout << "getting points" << std::endl;
        GLMVec3List points = getPoints(currentOptima);
        std::cout << "got " << points.size() << std::endl;
        DoubleIntMapList mapping = getPointMapping(points);
        std::cout << "got mapping"<<std::endl;

        // if got wrong numLabels everything crashes
        SizeTList shape_coords = { mapping[0].size(), mapping[1].size(), mapping[2].size() };
        SizeTList shape = { mapping[0].size(), mapping[1].size(), mapping[2].size(), orientationLabels(), orientationLabels() };

        GMExplicitFunction vonMises(shape_coords.begin(), shape_coords.end());
        GMSparseFunction visibility(shape.begin(), shape.end(), -1.0);
        GMSparseFunction projectionWeight(shape.begin(), shape.end(), -1.0);
        GMSparseFunction constraints(shape_coords.begin(), shape_coords.end(), 0.0);

        GMSparseFunctionList modelFunctions = {visibility, projectionWeight};
        BoostObjFunctionList evals = {
                                    boost::bind(&MCMCCamGenerator::visibilityDistribution, this, _1, _2, _3),
                                    boost::bind(&MCMCCamGenerator::imagePlaneWeight, this, _1, _2, _3)
                                };
        computeDistributionForList(modelFunctions, evals, centroid, normVector, points, mapping);
        fillObjectiveFunction(vonMises, centroid, normVector, points, mapping);
        fillConstraintFunction(constraints, centroid, points, mapping);

        addFunctionTo(vonMises, model, coordinateIndices);
        addFunctionTo(visibility, model, variableIndices);
        addFunctionTo(projectionWeight, model, variableIndices);
        addFunctionTo(constraints, model, coordinateIndices);
    }

    void MCMCCamGenerator::fillObjectiveFunction(GMExplicitFunction &objFunction, GLMVec3 &centroid, GLMVec3 &normVector, GLMVec3List &points, DoubleIntMapList &mapping)
    {
        #pragma omp parallel for
        for (int p = 0; p < points.size(); p++) {
            size_t x = mapping[0][points[p].x];
            size_t y = mapping[1][points[p].y];
            size_t z = mapping[2][points[p].z];

            LabelType val = -logVonMises(points[p], centroid, normVector);
            #pragma omp critical
            objFunction(x, y, z) = val;
        }
    }

    void MCMCCamGenerator::computeDistributionForList(GMSparseFunctionList &modelFunctions, BoostObjFunctionList &evals, GLMVec3 &centroid, GLMVec3 &normVector, GLMVec3List &points, DoubleIntMapList &mapping)
    {
        #pragma omp parallel for
        for (int p = 0; p < points.size(); p++) {
            size_t x = mapping[0][points[p].x];
            size_t y = mapping[1][points[p].y];
            size_t z = mapping[2][points[p].z];

            orientationcycles(0, orientationLabels(), 0, orientationLabels()) {
                size_t coord[] = {x, y, z, (size_t)ptc, (size_t)yaw};
                GLMVec2 scaledOri = scaleOrientation(GLMVec2(ptc, yaw));

                EigVector5 pose = getPose(points[p], scaledOri);

                #pragma omp parallel for
                for (int f = 0; f < modelFunctions.size(); f++) {
                    LabelType val = evals[f](pose, centroid, normVector);
                    #pragma omp critical
                    modelFunctions[f].insert(coord, val);
                }
            }
        }
    }

    void MCMCCamGenerator::fillConstraintFunction(GMSparseFunction &constraints, GLMVec3 &centroid, GLMVec3List &points, DoubleIntMapList &mapping)
    {
        #pragma omp parallel for
        for (int p = 0; p < points.size(); p++) {
            size_t x = mapping[0][points[p].x];
            size_t y = mapping[1][points[p].y];
            size_t z = mapping[2][points[p].z];
            size_t coord[] = {x, y, z};

            for (GLMVec3 cam : getCams()) {
                addValueToConstraintFunction(constraints, points[p], cam, centroid, coord);
            }
        }
    }
    

    OrderedStates MCMCCamGenerator::extractBestResults(AdderInferencePtr algorithm)
    {
        OrderedStates x;
        ((MultiBruteforcePtr)algorithm)->argAll(x);

        std::cout << "Value obtained: " << algorithm->value() << std::endl;

        // GLMVec3 realOptima = scalePoint(GLMVec3(x[0], x[1], x[2]));
        // GLMVec2 orientOptima = scaleOrientation(GLMVec2(x[3], x[4]));
        // x[0] = realOptima.x;
        // x[1] = realOptima.y;
        // x[2] = realOptima.z;
        // x[3] = orientOptima.x;
        // x[4] = orientOptima.y;

        // std::cout << "Optimal solution: " << x[0] << ' ' << x[1] << ' ' << x[2] << ' ';
        // std::cout << x[3] << ' ' << x[4] << std::endl << std::endl;

        return x;
    }

    DoubleIntMapList MCMCCamGenerator::getPointMapping(GLMVec3List &points)
    {
        DoubleIntMapList mappingCoordinates(3, std::map<double, int>());
        DoubleSetList sets(3, DoubleSet());     // sets are ordered otherwise this won't work

        for (int p = 0; p < points.size(); p++) {
            for (int i = 0; i < 3; i++) {
                sets[i].insert(points[p][i]);
            }
        }

        for (int s = 0; s < sets.size(); s++) {
            DoubleSetIterator it = sets[s].begin();

            for (int p = 0; p < sets[s].size(); p++) {
                mappingCoordinates[s][(*it)] = p;
                std::advance(it, 1);
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

} // namespace opview
