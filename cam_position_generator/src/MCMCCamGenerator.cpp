#include <opview/MCMCCamGenerator.hpp>


namespace opview {
    
    MCMCCamGenerator::MCMCCamGenerator(CameraGeneralConfiguration &camConfig, std::string &meshFile, GLMVec3List &cams, 
                                            MCConfiguration &mcConfig, double goalAngle, double dispersion)
    {
        this->sampler = new GaussianSampleGenerator();
        this->vonMisesConfig = {deg2rad(goalAngle), dispersion};
        this->camConfig = camConfig;
        this->cams = cams;
        this->meshFile = meshFile;
        this->mcConfig = mcConfig;
        this->log = new ExhaustiveReportWriter("mcmc.json");

        this->fillTree();     // called to assure usage of overridden function
    }

    MCMCCamGenerator::~MCMCCamGenerator()
    {
        delete sampler;
        delete log;
    }

    void MCMCCamGenerator::fillTree()
    {
        Polyhedron poly = extractPolyhedron();
        TriangleList triangles = getTriangleList(poly);

        tree = new Tree(triangles.begin(), triangles.end());
    }

    Polyhedron MCMCCamGenerator::extractPolyhedron()
    {
        std::ifstream meshIn(meshFilename);
        Polyhedron poly;
        meshIn >> poly;
        meshIn.close();

        return poly;
    }

    TriangleList MCMCCamGenerator::getTriangleList(Polyhedron &poly)
    {
        TriangleList triangles;
        for (Facet_iterator it = poly.facets_begin(); it != poly.facets_end(); it++) {
            Halfedge_facet_circulator p = it->facet_begin();
            Vertex p0 = p->vertex();
            Vertex p1 = (++p)->vertex();
            Vertex p2 = (++p)->vertex();

            triangles.push_back(Triangle(p0->point(), p1->point(), p2->point()));
        }
        return triangles;
    }
    
    DoubleList MCMCCamGenerator::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        OrderedPose currentOptima = uniformMCStep(centroid, normVector, 0);

        for (int d = 0; d < mcConfig.resamplingNum; d++) {
            currentOptima = resamplingStep(centroid, normVector, currentOptima, d+1);
        }

        ValuePose best = currentOptima.top();
        return convertVectorToList(best.second);    
    }

    OrderedPose MCMCCamGenerator::uniformMCStep(GLMVec3 &centroid, GLMVec3 &normVector, int round)
    {
        EigVector5List orientedPoints = uniformPointsGetter();
        OrderedPose poses = generalStep(centroid, normVector, orientedPoints);
        return this->extractBestResults(poses, round);
    }

    OrderedPose MCMCCamGenerator::resamplingStep(GLMVec3 &centroid, GLMVec3 &normVector, OrderedPose &currentOptima, int round)
    {
        EigVector5List orientedPoints = resamplingPointsGetter(currentOptima);
        OrderedPose poses = generalStep(centroid, normVector, orientedPoints);

        EigVector5 tmp =this->extractBestResults(poses, round).top().second;
        GLMVec2 point = getProjectedPoint(tmp, centroid, camConfig);

        return this->extractBestResults(poses, round);
    }

    OrderedPose MCMCCamGenerator::generalStep(GLMVec3 &centroid, GLMVec3 &normVector, EigVector5List &orientedPoints)
    {
        DoubleList visibility(orientedPoints.size(), 0.0);
        DoubleList vonMises(orientedPoints.size(), 0.0);
        DoubleList projection(orientedPoints.size(), 0.0);
        DoubleList constraints(orientedPoints.size(), 0.0);
        DoubleList values(orientedPoints.size(), 0.0);
        
        #pragma omp parallel sections 
        {
            #pragma omp section
            computeObjectiveFunction(vonMises, orientedPoints, centroid, normVector);
            #pragma omp section
            computeVisibilityFunction(visibility, orientedPoints, centroid, normVector);
            #pragma omp section
            computeProjectionFunction(projection, orientedPoints, centroid, normVector);
            #pragma omp section
            computeConstraintFunction(constraints, orientedPoints, centroid, normVector);
        }

        sumUpAll(values, visibility, vonMises, projection, constraints);

        return orderPoses(orientedPoints, values);
    }

    void MCMCCamGenerator::sumUpAll(DoubleList &dest, DoubleList &visibility, DoubleList &vonMises, DoubleList &projection, DoubleList &constraints)
    {
        #pragma omp parallel for
        for (int i = 0; i < dest.size(); i++) {
            dest[i] = visibility[i] + vonMises[i] + projection[i] + constraints[i];
        }
    }    

    OrderedPose MCMCCamGenerator::orderPoses(EigVector5List &orientedPoints, DoubleList &values)
    {
        OrderedPose poses;

        #pragma omp parallel for
        for (int p = 0; p < orientedPoints.size(); p++) {
            ValuePose el = ValuePose(values[p], orientedPoints[p]);
            #pragma omp critical
            poses.push(el);
        }
        return poses;
    }

    void MCMCCamGenerator::computeObjectiveFunction(DoubleList &values, EigVector5List &points, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        #pragma omp parallel for
        for (int p = 0; p < points.size(); p++) {
            values[p] += logVonMisesWrapper(points[p], centroid, normVector);
        }
    }

    void MCMCCamGenerator::computeProjectionFunction(DoubleList &values, EigVector5List &points, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        #pragma omp parallel for
        for (int p = 0; p < points.size(); p++) {
            values[p] += imageProjectionDistribution(points[p], centroid, normVector);
        }
    }

    void MCMCCamGenerator::computeVisibilityFunction(DoubleList &values, EigVector5List &points, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        #pragma omp parallel for
        for (int p = 0; p < points.size(); p++) {
            values[p] += visibilityDistribution(points[p], centroid, normVector);
        }
    }

    void MCMCCamGenerator::computeConstraintFunction(DoubleList &values, EigVector5List &points, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        #pragma omp parallel for collapse(2)
        for (int p = 0; p < points.size(); p++) {
            for (int c = 0; c < cams.size(); c++) {
                values[p] += computeBDConstraint(points[p], centroid, cams[c]);
            }
        }
    }

    double MCMCCamGenerator::computeBDConstraint(EigVector5 &newCamPose, GLMVec3 &centroid, GLMVec3 &cam)
    {
        GLMVec3 point = GLMVec3(newCamPose[0], newCamPose[1], newCamPose[2]);
        double D, B;
        #pragma omp parallel sections
        {
            #pragma omp section
            B = glm::distance(point, cam);
            #pragma omp section
            D = std::min(glm::distance(cam, centroid), glm::distance(point, centroid));
        }
        
        if (B / D < BD_TERRESTRIAL_ARCHITECTURAL) {
            return -1.0;
        }
        return 0.0;
    }
    
    OrderedPose MCMCCamGenerator::extractBestResults(OrderedPose &poses, int round)
    {
        OrderedPose optima = copy(poses, (size_t)(OFFSPRING*mcConfig.particles.num));
        
        OrderedPose convertedAnglesOptima = convertAngles(optima);

        this->log->append(convertAngles(optima), round);

        return optima;
    }

    OrderedPose MCMCCamGenerator::convertAngles(OrderedPose poses)
    {
        OrderedPose optima;
        int c = 0;
        while(!poses.empty() && c < (size_t)(OFFSPRING*mcConfig.particles.num)) {
            auto tmp = poses.top();
            tmp.second[3] = rad2deg(tmp.second[3]);
            tmp.second[4] = rad2deg(tmp.second[4]);
            optima.push(tmp);
            poses.pop();
            c++;
        }
        return optima;
    }
    
    EigVector5List MCMCCamGenerator::getCentersFromOptima(OrderedPose currentOptima) // not by refernce otherwise changes also the original
    {
        EigVector5List poses;
        while(!currentOptima.empty()) {
            poses.push_back(currentOptima.top().second);
            currentOptima.pop();
        }

        return poses;
    }

    DoubleList MCMCCamGenerator::getWeightsFromOptima(OrderedPose currentOptima) // not by refernce otherwise changes also the original
    {
        DoubleList weights;
        while(!currentOptima.empty()) {
            double solution = currentOptima.top().first;
            weights.push_back(solution);
            currentOptima.pop();
        }

        return weights;
    }

    double MCMCCamGenerator::logVonMisesWrapper(EigVector5 &newCamPose, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        GLMVec3 point = GLMVec3(newCamPose[0], newCamPose[1], newCamPose[2]);
        return -logVonMises(point, centroid, normVector, vonMisesConfig);
    }

    LabelType MCMCCamGenerator::visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        if (isPointInsideImage(pose, centroid, camConfig) && isMeaningfulPose(pose, centroid, tree, camConfig)){
            return 0.0;
        }
        return -10.0;
    }

    LabelType MCMCCamGenerator::imageProjectionDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        if (!isPointInsideImage(pose, centroid, camConfig)) {  // fast rejection, fast to compute.
            return -10.0;
        }
        if (!isMeaningfulPose(pose, centroid, tree, camConfig)) {
            return -10.0;
        }
        
        GLMVec2 point = getProjectedPoint(pose, centroid, camConfig);
        double centerx = (double)camConfig.size_x / 2.0;
        double centery = (double)camConfig.size_y / 2.0;
        double sigma_x = (double)camConfig.size_x / 3.0;
        double sigma_y = (double)camConfig.size_y / 3.0;

        return logBivariateGaussian(point.x, point.y, centerx, centery, sigma_x, sigma_y);  // positive because gaussian have highest value in the center
    }

    EigVector5List MCMCCamGenerator::uniformPointsGetter()
    {  
        GLMVec3List points = sampler->getUniformSamples(
            {std::make_pair(mcConfig.bounds.lower.x, mcConfig.bounds.upper.x), 
                std::make_pair(mcConfig.bounds.lower.y, mcConfig.bounds.upper.y), 
                std::make_pair(mcConfig.bounds.lower.z, mcConfig.bounds.upper.z)}, mcConfig.particles.uniform);
        return insertOrientation(points);
    }

    EigVector5List MCMCCamGenerator::resamplingPointsGetter(OrderedPose &currentOptima)
    {  
        EigVector5List centers = getCentersFromOptima(currentOptima); 
        DoubleList weights = getWeightsFromOptima(currentOptima); 
        EigVector5List newCenters = sampler->getWeightedSamples(centers, weights, mcConfig.particles.num * 0.9);
        return concatLists(newCenters, centers);
    }

    EigVector5List MCMCCamGenerator::insertOrientation(GLMVec3List &points)
    {
        EigVector5List orientedPoints;

        #pragma omp parallel for collapse(3)
        for (int p = 0; p < points.size(); p++) {
            orientationCycles(mcConfig.particles.deltaDegree) {
                EigVector5 pose;
                pose << points[p].x, points[p].y, points[p].z, deg2rad((float)ptc), deg2rad((float)yaw);
                #pragma omp critical
                orientedPoints.push_back(pose);
            }
        }
        return orientedPoints;
    }

    MCConfiguration MCMCCamGenerator::getMCConfiguration()
    {
        return mcConfig;
    }
    
    VonMisesConfiguration MCMCCamGenerator::getVonMisesConfiguration()
    {
        return vonMisesConfig;
    }
    
    CameraGeneralConfiguration MCMCCamGenerator::getCameraConfiguration()
    {
        return camConfig;
    }

    ReportWriterPtr MCMCCamGenerator::getLogger()
    {
        return log;
    }

    void MCMCCamGenerator::setLogger(ReportWriterPtr log)
    {
        delete this->log;
        this->log = log;
    }

    GLMVec3 MCMCCamGenerator::lowerBounds()
    {
        return mcConfig.bounds.lower;
    }

    GLMVec3 MCMCCamGenerator::upperBounds()
    {
        return mcConfig.bounds.upper;
    }

} // namespace opview
