#include <opview/MCMCCamGenerator.hpp>


namespace opview {
    
    MCMCCamGenerator::MCMCCamGenerator(CameraGeneralConfiguration &camConfig, std::string &meshFile, GLMVec3List &cams, 
                                            MCConfiguration &mcConfig, double goalAngle, double dispersion)
    {
        this->sampler = new MCMCSamplerGenerator();
        this->vonMisesConfig = {deg2rad(goalAngle), dispersion};
        this->camConfig = camConfig;
        this->cams = cams;
        this->meshFile = meshFile;
        this->mcConfig = mcConfig;
        this->initLambdas();
        this->fillTree();     // called to assure usage of overridden function
    }

    MCMCCamGenerator::~MCMCCamGenerator()
    {
        delete sampler;
    }

    void MCMCCamGenerator::initLambdas() 
    { 
        this->uniformPointGetter = [this](OrderedPose &currentOptima)  
                {  
                    return sampler->getUniformSamples({std::make_pair(offsetX(), 1.0f), std::make_pair(offsetY(), 1.0f), std::make_pair(offsetZ(), 1.0f)}, mcConfig.particles);  
                };
        this->resamplingPointGetter = [this](OrderedPose &currentOptima)
                {  
                    GLMVec3List centers = getCentersFromOptima(currentOptima); 
                    DoubleList weights = getWeightsFromOptima(currentOptima); 
                    return sampler->getWeightedSamples(centers, weights, mcConfig.particles);  
                };
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
            Vertex_handle p0 = p->vertex();
            Vertex_handle p1 = (++p)->vertex();
            Vertex_handle p2 = (++p)->vertex();

            triangles.push_back(Triangle(p0->point(), p1->point(), p2->point()));
        }
        return triangles;
    }
    
    void MCMCCamGenerator::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        OrderedPose currentOptima = uniformMCStep(centroid, normVector);

        for (int d = 0; d < mcConfig.resamplingNum; d++) {
            currentOptima = resamplingMCStep(centroid, normVector, currentOptima);
        }

    }

    OrderedPose MCMCCamGenerator::uniformMCStep(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        OrderedPose empty;
        OrderedPose poses = generalStep(centroid, normVector, empty, uniformPointGetter);
        return this->extractBestResults(poses);
    }

    OrderedPose MCMCCamGenerator::resamplingMCStep(GLMVec3 &centroid, GLMVec3 &normVector, OrderedPose &currentOptima)
    {
        OrderedPose poses = generalStep(centroid, normVector, currentOptima, resamplingPointGetter);
        return this->extractBestResults(poses);
    }

    OrderedPose MCMCCamGenerator::generalStep(GLMVec3 &centroid, GLMVec3 &normVector, OrderedPose &currentOptima, LambdaGLMPointsList &getPoints)
    {
        GLMVec3List points = getPoints(currentOptima);
        EigVector5List orientedPoints = insertOrientation(points);

        DoubleList visibility(orientedPoints.size(), 0.0);
        DoubleList vonMises(orientedPoints.size(), 0.0);
        DoubleList projection(orientedPoints.size(), 0.0);
        DoubleList constraints(orientedPoints.size(), 0.0);
        DoubleList values(orientedPoints.size(), 0.0);
        
        #pragma omp sections 
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
        #pragma omp parallel
        for (int i = 0; i < dest.size(); i++) {
            dest[i] = visibility[i] + vonMises[i] + projection[i] + constraints[i];
        }
    }

    EigVector5List MCMCCamGenerator::insertOrientation(GLMVec3List &points)
    {
        EigVector5List orientedPoints;

        #pragma omp parallel for collapse(3)
        for (int p = 0; p < points.size(); p++) {
            orientationCycles() {
                EigVector5 pose;
                pose << points[p].x, points[p].y, points[p].z, (float)ptc, (float)yaw;
                #pragma omp critical
                orientedPoints.push_back(pose);
            }
        }
        return orientedPoints;
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
        #pragma omp sections
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
    
    OrderedPose MCMCCamGenerator::extractBestResults(OrderedPose &poses)
    {
        OrderedPose optima;
        std::cout << "Best Value: " << poses.top().first << std::endl;
        std::cout << "Best Pose: " << poses.top().second.transpose() << std::endl << std::endl;

        int c = 0;
        while(!poses.empty() && c < (size_t)(OFFSPRING*mcConfig.particles)) {
            optima.push(poses.top());
            poses.pop();
            c++;
        }

        return optima;
    }
    
    GLMVec3List MCMCCamGenerator::getCentersFromOptima(OrderedPose currentOptima) // not by refernce otherwise changes also the original
    {
        GLMVec3List coords;
        while(!currentOptima.empty()) {
            EigVector5 solution = currentOptima.top().second;
            coords.push_back(GLMVec3(solution[0], solution[1], solution[2]));
            currentOptima.pop();
        }

        return coords;
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
        if (isPointInsideImage(pose, centroid) && isMeaningfulPose(pose, centroid)){
            return 1.0;
        }
        return 0.0;
    }

    LabelType MCMCCamGenerator::imageProjectionDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        if (!isPointInsideImage(pose, centroid)) {  // fast rejection, fast to compute.
            return 0.0;
        }
        if (!isMeaningfulPose(pose, centroid)) {
            return 0.0;
        }

        GLMVec2 point = getProjectedPoint(pose, centroid);
        double centerx = (double)camConfig.size_x / 2.0;
        double centery = (double)camConfig.size_y / 2.0;
        double sigma_x = (double)camConfig.size_x / 3.0;
        double sigma_y = (double)camConfig.size_y / 2.0;

        return bivariateGuassian(point.x, point.y, centerx, centery, sigma_x, sigma_y);  // positive because gaussian have highest value in the center
    }

    bool MCMCCamGenerator::isMeaningfulPose(EigVector5 &pose, GLMVec3 &centroid)
    {
        return isPointInsideImage(pose, centroid) 
        && !isIntersecting(pose, centroid) 
        && !isOppositeView(pose, centroid);
    }

    bool MCMCCamGenerator::isOppositeView(EigVector5 &pose, GLMVec3 &centroid)
    {
        PointD3 cam(pose[0], pose[1], pose[2]);
        PointD3 point(centroid.x, centroid.y, centroid.z);
        
        CGALVec3 rayVector = Ray(cam, point).to_vector();
        GLMVec3 ray = GLMVec3(rayVector[0], rayVector[1], rayVector[2]);
        
        RotationMatrix R = getRotationMatrix(0, -pose[3], -pose[4]);
        GLMVec3 zDirection = R * zdir;

        return glm::dot(ray, zDirection) > 0.0;
    }

    bool MCMCCamGenerator::isIntersecting(EigVector5 &pose, GLMVec3 &centroid)
    {
        PointD3 cam(pose[0], pose[1], pose[2]);
        PointD3 point(centroid.x, centroid.y, centroid.z);
        
        Segment segment_query(cam, point);
        return tree->do_intersect(segment_query);
    }

    bool MCMCCamGenerator::isPointInsideImage(EigVector5 &pose, GLMVec3 &centroid)
    {
        GLMVec2 point2D = getProjectedPoint(pose, centroid);

        return point2D.x < camConfig.size_x && point2D.x > 0.0 && point2D.y < camConfig.size_y && point2D.y > 0.0;
    }

    GLMVec2 MCMCCamGenerator::getProjectedPoint(EigVector5 &pose, GLMVec3 &centroid)
    {
        GLMVec4 point3D = GLMVec4(centroid, 1.0f);
        CameraMatrix P = getCameraMatrix(pose);
        GLMVec4 point2D = P * point3D;

        point2D = point2D / point2D.z;
        return GLMVec2(point2D.x, point2D.y);
    }

    RotationMatrix MCMCCamGenerator::getRotationMatrix(float roll, float pitch, float yaw)
    {
        // Calculate rotation about x axis
        RotationMatrix Rx = RotationMatrix(1, 0, 0, 
                                        0, std::cos(roll), -std::sin(roll),
                                        0, std::sin(roll), std::cos(roll));
        // Calculate rotation about y axis
        RotationMatrix Ry = RotationMatrix(std::cos(pitch), 0, std::sin(pitch), 
                                        0, 1, 0,
                                        -std::sin(pitch), 0, std::cos(pitch));
        // Calculate rotation about z axis
        RotationMatrix Rz = RotationMatrix(std::cos(yaw), -std::sin(yaw), 0, 
                                        std::sin(yaw), std::cos(yaw), 0,
                                        0, 0, 1);
        return (Rz * Ry) * Rx;
    }

    CameraMatrix MCMCCamGenerator::getCameraMatrix(EigVector5 &pose)
    {
        RotationMatrix R = getRotationMatrix(0, -pose[3], -pose[4]);  // already radians
        R = glm::transpose(R);
        CameraMatrix K = glm::scale(GLMVec3(camConfig.f, -camConfig.f , 1.0f));  // correct
        K[3][0] = (double)camConfig.size_x / 2.0;
        K[3][1] = (double)camConfig.size_y / 2.0;

        CameraMatrix P = CameraMatrix(R);

        GLMVec3 t(pose[0], pose[1], pose[2]);
        t = -R * t;

        P[3][0] = t[0];
        P[3][1] = t[1];
        P[3][2] = t[2];

        return K * P;
    }

} // namespace opview
