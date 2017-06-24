#include <opview/OrientationHierarchicalGraphicalModel.hpp>
#include <CGAL/exceptions.h>

namespace opview {

    OrientationHierarchicalGraphicalModel::OrientationHierarchicalGraphicalModel(SolverGeneratorPtr solver, 
                                            OrientationHierarchicalConfiguration &config, CameraGeneralConfiguration &camConfig,
                                            std::string meshFile, GLMVec3List &cams, double goalAngle, double dispersion)
                                            : HierarchicalDiscreteGraphicalModel(solver, config.config, cams, goalAngle, dispersion)
    {
        this->deltaAngle = config.deltaAngle;
        this->orientConfig = config;
        this->meshFilename = meshFile;
        this->camConfig = camConfig;
        log = new ReportWriter("orientation_log.json");

        fillTree();     // called to assure usage of overridden function
        initShapes();
    }

    OrientationHierarchicalGraphicalModel::~OrientationHierarchicalGraphicalModel()
    {
        delete log;
    }

    void OrientationHierarchicalGraphicalModel::initShapes()
    {
        super::initShapes();

        shape.clear();
        variableIndices.clear();
        for (size_t i = 0; i < numVariables() - 2; i++) {
            this->variableIndices.push_back(i);
            this->shape.push_back(numLabels());
        }
        this->variableIndices.push_back(3);
        this->shape.push_back(orientationLabels());
        this->variableIndices.push_back(4);
        this->shape.push_back(orientationLabels());
    }

    void OrientationHierarchicalGraphicalModel::fillTree()
    {
        Polyhedron poly = extractPolyhedron();
        TriangleList triangles = getTriangleList(poly);

        tree = new Tree(triangles.begin(), triangles.end());
    }

    Polyhedron OrientationHierarchicalGraphicalModel::extractPolyhedron()
    {
        std::ifstream meshIn(meshFilename);
        Polyhedron poly;
        meshIn >> poly;
        meshIn.close();

        return poly;
    }

    TriangleList OrientationHierarchicalGraphicalModel::getTriangleList(Polyhedron &poly)
    {
        TriangleList triangles;
        for (Facet_iterator it = poly.facets_begin(); it != poly.facets_end(); it++) {
            Halfedge_facet_circulator p = it->facet_begin();
            Vertex_handle p0 = p->vertex();
            Vertex_handle p1 = (++p)->vertex();
            Vertex_handle p2 = (++p)->vertex();

            Triangle t = Triangle(p0->point(), p1->point(), p2->point());
            if (t.is_degenerate()) {
                throw std::runtime_error("A triangle is degenerate.");
            }
            triangles.push_back(t);
        }

        return triangles;
    }

    void OrientationHierarchicalGraphicalModel::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        this->resetPosition();

        LabelList currentOptima = {MIN_COORDINATE, MIN_COORDINATE, MIN_COORDINATE, 0, 0};
        
        for (int d = 0; d < this->getDepth(); d++) {
            std::cout << "Current depth: " << d << std::endl;
            SimpleSpace space(shape.begin(), shape.end());
            GraphicalModelAdder model(space);

            this->fillModel(model, centroid, normVector);

            auto discreteOptima = getOptimaForDiscreteSpace(currentOptima);
            AdderInferencePtr algorithm = solverGenerator()->getOptimizerAlgorithm(model, discreteOptima, numVariables());
            algorithm->infer();

            currentOptima = this->extractResults(algorithm);

            this->reduceScale(currentOptima, d+1);
        }
    }

    LabelList OrientationHierarchicalGraphicalModel::extractResults(AdderInferencePtr algorithm)
    {
        VarIndexList x;
        algorithm->arg(x);

        std::cout << "Value obtained: " << algorithm->value() << std::endl;

        LabelList convertedOpt(x.size());
        GLMVec3 realOptima = scalePoint(GLMVec3(x[0], x[1], x[2]));
        GLMVec2 orientOptima = scaleOrientation(GLMVec2(x[3], x[4]));
        convertedOpt[0] = realOptima.x;
        convertedOpt[1] = realOptima.y;
        convertedOpt[2] = realOptima.z;
        convertedOpt[3] = orientOptima.x;
        convertedOpt[4] = orientOptima.y;

        std::cout << "Optimal solution: " << convertedOpt[0] << ", " << convertedOpt[1] << ", " << convertedOpt[2] << ", ";
        std::cout << convertedOpt[3] << ", " << convertedOpt[4] << std::endl << std::endl;

        log->append(convertedOpt, algorithm->value());

        return convertedOpt;
    }

    VarIndexList OrientationHierarchicalGraphicalModel::getOptimaForDiscreteSpace(LabelList &currentOptima)
    {
        GLMVec3 spaceOptima = unscalePoint(GLMVec3(currentOptima[0], currentOptima[1], currentOptima[2]));
        GLMVec2 spaceOrient = unscaleOrientation(GLMVec2(currentOptima[3], currentOptima[4]));

        return {(VariableIndexType)spaceOptima.x, (VariableIndexType)spaceOptima.y, (VariableIndexType)spaceOptima.z, 
                    (VariableIndexType)spaceOrient.x, (VariableIndexType)spaceOrient.y};
    }

    void OrientationHierarchicalGraphicalModel::fillModel(GraphicalModelAdder &model, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        GMExplicitFunction vonMises(shape.begin(), shape.end());
        GMExplicitFunction visibility(shape.begin(), shape.end());
        GMExplicitFunction projectionWeight(shape.begin(), shape.end());
        GMExplicitFunction constraints(shape.begin(), shape.end());

        #pragma omp parallel sections 
        {
            #pragma omp section
            fillExplicitOrientationFunction(visibility, boost::bind(&OrientationHierarchicalGraphicalModel::visibilityDistribution, this, _1, _2, _3), centroid, normVector);
            #pragma omp section
            fillExplicitOrientationFunction(projectionWeight, boost::bind(&OrientationHierarchicalGraphicalModel::imagePlaneWeight, this, _1, _2, _3), centroid, normVector);
            #pragma omp section
            fillObjectiveFunction(vonMises, centroid, normVector);
            #pragma omp section
            fillConstraintFunction(constraints, centroid);
        }

        addFunctionTo(vonMises, model, variableIndices);
        addFunctionTo(visibility, model, variableIndices);
        addFunctionTo(projectionWeight, model, variableIndices);
        addFunctionTo(constraints, model, variableIndices);
    }

    void OrientationHierarchicalGraphicalModel::fillExplicitOrientationFunction(GMExplicitFunction &modelFunction, BoostObjFunction evals, GLMVec3 &centroid, GLMVec3 &normVector) 
    {
        #pragma omp parallel for collapse(5)
        coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) { 
            orientationcycles(0, orientationLabels(), 0, orientationLabels()) {
                size_t coord[] = {(size_t)x, (size_t)y, (size_t)z, (size_t)ptc, (size_t)yaw};
                computeDistributionForFunction(modelFunction, evals, coord, centroid, normVector);
            }
        }
    }

    void OrientationHierarchicalGraphicalModel::fillObjectiveFunction(GMExplicitFunction &vonMises, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        #pragma omp parallel for collapse(3)
        coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) {
            GLMVec3 pos = scalePoint(GLMVec3(x, y, z));
            LabelType val = -logVonMisesWrapper(pos, centroid, normVector);
            orientationcycles(0, orientationLabels(), 0, orientationLabels()) {
                #pragma omp critical
                vonMises(x, y, z, ptc, yaw) = val;
            }
        }
    }

    // quite heavy as function, parallelism?
    void OrientationHierarchicalGraphicalModel::addValueToConstraintFunction(GMExplicitFunction &function, GLMVec3 &point, GLMVec3 &cam, GLMVec3 &centroid, size_t coords[])
    {
        double B, D;
        #pragma omp parallel sections 
        {
            #pragma omp section
            B = glm::distance(point, cam);
            #pragma omp section
            D = std::min(glm::distance(cam, centroid), glm::distance(point, centroid));
        }

        LabelType val = 0.0;
        if (B / D < BD_TERRESTRIAL_ARCHITECTURAL) {
            val = -10.0;
        }

        orientationcycles(0, orientationLabels(), 0, orientationLabels()) {
            #pragma omp critical
            function(coords[0], coords[1], coords[2], ptc, yaw) = val;
        }
    }

    void OrientationHierarchicalGraphicalModel::computeDistributionForFunction(GMExplicitFunction &modelFunctions, BoostObjFunction &evals, size_t coord[], GLMVec3 &centroid, GLMVec3 &normVector)
    {
        GLMVec3 scaledPos = scalePoint(GLMVec3(coord[0], coord[1], coord[2]));
        GLMVec2 scaledOri = scaleOrientation(GLMVec2(coord[3], coord[4]));

        EigVector5 pose = getPose(scaledPos, scaledOri);
        
        LabelType val = evals(pose, centroid, normVector);
        
        #pragma omp critical
        modelFunctions(coord[0], coord[1], coord[2], coord[3], coord[4]) = val;
    }

    LabelType OrientationHierarchicalGraphicalModel::visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        if (isPointInsideImage(pose, centroid) && isMeaningfulPose(pose, centroid)) {
            return 1.0;
        }
        return -10.0;
    }

    LabelType OrientationHierarchicalGraphicalModel::estimateObjDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {        
        GLMVec3 point = GLMVec3(pose[0], pose[1], pose[2]);
        return -logVonMisesWrapper(point, centroid, normalVector);     // log gives a negative number but we want a positive one to maximize
    }

    LabelType OrientationHierarchicalGraphicalModel::imagePlaneWeight(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        if (!isPointInsideImage(pose, centroid)) {  // fast rejection, fast to compute.
            return -10.0;
        }
        if (!isMeaningfulPose(pose, centroid)) {
            return -10.0;
        }

        GLMVec2 point = getProjectedPoint(pose, centroid);
        double centerx = (double)camConfig.size_x / 2.0;
        double centery = (double)camConfig.size_y / 2.0;
        double sigma_x = (double)camConfig.size_x / 3.0;
        double sigma_y = (double)camConfig.size_y / 2.0;

        return logBivariateGaussian(point.x, point.y, centerx, centery, sigma_x, sigma_y);  // positive because gaussian have highest value in the center
    }

    bool OrientationHierarchicalGraphicalModel::isMeaningfulPose(EigVector5 &pose, GLMVec3 &centroid)
    {
        return isPointInsideImage(pose, centroid) && !isIntersecting(pose, centroid) && !isOppositeView(pose, centroid);
    }

    bool OrientationHierarchicalGraphicalModel::isOppositeView(EigVector5 &pose, GLMVec3 &centroid)
    {
        GLMVec3 ray = GLMVec3(pose[0]-centroid.x, pose[1]-centroid.y, pose[3]-centroid.z);
        
        RotationMatrix R = getRotationMatrix(0, -pose[3], -pose[4]);
        GLMVec3 zDirection = R * zdir;

        return glm::dot(ray, zDirection) > 0.0f;    // if < 0.0 than it sees the object, but we want to know when it is opposite.
    }

    bool OrientationHierarchicalGraphicalModel::isIntersecting(EigVector5 &pose, GLMVec3 &centroid)
    {
        PointD3 cam(pose[0], pose[1], pose[2]);
        PointD3 point(centroid.x, centroid.y, centroid.z);
        
        Segment segment_query(cam, point);
        try {
            return tree->do_intersect(segment_query);
        } catch (CGAL::Failure_exception e) {
            std::cout << e.what() << std::endl;
            return true;
        }
    }

    bool OrientationHierarchicalGraphicalModel::isPointInsideImage(EigVector5 &pose, GLMVec3 &centroid)
    {
        GLMVec2 point2D = getProjectedPoint(pose, centroid);

        return point2D.x < (float)camConfig.size_x && point2D.x > 0.0f && point2D.y < (float)camConfig.size_y && point2D.y > 0.0f;
    }

    GLMVec2 OrientationHierarchicalGraphicalModel::getProjectedPoint(EigVector5 &pose, GLMVec3 &centroid)
    {
        GLMVec4 point3D = GLMVec4(centroid, 1.0f);
        CameraMatrix P = getCameraMatrix(pose);
        GLMVec4 point2D = P * point3D;

        point2D = point2D / point2D.z;
        return GLMVec2(point2D.x, point2D.y);
    }

    RotationMatrix OrientationHierarchicalGraphicalModel::getRotationMatrix(float roll, float pitch, float yaw)
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

    CameraMatrix OrientationHierarchicalGraphicalModel::getCameraMatrix(EigVector5 &pose)
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

    void OrientationHierarchicalGraphicalModel::reduceScale(LabelList &currentOptimal, int depth)
    {
        super::reduceScale(currentOptimal);
        this->deltaAngle = (depth < orientConfig.deltaAngles.size()) ? orientConfig.deltaAngles[depth] : orientConfig.deltaAngles[orientConfig.deltaAngles.size()-1];
    }

    void OrientationHierarchicalGraphicalModel::resetPosition()
    {
        super::resetPosition();
        this->deltaAngle = orientConfig.deltaAngles[0];
    }

    EigVector5 OrientationHierarchicalGraphicalModel::getPose(GLMVec3 &scaledPos, GLMVec2 &scaledOri)
    {
        EigVector5 pose;
        pose << scaledPos.x, scaledPos.y, scaledPos.z, deg2rad(scaledOri.x), deg2rad(scaledOri.y);
        return pose;
    }

    GLMVec2 OrientationHierarchicalGraphicalModel::scaleOrientation(GLMVec2 orientation)
    {
        return orientation * 360.0f / (float)orientationLabels();
    }

    GLMVec2 OrientationHierarchicalGraphicalModel::unscaleOrientation(GLMVec2 orientation)
    {
        return orientation * (float)orientationLabels() / 360.0f;
    }

    size_t OrientationHierarchicalGraphicalModel::numVariables()
    {
        return ORIENTATION_VARS;
    }

    size_t OrientationHierarchicalGraphicalModel::orientationLabels()
    {
        return (int) (360.0 / getDeltaAngle());
    }

    float OrientationHierarchicalGraphicalModel::getDeltaAngle()
    {
        return deltaAngle;
    }

    std::string OrientationHierarchicalGraphicalModel::getMeshFilename()
    {
        return meshFilename;
    }

    void OrientationHierarchicalGraphicalModel::setMeshFilename(std::string filename)
    {
        this->meshFilename = filename;
    }

    ReportWriterPtr OrientationHierarchicalGraphicalModel::getLogger()
    {
        return log;
    }

    void OrientationHierarchicalGraphicalModel::setLogger(ReportWriterPtr log)
    {
        delete this->log;
        this->log  = log;
    }

} // namespace opview
