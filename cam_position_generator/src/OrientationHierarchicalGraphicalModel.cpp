#include <opview/OrientationHierarchicalGraphicalModel.hpp>

namespace opview {

    OrientationHierarchicalGraphicalModel::OrientationHierarchicalGraphicalModel(SolverGeneratorPtr solver, 
                                            OrientationHierarchicalConfiguration &config, CameraGeneralConfiguration &camConfig,
                                            std::string meshFile, GLMVec3List &cams, double goalAngle, double dispersion)
                                            : HierarchicalDiscreteGraphicalModel(solver, config.config, cams, goalAngle, dispersion)
    {
        this->deltaAngle = config.deltaAngle;
        this->meshFilename = meshFile;
        this->camConfig = camConfig;

        fillTree();     // called to assure usage of overridden function
        initShapes();
    }

    OrientationHierarchicalGraphicalModel::~OrientationHierarchicalGraphicalModel()
    { /*    */ }

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

            triangles.push_back(Triangle(p0->point(), p1->point(), p2->point()));
        }
        return triangles;
    }

    LabelList OrientationHierarchicalGraphicalModel::extractResults(AdderInferencePtr algorithm)
    {
        LabelList x;
        algorithm->arg(x);

        std::cout << "Value obtained: " << algorithm->value() << std::endl;

        GLMVec3 realOptima = scalePoint(GLMVec3(x[0], x[1], x[2]));
        x[0] = realOptima.x;
        x[1] = realOptima.y;
        x[2] = realOptima.z;

        GLMVec2 orientOptima = scaleOrientation(GLMVec2(x[3], x[4]));
        x[3] = orientOptima.x;
        x[4] = orientOptima.y;

        std::cout << "Optimal solution: " << x[0] << ' ' << x[1] << ' ' << x[2] << ' ';
        std::cout << x[3] << ' ' << x[4] << std::endl << std::endl;

        return x;
    }

    void OrientationHierarchicalGraphicalModel::fillModel(GraphicalModelAdder &model, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        GMSparseFunction vonMises(shape.begin(), shape.end(), 0.0);
        GMSparseFunction projectionWeight(shape.begin(), shape.end(), 0.0);
        GMSparseFunction constraints(shape.begin(), shape.end(), 0.0);
        GMSparseFunction distances(shape.begin(), shape.end(), 0.0);

        GMSparseFunctionList modelFunctions = {vonMises, projectionWeight};
        BoostObjFunctionList evals = {
                                boost::bind(&OrientationHierarchicalGraphicalModel::estimateObjDistribution, this, _1, _2, _3),
                                boost::bind(&OrientationHierarchicalGraphicalModel::imagePlaneWeight, this, _1, _2, _3)
                            };
        fillSparseObjectivesFromFunctions(modelFunctions, evals, centroid, normVector);
        fillConstraintFunction(constraints, distances, centroid);

        addFunctionTo(vonMises, model, variableIndices);
        addFunctionTo(constraints, model, variableIndices);
        addFunctionTo(distances, model, variableIndices);
    }

    void OrientationHierarchicalGraphicalModel::fillSparseObjectivesFromFunctions(GMSparseFunctionList &modelFunctions, BoostObjFunctionList &evals, GLMVec3 &centroid, GLMVec3 &normVector) 
    {
        #pragma omp parallel for collapse(5)
        coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) { 
            orientationcycles(0, orientationLabels(), 0, orientationLabels()) {
                size_t coord[] = {(size_t)x, (size_t)y, (size_t)z, (size_t)ptc, (size_t)yaw};
                computeDistributionForFunctions(modelFunctions, evals, coord, centroid, normVector);
            }
        }
    }

    void OrientationHierarchicalGraphicalModel::computeDistributionForFunctions(GMSparseFunctionList &modelFunctions, BoostObjFunctionList &evals, size_t coord[], GLMVec3 &centroid, GLMVec3 &normVector)
    {
        GLMVec3 scaledPos = scalePoint(GLMVec3(coord[0], coord[1], coord[2]));
        GLMVec2 scaledOri = scaleOrientation(GLMVec2(coord[3], coord[4]));

        EigVector5 pose = getPose(scaledPos, scaledOri);
        
        for (int f = 0; f < modelFunctions.size(); f++) {
            LabelType val = evals[f](pose, centroid, normVector);
        
            #pragma omp critical
            modelFunctions[f].insert(coord, val);
        }
    }

    LabelType OrientationHierarchicalGraphicalModel::estimateObjDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        if (!isPointInsideImage(pose, centroid)) {  // fast rejection, fast to compute.
            return 0.0;
        }

        if (!isMeaningfulPose(pose, centroid)) {
            return 0.0;
        }

        GLMVec3 point = GLMVec3(pose[0], pose[1], pose[2]);
        return -logVonMises(point, centroid, normalVector);     // log gives a negative number but we want a positive one to maximize
    }

    LabelType OrientationHierarchicalGraphicalModel::imagePlaneWeight(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        if (!isPointInsideImage(pose, centroid)) {  // fast rejection, fast to compute.
            return 0.0;
        }
        if (!isMeaningfulPose(pose, centroid)) {
            return 0.0;
        }

        GLMVec2 point = getProjectedPoint(pose, centroid);
        double centerx = camConfig.size_x / 2.0;
        double centery = camConfig.size_y / 2.0;
        double sigma_x = camConfig.size_x / 3.0;
        double sigma_y = camConfig.size_y / 2.0;

        return bivariateGuassian(point.x, point.y, centerx, centery, sigma_x, sigma_y);  // positive because gaussian have highest value in the center
    }

    bool OrientationHierarchicalGraphicalModel::isMeaningfulPose(EigVector5 &pose, GLMVec3 &centroid)
    {
        return isPointInsideImage(pose, centroid) 
        && !isIntersecting(pose, centroid) 
        && !isOppositeView(pose, centroid)
        ;
    }

    bool OrientationHierarchicalGraphicalModel::isOppositeView(EigVector5 &pose, GLMVec3 &centroid)
    {
        PointD3 cam(pose[0], pose[1], pose[2]);
        PointD3 point(centroid.x, centroid.y, centroid.z);
        
        CGALVec3 rayVector = Ray(cam, point).to_vector();
        GLMVec3 ray = GLMVec3(rayVector[0], rayVector[1], rayVector[2]);
        
        RotationMatrix R = getRotationMatrix(0, pose[3], pose[4]);
        GLMVec3 zDirection = R * zdir;

        return glm::dot(ray, zDirection) < 0.0;
    }

    bool OrientationHierarchicalGraphicalModel::isIntersecting(EigVector5 &pose, GLMVec3 &centroid)
    {
        PointD3 cam(pose[0], pose[1], pose[2]);
        PointD3 point(centroid.x, centroid.y, centroid.z);
        
        Segment segment_query(cam, point);
        //return false;
        // std::cout << segment_query << std::endl;
        // -4.84375 -9.53125 6.09375 -0.40168 -0.0697156 2.3197
        // -6.09375 -9.53125 7.65625 -0.405273 -0.0428495 2.34096
        // -6.09375 -9.84375 7.03125 -0.443026 -0.075465 2.31818
        // -4.21875 -10.1562 7.03125 -0.4016 -0.0697 2.3197
        // -4.53125 -9.84375 8.28125 -0.4016 -0.0697 2.3197
        // -4.84375 -9.53125 6.71875 -0.4052 -0.0428 2.3409
        return tree->do_intersect(segment_query);
    }

    bool OrientationHierarchicalGraphicalModel::isPointInsideImage(EigVector5 &pose, GLMVec3 &centroid)
    {
        GLMVec2 point2D = getProjectedPoint(pose, centroid);

        return point2D.x < camConfig.size_x && point2D.x > 0.0 && point2D.y < camConfig.size_y && point2D.y > 0.0;
    }

    GLMVec2 OrientationHierarchicalGraphicalModel::getProjectedPoint(EigVector5 &pose, GLMVec3 &centroid)
    {
        GLMVec4 point3D = GLMVec4(centroid, 1.0);
        CameraMatrix P = getCameraMatrix(pose);
        GLMVec4 point2D = P * point3D;

        point2D = point2D / point2D.z;
        return GLMVec2(point2D.x, point2D.y);
    }

    RotationMatrix OrientationHierarchicalGraphicalModel::getRotationMatrix(float roll, float pitch, float yaw)
    {
        // Calculate rotation about x axis
        RotationMatrix Rx = RotationMatrix(GLMVec3(1, 0, 0), 
                                        GLMVec3(0, cos(roll), -sin(roll)),
                                        GLMVec3(0, sin(roll), cos(roll)));
        // Calculate rotation about y axis
        RotationMatrix Ry = RotationMatrix(GLMVec3(cos(pitch), 0, sin(pitch)), 
                                        GLMVec3(0, 1, 0),
                                        GLMVec3(-sin(pitch), 0, cos(pitch)));
        // Calculate rotation about z axis
        RotationMatrix Rz = RotationMatrix(GLMVec3(cos(yaw), -sin(yaw), 0), 
                                        GLMVec3(sin(yaw), cos(yaw), 0),
                                        GLMVec3(0, 0, 1));
        return Rz * Ry * Rx;

        // glm::vec3 myRotationAxis( ??, ??, ??);
        // glm::rotate( angle_in_degrees, myRotationAxis );
    }

    CameraMatrix OrientationHierarchicalGraphicalModel::getCameraMatrix(EigVector5 &pose)
    {
        RotationMatrix R = getRotationMatrix(0, pose[3], pose[4]);
        GLMVec3 t = GLMVec3(pose[0], pose[1], pose[2]);
        CameraMatrix K = glm::scale(GLMVec3(camConfig.f, camConfig.f , 1.0f));

        CameraMatrix P = CameraMatrix(R);

        P = glm::translate(P, t);
        P[4][3] = 1.0;  // fourth column, third row
        P[4][4] = 0.0;  // fourth column, fourth row

        return K * P;
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

} // namespace opview
