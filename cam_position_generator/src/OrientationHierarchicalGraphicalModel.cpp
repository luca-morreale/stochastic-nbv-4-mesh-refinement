#include <opview/OrientationHierarchicalGraphicalModel.hpp>
#include <CGAL/exceptions.h>

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

    void OrientationHierarchicalGraphicalModel::initShapes()
    {
        super::initShapes();

        coordinateIndices.clear();
        coordinateShape.clear();
        for (size_t i = 0; i < numVariables() - 2; i++) {
            this->coordinateIndices.push_back(i);
            this->coordinateShape.push_back(numLabels());
        }
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

    LabelList OrientationHierarchicalGraphicalModel::extractResults(AdderInferencePtr algorithm)
    {
        LabelList x;
        algorithm->arg(x);

        std::cout << "Value obtained: " << algorithm->value() << std::endl;

        GLMVec3 realOptima = scalePoint(GLMVec3(x[0], x[1], x[2]));
        GLMVec2 orientOptima = scaleOrientation(GLMVec2(x[3], x[4]));
        x[0] = realOptima.x;
        x[1] = realOptima.y;
        x[2] = realOptima.z;
        x[3] = orientOptima.x;
        x[4] = orientOptima.y;

        std::cout << "Optimal solution: " << x[0] << ' ' << x[1] << ' ' << x[2] << ' ';
        std::cout << x[3] << ' ' << x[4] << std::endl << std::endl;

        return x;
    }

    void OrientationHierarchicalGraphicalModel::fillModel(GraphicalModelAdder &model, GLMVec3 &centroid, GLMVec3 &normVector)
    {        
        GMExplicitFunction vonMises(coordinateShape.begin(), coordinateShape.end());
        GMExplicitFunction visibility(shape.begin(), shape.end());
        GMExplicitFunction projectionWeight(shape.begin(), shape.end());
        GMExplicitFunction constraints(coordinateShape.begin(), coordinateShape.end());

        GMExplicitFunctionList modelFunctions = {visibility, projectionWeight};
        BoostObjFunctionList evals = {
                                boost::bind(&OrientationHierarchicalGraphicalModel::visibilityDistribution, this, _1, _2, _3),
                                boost::bind(&OrientationHierarchicalGraphicalModel::imagePlaneWeight, this, _1, _2, _3)
                            };
        fillExplicitOrientationFunctions(modelFunctions, evals, centroid, normVector);
        fillObjectiveFunction(vonMises, centroid, normVector);
        fillConstraintFunction(constraints, centroid);
        
        addFunctionTo(vonMises, model, coordinateIndices);
        addFunctionTo(visibility, model, variableIndices);
        addFunctionTo(projectionWeight, model, variableIndices);
        addFunctionTo(constraints, model, coordinateIndices);
    }

    void OrientationHierarchicalGraphicalModel::fillExplicitOrientationFunctions(GMExplicitFunctionList &modelFunctions, BoostObjFunctionList &evals, GLMVec3 &centroid, GLMVec3 &normVector) 
    {
        #pragma omp parallel for collapse(5)
        coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) { 
            orientationcycles(0, orientationLabels(), 0, orientationLabels()) {
                size_t coord[] = {(size_t)x, (size_t)y, (size_t)z, (size_t)ptc, (size_t)yaw};
                computeDistributionForFunctions(modelFunctions, evals, coord, centroid, normVector);
            }
        }
    }

    void OrientationHierarchicalGraphicalModel::computeDistributionForFunctions(GMExplicitFunctionList &modelFunctions, BoostObjFunctionList &evals, size_t coord[], GLMVec3 &centroid, GLMVec3 &normVector)
    {
        GLMVec3 scaledPos = scalePoint(GLMVec3(coord[0], coord[1], coord[2]));
        GLMVec2 scaledOri = scaleOrientation(GLMVec2(coord[3], coord[4]));

        EigVector5 pose = getPose(scaledPos, scaledOri);
        
        for (int f = 0; f < modelFunctions.size(); f++) {
            LabelType val = evals[f](pose, centroid, normVector);
            auto coords = coord;
        
            #pragma omp critical
            (modelFunctions[f])(coords) = val;
        }
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

        return bivariateGuassian(point.x, point.y, centerx, centery, sigma_x, sigma_y);  // positive because gaussian have highest value in the center
    }

    bool OrientationHierarchicalGraphicalModel::isMeaningfulPose(EigVector5 &pose, GLMVec3 &centroid)
    {
        return isPointInsideImage(pose, centroid) && !isIntersecting(pose, centroid) && !isOppositeView(pose, centroid);
    }

    bool OrientationHierarchicalGraphicalModel::isOppositeView(EigVector5 &pose, GLMVec3 &centroid)
    {
        PointD3 cam(pose[0], pose[1], pose[2]);
        PointD3 point(centroid.x, centroid.y, centroid.z);
        
        CGALVec3 rayVector = Ray(cam, point).to_vector();
        GLMVec3 ray = GLMVec3(rayVector[0], rayVector[1], rayVector[2]);
        
        RotationMatrix R = getRotationMatrix(0, pose[3], pose[4]);
        GLMVec3 zDirection = R * zdir;

        return glm::dot(ray, zDirection) > 0.0f;
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
    }

    CameraMatrix OrientationHierarchicalGraphicalModel::getCameraMatrix(EigVector5 &pose)
    {
        RotationMatrix R = getRotationMatrix(0, pose[3], pose[4]);
        GLMVec3 t = GLMVec3(pose[0], pose[1], pose[2]);
        CameraMatrix K = glm::scale(GLMVec3(camConfig.f, camConfig.f , 1.0f));

        CameraMatrix P = CameraMatrix(R);

        P = glm::translate(P, t);
        P[2][3] = 1.0f;  // fourth column, third row
        P[3][3] = 0.0f;  // fourth column, fourth row

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
