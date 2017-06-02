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

    void OrientationHierarchicalGraphicalModel::fillModel(GraphicalModelAdder &model, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        GMSparseFunction vonMises(shape.begin(), shape.end(), 0.0);
        GMSparseFunction constraints(shape.begin(), shape.end(), 0.0);
        GMSparseFunction distances(shape.begin(), shape.end(), 0.0);

        fillObjectiveFunction(vonMises, centroid, normVector);
        addFunctionTo(vonMises, model, variableIndices);

        fillConstraintFunction(constraints, distances, centroid);
        addFunctionTo(constraints, model, variableIndices);
        addFunctionTo(distances, model, variableIndices);
    }

    void OrientationHierarchicalGraphicalModel::fillObjectiveFunction(GMSparseFunction &vonMises, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        #pragma omp parallel for collapse(5)
        coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) { 
            orientationcycles(0, orientationLabels(), 0, orientationLabels()) { 
                GLMVec3 scaledPos = scalePoint(GLMVec3(x, y, z));
                GLMVec2 scaledOri = scaleOrientation(GLMVec2(ptc, yaw));

                EigVector5 pose = getPose(scaledPos, scaledOri);
                
                LabelType val = computeObjectiveFunction(pose, centroid, normVector);
                size_t coord[] = {(size_t)x, (size_t)y, (size_t)z, (size_t)ptc, (size_t)yaw};

                #pragma omp critical
                vonMises.insert(coord, val);
            }
        }
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

    LabelType OrientationHierarchicalGraphicalModel::computeObjectiveFunction(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        GLMVec3 point = GLMVec3(pose[0], pose[1], pose[2]);

        if (!isMeaningfulPose(pose, centroid)) {
            return 0.0;
        }
        return -logVonMises(point, centroid, normalVector);     // log gives a negative number but we want a positive one to maximize
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

        return glm::dot(ray, zDirection) < 0;
    }

    bool OrientationHierarchicalGraphicalModel::isIntersecting(EigVector5 &pose, GLMVec3 &centroid)
    {
        PointD3 cam((double)pose[0], (double)pose[1], (double)pose[2]);

        PointD3 point((double)centroid.x, (double)centroid.y, (double)centroid.z);
        Segment segment_query(cam, point);

        return tree->do_intersect(segment_query);
    }

    bool OrientationHierarchicalGraphicalModel::isPointInsideImage(EigVector5 &pose, GLMVec3 &centroid)
    {
        GLMVec4 point3D = GLMVec4(centroid, 1.0);
        CameraMatrix P = getCameraMatrix(pose);
        GLMVec4 point2D = P * point3D;

        point2D = point2D / point2D.z;

        return point2D.x < 1920.0f && point2D.x > 0.0f && point2D.y < 1080.0f && point2D.y > 0.0f;
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
