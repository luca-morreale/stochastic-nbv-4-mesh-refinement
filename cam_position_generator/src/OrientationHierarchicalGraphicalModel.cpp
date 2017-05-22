#include <opview/OrientationHierarchicalGraphicalModel.hpp>

namespace opview {

    OrientationHierarchicalGraphicalModel::OrientationHierarchicalGraphicalModel(SolverGeneratorPtr solver, size_t depth, size_t labels,
                                                                        float dAngle, Delaunay3 &dt_, CGALCellSet &cells, 
                                                                        GLMVec3List &cams, double goalAngle, double dispersion)
                                                                        : HierarchicalDiscreteGraphicalModel(solver, depth, labels, cams, goalAngle, dispersion)
    {
        this->dAngle = dAngle;
        fillTree(dt_, cells);
    }

    OrientationHierarchicalGraphicalModel::~OrientationHierarchicalGraphicalModel()
    { /*    */ }

    void OrientationHierarchicalGraphicalModel::fillTree(Delaunay3 &dt_, CGALCellSet &cells)
    {
        std::vector<Triangle> faceList;
        for (auto cell : cells) {
            for (int facetIndex = 0; facetIndex < 4; facetIndex++){
                faceList.push_back(dt_.triangle(cell, facetIndex));
            }
        }
        
        tree = new Tree(faceList.begin(), faceList.end());
    }


    void OrientationHierarchicalGraphicalModel::fillObjectiveFunction(GMExplicitFunction &vonMises, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        #pragma omp parallel for collapse(5)
        coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) { 
            orientationcycles(0, orientationLabels(), 0, orientationLabels()) { 
                GLMVec3 scaledPos = scalePoint(GLMVec3(x, y, z));
                EigVector5 pose;
                pose << scaledPos.x, scaledPos.y, scaledPos.z, deg2rad(ptc), deg2rad(yaw);
                
                LabelType val = computeObjectiveFunction(pose, centroid, normVector);

                #pragma omp critical
                vonMises(pose[0], pose[1], pose[2], pose[3], pose[4]) = val;
            }
        }
    }

    LabelType OrientationHierarchicalGraphicalModel::computeObjectiveFunction(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        GLMVec3 point = GLMVec3(pose[0], pose[1], pose[2]);

        if (!isPointInsideImage(pose, centroid) || isIntersecting(pose, centroid) || isOppositeView(pose, centroid)) {
            return 0.0;
        }

        return logVonMises(point, centroid, normalVector);
    }

    bool OrientationHierarchicalGraphicalModel::isOppositeView(EigVector5 &pose, GLMVec3 &centroid)
    {
        PointD3 cam(pose[0], pose[1], pose[2]);
        PointD3 point(centroid.x, centroid.y, centroid.z);
        
        CGALVec3 rayVector = Ray(cam, point).to_vector();
        GLMVec3 ray = GLMVec3(rayVector[0], rayVector[1], rayVector[2]);
        GLMVec3 direction = GLMVec3(1.0, cos(pose[3]), cos(pose[4]));

        return glm::dot(ray, direction) < 0;
    }

    bool OrientationHierarchicalGraphicalModel::isIntersecting(EigVector5 &pose, GLMVec3 &centroid)
    {
        PointD3 cam(pose[0], pose[1], pose[2]);
        PointD3 point(centroid.x, centroid.y, centroid.z);
        Segment segment_query(cam, point);
        return tree->do_intersect(segment_query);
    }

    bool OrientationHierarchicalGraphicalModel::isPointInsideImage(EigVector5 &pose, GLMVec3 &centroid)
    {
        GLMVec4 viewport = GLMVec4(0, 0, 1920, 1080);
        CameraMatrix projection = glm::perspective(glm::radians(90.0f), (float) 1920 / (float)1080, 0.1f, 100.0f); // what is this??
        CameraMatrix cam = getCameraMatrix(pose);

        GLMVec3 projected = glm::project(centroid, cam, projection, viewport);
        projected = projected / projected.z;

        return projected.x < 1920.0f && projected.x > 0.0f && projected.y < 1080.0f && projected.y > 0.0f;
    }

    CameraMatrix OrientationHierarchicalGraphicalModel::getCameraMatrix(EigVector5 &pose)
    {
        GLMVec3 cameraPos = GLMVec3(pose[0], pose[1], pose[2]);
        GLMVec3 cameraOrient = GLMVec3(sin(pose[3]) * cos(pose[4]), sin(pose[3]), sin(pose[3]) * cos(pose[4]));
        cameraOrient = glm::normalize(cameraOrient);
        
        return glm::lookAt(cameraPos, cameraPos + cameraOrient, GLMVec3(0.0f, 1.0f,  0.0f));
    }


    size_t OrientationHierarchicalGraphicalModel::numVariables()
    {
        return ORIENTATION_VARS;
    }

    size_t OrientationHierarchicalGraphicalModel::orientationLabels()
    {
        return (int) (360.0 / deltaAngle());
    }

    float OrientationHierarchicalGraphicalModel::deltaAngle()
    {
        return dAngle;
    }

    float OrientationHierarchicalGraphicalModel::deg2rad(float deg)
    {
        return deg * M_PI / 180.0;
    }

    float OrientationHierarchicalGraphicalModel::rad2deg(float rad)
    {
        return rad * 180.0 / M_PI;
    }
    //

} // namespace opview
