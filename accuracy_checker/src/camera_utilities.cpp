#include <meshac/camera_utilities.hpp>

namespace meshac {

    const GLMVec4 zdir = GLMVec4(0.0f, 0.0f, 1.0f, 0.0f);

    ListMappingGLMVec2 projectMeshPointsThroughCameras(GLMVec3List &points, CameraList &cams, TreePtr tree)
    {
        ListMappingGLMVec2 point3DTo2DThroughCam;
        point3DTo2DThroughCam.assign(points.size(), CamToPointMap());

        #pragma omp parallel for collapse(2)
        for (int p = 0; p < points.size(); p++) {
            for (int c = 0; c < cams.size(); c++) {
                try {
                    GLMVec2 point = projectThrough(points[p], cams[c], tree);
                    #pragma omp critical
                    point3DTo2DThroughCam[p].insert(CamPointPair(c, point));
                } catch (const UnprojectablePointThroughCamException &ex) 
                { } // do nothing because it makes no sense to project it
            }
        }
        return point3DTo2DThroughCam;
    }


    GLMVec2 projectThrough(GLMVec3 &meshPoint, CameraType &P, TreePtr tree)
    {
        GLMVec2 point = getProjectedPoint(meshPoint, P.cameraMatrix);
        
        if (!isPointInsideImage(point, P)) {  // fast rejection, fast to compute
            throw UnprojectablePointThroughCamException();
        }
        if (!isMeaningfulPose(meshPoint, P, tree)) {
            throw UnprojectablePointThroughCamException();
        }
        return point;
    }

    bool isMeaningfulPose(GLMVec3 &meshPoint, CameraType &P, TreePtr tree)
    {
        return !isIntersecting(meshPoint, P, tree) && !isOppositeView(meshPoint, P);
    }

    bool isOppositeView(GLMVec3 &meshPoint, CameraType &P)
    {
        CameraMatrix E = getExtrinsicMatrix(P);

        GLMVec4 p = E * GLMVec4(meshPoint, 1.0f);
        return glm::dot(glm::normalize(p), zdir) < 0.0f;    // if > 0.0 than it sees the object.
    }

    bool isIntersecting(GLMVec3 &meshPoint, CameraType &P, TreePtr tree)
    {
        GLMVec3 pose = getCameraCenter(P);
        Point cam(pose[0], pose[1], pose[2]);
        Point point(meshPoint[0], meshPoint[1], meshPoint[2]);
        
        try {
            Segment segment_query(cam, point);
            Segment_intersection intersection;
            
            #pragma omp critical
            intersection = tree->any_intersection(segment_query);  // gives the first intersected primitives, so probably the farer one
            
            if (intersection) {
                return !isMathemathicalError(intersection, point);
            } else {
                return false;
            }
        } catch (const CGAL::Assertion_exception &ex) {
            return true;
        }
    }

    bool isMathemathicalError(Segment_intersection &intersection, Point &point)
    {
        const Point* intersectedPoint = boost::get<Point>(&(intersection->first));
        if(intersectedPoint) {
            return CGAL::squared_distance(*intersectedPoint, point) < 0.0001f;
        }
        return false;
    }

    bool isPointInsideImage(GLMVec2 &point2D, CameraType &P)
    {
        return point2D.x < (float)getImageWidth(P) && point2D.x > 0.0f && point2D.y < (float)getImageHeight(P) && point2D.y > 0.0f;
    }

    GLMVec2 getProjectedPoint(GLMVec3 &meshPoint, CameraMatrix &P)
    {
        GLMVec4 point3D = GLMVec4(meshPoint, 1.0f);
        P = glm::transpose(P);
        GLMVec4 point2D = P * point3D;

        // point2D = point2D / point2D.z;

        return GLMVec2(point2D.x, point2D.y);
    }

    int getImageWidth(CameraType &P)
    {
        return P.imageWidth;
    }

    int getImageHeight(CameraType &P)
    {
        return P.imageHeight;
    }

    GLMVec3 getCameraCenter(CameraType &P)
    {
        return P.center;
    }

    CameraMatrix getExtrinsicMatrix(CameraType &P)
    {
        CameraMatrix cam = CameraMatrix(getRotationMatrix(P));
        return glm::translate(cam, getCameraCenter(P));
    }

    RotationMatrix getRotationMatrix(CameraType &P)
    {
        return P.rotation;
    }

    CameraMatrix getCameraMatrix(CameraType &P)
    {
        return P.cameraMatrix;
    }

}
