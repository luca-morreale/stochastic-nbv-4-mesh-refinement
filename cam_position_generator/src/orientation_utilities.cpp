#include <opview/orientation_utilities.hpp>

namespace opview {

    bool isMeaningfulPose(EigVector5 &pose, GLMVec3 &centroid, TreePtr tree, CameraGeneralConfiguration &camConfig)
    {
        return isPointInsideImage(pose, centroid, camConfig) && !isIntersecting(pose, centroid, tree) && !isOppositeView(pose, centroid);
    }

    bool isOppositeView(EigVector5 &pose, GLMVec3 &centroid)
    {
        CameraMatrix E = getExtrinsicMatrix(pose);

        GLMVec4 point = GLMVec4(centroid, 1.0f) * E;
        return point.z < 0.0;
    }

    bool isIntersecting(EigVector5 &pose, GLMVec3 &centroid, TreePtr tree)
    {
        Point cam(pose[0], pose[1], pose[2]);
        Point point(centroid.x, centroid.y, centroid.z);
        
        Segment segment_query(cam, point);
        if (segment_query.is_degenerate()) return true;
        Segment_intersection intersection = tree->any_intersection(segment_query);  // gives the first intersected primitives, so probably the farer one

        if (intersection) {
            return !isMathemathicalError(intersection, point);
        } else {
            return false;
        }
    }

    bool isMathemathicalError(Segment_intersection &intersection, Point &point)
    {
        const Point* intersectedPoint = boost::get<Point>(&(intersection->first));
        if(intersectedPoint) {
            return CGAL::squared_distance(*intersectedPoint, point) < 0.0001;
        }
        return false;
    }

    bool isPointInsideImage(EigVector5 &pose, GLMVec3 &centroid, CameraGeneralConfiguration &camConfig)
    {
        GLMVec2 point2D = getProjectedPoint(pose, centroid, camConfig);

        return point2D.x > 0.0f && point2D.x < camConfig.size_x && point2D.y > 0.0f && point2D.y < camConfig.size_y;
    }

    GLMVec2 getProjectedPoint(EigVector5 &pose, GLMVec3 &centroid, CameraGeneralConfiguration &camConfig)
    {
        GLMVec4 point3D = GLMVec4(centroid, 1.0f);
        CameraMatrix P = getCameraMatrix(pose, camConfig);
        GLMVec4 point2D = point3D * P;
        point2D = point2D / point2D.z;

        
        return GLMVec2(point2D.x, point2D.y);
    }

    RotationMatrix getRotationMatrix(float roll, float pitch, float yaw)
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
        return Rz * glm::transpose(Ry) * Rx;
    }

    CameraMatrix getExtrinsicMatrix(EigVector5 &pose)
    {
        RotationMatrix R = getRotationMatrix(0, pose[3], pose[4]);  // already radians
        // R = glm::transpose(R);
        
        GLMVec3 t(pose[0], pose[1], pose[2]);
        t = -t * R;

        CameraMatrix E = CameraMatrix(R);
        E[0][3] = t[0];
        E[1][3] = t[1];
        E[2][3] = t[2];
        E[3][3] = 1.0f;

        return E;
    }

    CameraMatrix getIntrisincMatrix(CameraGeneralConfiguration &camConfig)
    {
        CameraMatrix K(0.0f);
        K[0][0] = (float)camConfig.f;
        K[1][1] = (float)camConfig.f;  // correct
        K[0][2] = (float)camConfig.size_x / 2.0f;
        K[1][2] = (float)camConfig.size_y / 2.0f;
        K[2][2] = 1.0f;
        K[3][3] = 1.0f;

        return K;
    }

    CameraMatrix getCameraMatrix(EigVector5 &pose, CameraGeneralConfiguration &camConfig)
    {
        CameraMatrix E = getExtrinsicMatrix(pose);
        CameraMatrix K = getIntrisincMatrix(camConfig);

        return E * K;
    }


}
