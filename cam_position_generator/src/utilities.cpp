
#include <opview/utilities.hpp>

namespace opview {

    Face convertCGALFaceToFace(CGALFace triangleVertices) {
        Face face;

        #pragma omp parallel for
        for (int i = 0; i < triangleVertices.size(); i++) {
            face.face[i] = triangleVertices[i]->point();
        }
        return face;
    }

    Face convertCGALFaceToFace(CGALFace triangleVertices, PointD3 oppositeVertex) {
        Face face = convertCGALFaceToFace(triangleVertices);
        face.oppositeVertex = oppositeVertex;
        return face;
    }

    GLMVec3 convertPoinToGLMVec(PointD3 &point)
    {
        return GLMVec3(point.hx(), point.hy(), point.hz());
    }

    CGALVec3 convertPoinToCGALVec(PointD3 &point)
    {
        return CGALVec3(point.hx(), point.hy(), point.hz());
    }

    CGALVec3 convertPoinToCGALVec(GLMVec3 &point)
    {
        return CGALVec3(point.x, point.y, point.z);
    }

    CGALVec3List convertListToCGALVecList(GLMVec3List &inList)
    {
        CGALVec3List outList;
        for (auto el : inList) {
            outList.push_back(convertPoinToCGALVec(el));
        }
        return outList;
    }

    double distanceCGALVec3(CGALVec3 &a, CGALVec3 &b)
    {   
        double distance = 0;
        for (int i = 0; i < a.dimension(); i++) {
            distance += std::sqrt(a[i] - b[i]);
        }
        return distance;
    }

    float deg2rad(float deg)
    {
        return deg * M_PI / 180.0;
    }

    float rad2deg(float rad)
    {
        return rad * 180.0 / M_PI;
    }

    // sigmax = 640 sigmay = 360
    double bivariateGuassian(double x, double y, double centerx, double centery, double sigmax, double sigmay) {
        return std::exp(- 0.5 * std::pow((x-centerx), 2) / std::pow(sigmax, 2) - 0.5 * std::pow((y-centery), 2) / std::pow(sigmay, 2));
    }

} // namespace opview
