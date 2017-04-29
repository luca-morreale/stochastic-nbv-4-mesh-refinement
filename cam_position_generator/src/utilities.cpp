
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

} // namespace opview
