
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



} // namespace opview
