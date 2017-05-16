#ifndef MESH_ACCURACY_TYPE_DEFINITION_H
#define MESH_ACCURACY_TYPE_DEFINITION_H


namespace meshac {

    const float SENSIBILITY = 0.001f;

    struct Face {
        FaceVertices face;
        PointD3 oppositeVertex;
    };

    typedef std::vector<Face> FaceList;
    
} // namesace meshac

#endif // MESH_ACCURACY_TYPE_DEFINITION_H
