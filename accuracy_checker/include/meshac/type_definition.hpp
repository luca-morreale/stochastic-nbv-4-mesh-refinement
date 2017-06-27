#ifndef MESH_ACCURACY_TYPE_DEFINITION_H
#define MESH_ACCURACY_TYPE_DEFINITION_H

#include <vector>

namespace meshac {

    const float SENSIBILITY = 0.001f;

    typedef std::array<size_t, 3> SizeT3Array;

    typedef struct FaceIndex {
        SizeT3Array vs;

        FaceIndex() { /*    */ }
        FaceIndex(size_t x, size_t y, size_t z)
        {
            vs[0] = x; vs[1] = y; vs[2] = z;
        }

        void set(int pos, size_t index)
        {
            vs[pos] = index;
        }

    } FaceIndex;

    typedef std::vector<FaceIndex> FaceIndexList;
    
} // namesace meshac

#endif // MESH_ACCURACY_TYPE_DEFINITION_H
