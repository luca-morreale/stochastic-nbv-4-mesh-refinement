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

        bool is(int a, int b, int c)
        {
            return (a == vs[0] && b == vs[1] && c == vs[2]) ||
                (b == vs[0] && c == vs[1] && a == vs[2]) ||
                (c == vs[0] && a == vs[1] && b == vs[2]);
        }

    } FaceIndex;

    typedef std::vector<FaceIndex> FaceIndexList;
    
} // namesace meshac

#endif // MESH_ACCURACY_TYPE_DEFINITION_H
