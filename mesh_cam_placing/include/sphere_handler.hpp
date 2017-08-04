#ifndef CAM_PLACING_SPHERE_READER_H
#define CAM_PLACING_SPHERE_READER_H

#include <aliases.h>
#include <iostream>
#include <fstream>

namespace camplacing {

    std::pair<GLMVec3List, TriangularFacetList> readSphere();

    void printSpherePoints(std::ofstream &off, GLMVec3List &sphere, GLMVec3 &poi);
    void printSphereFacets(std::ofstream &off, TriangularFacetList &list, int offset);

} // namespace camplacing

#endif // CAM_PLACING_SPHERE_READER_H