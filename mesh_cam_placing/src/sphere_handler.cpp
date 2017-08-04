#include <sphere_handler.hpp>

namespace camplacing {

    std::pair<GLMVec3List, TriangularFacetList> readSphere()
    {
        GLMVec3List pointsList;
        TriangularFacetList facetsList;

        std::ifstream cin("sphere_colored.off");
        std::string voidString;
        int points, facets, voidInt;
        double x, y, z;
        double r, g, b, a;
        cin >> voidString;

        cin >> points >> facets >> voidInt;

        for (int i = 0; i < points; i++) {
            cin >> x >> y >> z;
            pointsList.push_back(GLMVec3(x, y, z));
        }

        for (int i = 0; i < facets; i++) {
            cin >> voidInt >> x >> y >> z >> r >> g >> b >> a;

            facetsList.push_back((TriangularFacet){(int)x, (int)y, (int)z, r, g, b, a});
        }

        return std::make_pair(pointsList, facetsList);
    }

    void printSpherePoints(std::ofstream &off, GLMVec3List &sphere, GLMVec3 &poi)
    {
        for (int i = 0; i < sphere.size(); i++) {
            sphere[i] += poi;
            off << sphere[i].x << "  " << sphere[i].y << "  " << sphere[i].z << std::endl;
        }
    }

    void printSphereFacets(std::ofstream &off, TriangularFacetList &list, int offset)
    {
        for (int i = 0; i < list.size(); i++) {
            off << "3  " << list[i].x + offset << "  " << list[i].y+offset << "  " << list[i].z+offset << "  ";
            off << list[i].r << " " << list[i].g << " " << list[i].b << " " << list[i].a << " " << std::endl;
        
        }
    }

}