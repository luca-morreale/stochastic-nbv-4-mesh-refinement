#include <meshac/FaceAccuracyModel.hpp>

namespace meshac {

    FaceAccuracyModel::FaceAccuracyModel(std::string &meshFile)
    {
        this->meshFilename = meshFile;
        this->faces = generateTriangleList();        
    }

    FaceAccuracyModel::~FaceAccuracyModel()
    {
        this->faces.clear();        
    }

    TriangleList FaceAccuracyModel::getFaces()
    {
        return this->faces;
    }

    void FaceAccuracyModel::changeMesh(std::string meshFile)
    {
        this->meshFilename = meshFile;
        this->faces = generateTriangleList();
    }

    TriangleList FaceAccuracyModel::generateTriangleList()
    {
        Polyhedron poly = extractPolyhedron();

        TriangleList triangles;
        for (Facet_iterator it = poly.facets_begin(); it != poly.facets_end(); it++) {
            Halfedge_facet_circulator p = it->facet_begin();
            Vertex p0 = p->vertex();
            Vertex p1 = (++p)->vertex();
            Vertex p2 = (++p)->vertex();

            Triangle t = Triangle(p0->point(), p1->point(), p2->point());
            if (t.is_degenerate()) {
                throw std::runtime_error("A triangle is degenerate.");
            }
            triangles.push_back(t);
        }

        return triangles;
    }

    Polyhedron FaceAccuracyModel::extractPolyhedron()
    {
        std::ifstream meshIn(meshFilename);
        Polyhedron poly;
        meshIn >> poly;
        meshIn.close();

        return poly;
    }

} // namespace meshac
