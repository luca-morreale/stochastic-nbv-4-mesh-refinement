#include <meshac/FaceAccuracyModel.hpp>

namespace meshac {

    FaceAccuracyModel::FaceAccuracyModel(std::string &meshFile)
    {
        this->meshFilename = meshFile;
        this->faces = generateTriangleList();

        OffParser parser(meshFile);
        this->pointToIndex = parser.getPointToIndexMap();
        this->trianglesIndex = parser.getFaceIndexList();
        
        this->initTree();
    }

    FaceAccuracyModel::~FaceAccuracyModel()
    {
        this->faces.clear();
        this->pointToIndex.clear();
        this->trianglesIndex.clear();
        delete tree;
    }

    PointList FaceAccuracyModel::getPoints()
    {
        IntPointMap indexToPoint = invert(pointToIndex);
        return values(indexToPoint);
    }

    TriangleList FaceAccuracyModel::getFaces()
    {
        return this->faces;
    }

    FaceIndexList FaceAccuracyModel::getFacetsIndex()
    {
        return this->trianglesIndex;
    }

    void FaceAccuracyModel::setMeshFile(std::string meshFile)
    {
        this->meshFilename = meshFile;
        this->faces = generateTriangleList();
    }

    TreePtr FaceAccuracyModel::getTree()
    {
        return tree;
    }

    void FaceAccuracyModel::initTree()
    {
        TriangleList triangles = this->getFaces();
        this->tree = new Tree(triangles.begin(), triangles.end());
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

            if ((p0->point().x() == p1->point().x() && p0->point().y() == p1->point().y() && p0->point().z() == p1->point().z()) || 
                (p0->point().x() == p2->point().x() && p0->point().y() == p2->point().y() && p0->point().z() == p2->point().z()) || 
                (p2->point().x() == p1->point().x() && p2->point().y() == p1->point().y() && p2->point().z() == p1->point().z()) )
                continue;

            if (t.is_degenerate() || TreeKernel().is_degenerate_3_object()(t)) {      // degenerate so not added to tree 
                continue;
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

        poly.normalize_border();

        return poly;
    }

} // namespace meshac
