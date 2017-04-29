
#include <opview/OptimalViewEstimator.hpp>

namespace opview {

    OptimalViewEstimator::OptimalViewEstimator()
    { /*    */ }

    OptimalViewEstimator::~OptimalViewEstimator()
    { /*    */ }


    GLMVec3List OptimalViewEstimator::estimateOptimalViews(CGALCellList &voxels)
    {
        GLMVec3List camPositions;
        
        for (CGALCell voxel : voxels) {
            GLMVec3List optimalPositions = this->estimateOptimalViews(voxel);
            camPositions.insert(camPositions.end(), optimalPositions.begin(), optimalPositions.end());
        }
        
        return camPositions;
    }

    GLMVec3List OptimalViewEstimator::estimateOptimalViews(CGALCell &voxel)
    {
        GLMVec3List posList;
        FaceList faces = this->extractTrianglesFromCell(voxel);
        
        for (Face face : faces) {
            GLMVec3 camPos = this->determingBestPositionForFace(face);
            posList.push_back(camPos);
        }

        return posList;
    }

    GLMVec3 OptimalViewEstimator::estimateOptimalView(CGALFace &triangleVertices, PointD3 &oppositeVertex)
    {
        Face face = convertCGALFaceToFace(triangleVertices, oppositeVertex);
        return this->determingBestPositionForFace(face);
    }
    

    FaceList OptimalViewEstimator::extractTrianglesFromCells(CGALCellSet &boundaryCells)
    {
        FaceList triangles;

        for (CGALCell cell : boundaryCells) {
            FaceList faces  = extractTrianglesFromCell(cell);
            triangles.insert(triangles.end(), faces.begin(), faces.end());
        }

        return triangles;
    }

    FaceList OptimalViewEstimator::extractTrianglesFromCell(CGALCell &cell)
    {
        FaceList triangles;
        //std::cout << "is a cell? " << this->isCell(cell) << std::endl;
        /*if (!this->isCell(cell)) {
            return FaceList();
        }*/

        for (int faceIndex = 0; faceIndex < 4; faceIndex++) { // For each face in the cell

            // If the face is a boundary face (between the boundary cell and a non manifold cell)
            if (this->isInBoundaries(cell, faceIndex)) {

                CGALFace triangleVertices = faceIndexToVertices(cell, faceIndex);

                if (this->existsSteinerPoint(triangleVertices) > 0) {
                    continue;       // do not add the faces that contains a steiner vertex
                }

                Face face = convertCGALFaceToFace(triangleVertices, cell->vertex(this->oppositeVertex[faceIndex])->point());
                triangles.push_back(face);
            }
        }

        return triangles;
    }

    CGALFace OptimalViewEstimator::faceIndexToVertices(Delaunay3::Cell_handle &cell, int faceIndex) 
    {
        CGALFace vertices;

        for (int i = 0; i < 3; i++) {
            vertices[i] = cell->vertex(this->faceToTriangleMatrix[faceIndex][i]);
        }

        return vertices;
    }

    CGALVec3 OptimalViewEstimator::normalVectorToFace(Face &face)
    {
        // the point opposite to facet
        const PointD3 &p0 = face.oppositeVertex;
        // points on the facet
        const PointD3 &p1 = face.face[0];
        const PointD3 &p2 = face.face[1];
        const PointD3 &p3 = face.face[2];

        CGALVec3 v1 = p1 - p0;
        CGALVec3 n = CGAL::normal(p1, p2, p3);

        if (n * v1 < 0) { 
            n = -n;
        }
        
        return n;
    }

    PointD3 OptimalViewEstimator::barycenterVectorToFace(FaceVertices &face)
    {
        std::vector<std::pair<PointD3, double> > points;
        for (int i = 0; i < face.size(); i++) {
            points.push_back(std::make_pair(face[i], 1.0));   // 1.0 is the weight of the point, it could be changed depending on the accuracy
        }
        return CGAL::barycenter(points.begin(), points.end());
    }


    bool OptimalViewEstimator::isCell(CGALCell &cell)
    {
        return this->delaunayTriang.is_cell(cell);
    }

    bool OptimalViewEstimator::isInBoundaries(CGALCell &cell, int faceIndex)
    {
        auto neighbor = cell->neighbor(faceIndex);
        auto neighInfo = neighbor->info();
        return !neighInfo.getManifoldFlag(); 
    }

    bool OptimalViewEstimator::existsSteinerPoint(CGALFace &triangleVertices)
    {
        for (auto vertex : triangleVertices) {
            if(vertex->info().getPointId() < 0) { 
                return true;
            }
        }
        return false;
    }

    int OptimalViewEstimator::countSteinerPoints(CGALFace &triangleVertices)
    {
        int steinerVertices = 0;
        for (auto vertex : triangleVertices) {
            if(vertex->info().getPointId() < 0) { 
                steinerVertices++;
            }
        }
        return steinerVertices;
    }



} // namespace opview
