#ifndef CAM_POSITION_GENERATOR_OPTIMAL_VIEW_ESTIMATOR_H
#define CAM_POSITION_GENERATOR_OPTIMAL_VIEW_ESTIMATOR_H

#include <CGAL/barycenter.h>

#include <opview/alias_definition.h>
#include <opview/type_definition.h>
#include <opview/utilities.hpp>

namespace opview{

    class OptimalViewEstimator {
    public:
        OptimalViewEstimator();
        ~OptimalViewEstimator();

        virtual GLMVec3List estimateOptimalViews(CGALCell &voxel);
        virtual GLMVec3List estimateOptimalViews(CGALCellList &voxels);
        virtual GLMVec3 estimateOptimalView(CGALFace &triangleVertices, PointD3 &oppositeVertex);

    protected:

        virtual GLMVec3 determingBestPositionForFace(Face &face) = 0;

        virtual FaceList extractTrianglesFromCells(CGALCellSet &boundaryCells);
        virtual FaceList extractTrianglesFromCell(CGALCell &cell);
        virtual CGALFace faceIndexToVertices(CGALCell &cell, int faceIndex);

        virtual CGALVec3 normalVectorToFace(Face &face);
        virtual PointD3 barycenterVectorToFace(FaceVertices &face);  // should produce another function for weighted points

    private:
        bool isCell(CGALCell &cell);
        bool isInBoundaries(CGALCell &cell, int faceIndex);
        bool existsSteinerPoint(CGALFace &triangleVertices);
        int countSteinerPoints(CGALFace &triangleVertices);


        const int faceToTriangleMatrix[4][3] = { 
                { 3, 2, 1 },    // faceIndex : 0
                { 0, 2, 3 },    // faceIndex : 1
                { 3, 1, 0 },    // faceIndex : 2
                { 0, 1, 2 }     // faceIndex : 3
        };
        const int oppositeVertex[4] = { 0, 1, 2, 3};

        Delaunay3 delaunayTriang;
    };

    typedef OptimalViewEstimator* OptimalViewEstimatorPtr;

}

#endif // CAM_POSITION_GENERATOR_OPTIMAL_VIEW_ESTIMATOR_H
