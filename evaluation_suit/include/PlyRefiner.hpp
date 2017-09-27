#ifndef EVALUATION_CAMERA_POSITION_PLY_REFINER_H_
#define EVALUATION_CAMERA_POSITION_PLY_REFINER_H_

#include <fstream>
#include <iostream>

#include <aliases.h>
#include <OpenMvgParser.hpp>

namespace cameval {

    class PlyRefiner {
    public:
        PlyRefiner(std::string &jsonfile, GLMVec3 &color, float outlierThreshold=0.25);
        ~PlyRefiner();

        void load(std::string &file);
        void write(std::string &file, bool colors=true);        
        void filterOutliers();

    protected:
        int getPointsCount();

        virtual bool isInlier(int pointIndex);
        virtual int GaussNewton(const CVMatList &cameras, const CVPoint2fList &points, CVPoint3f init3Dpoint, CVPoint3f &optimizedPoint, const float outlierThreshold);
        virtual int point2D3DJacobian(const CVMatList &cameras, const CVMat &cur3Dpoint, CVMat &J, CVMat &hessian);

    private:
        GLMVec3 pointColor;
        BoolList inliers;
        float outlierThreshold;

        GLMVec3List points;                           // list of 3D points
        GLMMat4List cameras;
        IntArrayList camViewingPoint;         // each list contains the index of camera that see that point
        IntArrayList pointsVisibleFromCamN_;    // each list contains the index of points that are seen from that cam
        std::vector<GLMVec2List> point2DoncamViewingPoint;   // a list for each 3D point, each one contains the 2D observations

    };

} // namespace cameval

#endif  // EVALUATION_CAMERA_POSITION_PLY_REFINER_H_
