#ifndef EVALUATION_CAMERA_POSITION_POINT_CLOUD_INTERSECTER_H_
#define EVALUATION_CAMERA_POSITION_POINT_CLOUD_INTERSECTER_H_

#include <fstream>
#include <iostream>

#include <aliases.h>
#include <OpenMvgParser.hpp>

#include <realtimeMR/types_reconstructor.hpp>

namespace cameval {

    class PointCloudIntersecter {
    public:
        PointCloudIntersecter(SfMData &dataPC1, SfMData &dataPC2);
        PointCloudIntersecter(std::string &filePC1, std::string &filePC2);
        ~PointCloudIntersecter();

        void write(std::string outPC1, std::string outPC2, double threshold=0.8);
        static void write(std::string &file, GLMVec3List &points);    

    protected:
        virtual void intersect(double threshold);
        virtual bool commonFeatures(IntGLMVe2Map map1, IntGLMVe2Map map2, double threshold);
        
    private:
        GLMVec3List pointsPC1;
        GLMVec3List pointsPC2;
        IntGLMVe2MapList point3DTo2DThroughCamPC1;   // maps points 3D to 2D through cam
        IntGLMVe2MapList point3DTo2DThroughCamPC2;   // maps points 3D to 2D through cam

        GLMVec3List pointsPC1Intersected;
        GLMVec3List pointsPC2Intersected;

        void setUpClouds(SfMData &dataPC1, SfMData &dataPC2);

    };

    typedef PointCloudIntersecter* PointCloudIntersecterPtr;

} // namespace cameval

#endif /* EVALUATION_CAMERA_POSITION_POINT_CLOUD_INTERSECTER_H_ */
