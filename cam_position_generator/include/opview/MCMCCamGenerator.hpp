#ifndef CAM_POSITION_GENERATOR_MCMC_CAM_GENERATOR_H
#define CAM_POSITION_GENERATOR_MCMC_CAM_GENERATOR_H

#include <glm/gtx/norm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/transform.hpp>

#include <opview/type_definition.h>
#include <opview/alias_definition.h>
#include <opview/MCMCSamplerGenerator.hpp>
#include <opview/utilities.hpp>

namespace opview {

    #define OFFSPRING 0.1   // 10% of the total
    #define ORIENTATION 10
    #define orientationCycles() for(int ptc = 0; ptc < 360; ptc+=ORIENTATION) for(int yaw = 0; yaw < 360; yaw+=ORIENTATION)

    #define BD_AERIAL 0.2f
    #define BD_TERRESTRIAL_PROSPECTIVE 0.25f
    #define BD_TERRESTRIAL_ARCHITECTURAL 0.33f 

    class MCMCCamGenerator {
    public:
        MCMCCamGenerator(CameraGeneralConfiguration &camConfig, std::string &meshFile, GLMVec3List &cams, 
                                            MCConfiguration &mcConfig, double goalAngle=55, double dispersion=8);

        ~MCMCCamGenerator();

        virtual void estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector);

    protected:
        virtual void initLambdas();
        virtual EigVector5List insertOrientation(GLMVec3List &points);
        virtual OrderedPose uniformMCStep(GLMVec3 &centroid, GLMVec3 &normVector);
        virtual OrderedPose resamplingMCStep(GLMVec3 &centroid, GLMVec3 &normVector, OrderedPose &currentOptima);
        virtual OrderedPose generalStep(GLMVec3 &centroid, GLMVec3 &normVector, OrderedPose &currentOptima, LambdaGLMPointsList &getPoints);
        virtual OrderedPose orderPoses(EigVector5List &orientedPoints, DoubleList &values);
        virtual void computeObjectiveFunction(DoubleList &values, EigVector5List &points, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual double logVonMisesWrapper(EigVector5 &newCamPose, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void computeProjectionFunction(DoubleList &values, EigVector5List &points, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void computeVisibilityFunction(DoubleList &values, EigVector5List &points, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void computeConstraintFunction(DoubleList &values, EigVector5List &points, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual double computeBDConstraint(EigVector5 &newCamPose, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual OrderedPose extractBestResults(OrderedPose &poses);
        virtual GLMVec3List getCentersFromOptima(OrderedPose currentOptima);
        virtual DoubleList getWeightsFromOptima(OrderedPose currentOptima);

        virtual LabelType visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);
        virtual LabelType imageProjectionDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);
        virtual bool isMeaningfulPose(EigVector5 &pose, GLMVec3 &centroid);
        virtual bool isOppositeView(EigVector5 &pose, GLMVec3 &centroid);
        virtual bool isIntersecting(EigVector5 &pose, GLMVec3 &centroid);
        virtual bool isPointInsideImage(EigVector5 &pose, GLMVec3 &centroid);
        virtual GLMVec2 getProjectedPoint(EigVector5 &pose, GLMVec3 &centroid);
        virtual RotationMatrix getRotationMatrix(float roll, float pitch, float yaw);
        virtual CameraMatrix getCameraMatrix(EigVector5 &pose);

        LambdaGLMPointsList uniformPointGetter;
        LambdaGLMPointsList resamplingPointGetter;

        LambdaFloat offsetX = [](){ return -4.0; };
        LambdaFloat offsetY = [](){ return -4.0; };
        LambdaFloat offsetZ = [](){ return 1.0; };

    private:
        MCMCSamplerGeneratorPtr sampler;
        MCConfiguration mcConfig;
        GLMVec3List cams;
        VonMisesConfiguration vonMisesConfig;
        std::string meshFile;

        float deltaAngle;
        const GLMVec3 zdir = GLMVec3(0.0, 0.0, 1.0);
        std::string meshFilename;
        TreePtr tree;

        CameraGeneralConfiguration camConfig;

        void fillTree();
        Polyhedron extractPolyhedron();
        TriangleList getTriangleList(Polyhedron &poly);

    };

} // namespace opview

#endif // CAM_POSITION_GENERATOR_MCMC_CAM_GENERATOR_H
