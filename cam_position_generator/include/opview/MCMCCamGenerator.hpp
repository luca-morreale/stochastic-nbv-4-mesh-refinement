#ifndef CAM_POSITION_GENERATOR_MCMC_CAM_GENERATOR_H
#define CAM_POSITION_GENERATOR_MCMC_CAM_GENERATOR_H

#include <glm/gtx/norm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/transform.hpp>

#include <opview/type_definition.h>
#include <opview/alias_definition.h>
#include <opview/GaussianSampleGenerator.hpp>
#include <opview/utilities.hpp>
#include <opview/ExhaustiveReportWriter.hpp>

namespace opview {

    #define OFFSPRING 0.1   // 10% of the total
    #define orientationCycles(delta) for(int ptc = 0; ptc < 360; ptc+=delta) for(int yaw = 0; yaw < 360; yaw+=delta)

    #define BD_AERIAL 0.2f
    #define BD_TERRESTRIAL_PROSPECTIVE 0.25f
    #define BD_TERRESTRIAL_ARCHITECTURAL 0.33f 

    class MCMCCamGenerator {
    public:
        MCMCCamGenerator(CameraGeneralConfiguration &camConfig, std::string &meshFile, GLMVec3List &cams, 
                                            MCConfiguration &mcConfig, double goalAngle=45, double dispersion=8);

        ~MCMCCamGenerator();

        virtual void estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector);

    protected:
        virtual EigVector5List insertOrientation(GLMVec3List &points);
        virtual OrderedPose uniformMCStep(GLMVec3 &centroid, GLMVec3 &normVector, int round);
        virtual OrderedPose resamplingStep(GLMVec3 &centroid, GLMVec3 &normVector, OrderedPose &currentOptima, int round);
        virtual OrderedPose generalStep(GLMVec3 &centroid, GLMVec3 &normVector, EigVector5List &orientedPoints);
        virtual OrderedPose orderPoses(EigVector5List &orientedPoints, DoubleList &values);
        virtual void computeObjectiveFunction(DoubleList &values, EigVector5List &points, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual double logVonMisesWrapper(EigVector5 &newCamPose, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void computeProjectionFunction(DoubleList &values, EigVector5List &points, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void computeVisibilityFunction(DoubleList &values, EigVector5List &points, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void computeConstraintFunction(DoubleList &values, EigVector5List &points, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual double computeBDConstraint(EigVector5 &newCamPose, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual OrderedPose extractBestResults(OrderedPose &poses, int round);
        virtual EigVector5List getCentersFromOptima(OrderedPose currentOptima);
        virtual DoubleList getWeightsFromOptima(OrderedPose currentOptima);

        virtual LabelType visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);
        virtual LabelType imageProjectionDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);
        virtual bool isMeaningfulPose(EigVector5 &pose, GLMVec3 &centroid);
        virtual bool isOppositeView(EigVector5 &pose, GLMVec3 &centroid);
        virtual bool isIntersecting(EigVector5 &pose, GLMVec3 &centroid);
        virtual bool isPointInsideImage(EigVector5 &pose, GLMVec3 &centroid);
        virtual GLMVec2 getProjectedPoint(EigVector5 &pose, GLMVec3 &centroid);
        virtual RotationMatrix getRotationMatrix(float roll, float pitch, float yaw);
        virtual CameraMatrix getExtrinsicMatrix(EigVector5 &pose);
        virtual CameraMatrix getCameraMatrix(EigVector5 &pose);

        bool isMathemathicalError(Segment_intersection &intersection, Point &point);

        virtual EigVector5List uniformPointsGetter();
        virtual EigVector5List resamplingPointsGetter(OrderedPose &currentOptima);

        virtual void sumUpAll(DoubleList &dest, DoubleList &visibility, DoubleList &vonMises, DoubleList &projection, DoubleList &constraints);
        
        MCConfiguration getMCConfiguration();
        VonMisesConfiguration getVonMisesConfiguration();
        CameraGeneralConfiguration getCameraConfiguration();
        
        ReportWriterPtr getLogger();
        void setLogger(ReportWriterPtr log);

        LambdaFloat offsetX = [](){ return -1.0; };
        LambdaFloat offsetY = [](){ return -1.0; };
        LambdaFloat offsetZ = [](){ return -1.0; };

    private:
        GaussianSampleGeneratorPtr sampler;
        MCConfiguration mcConfig;
        GLMVec3List cams;
        VonMisesConfiguration vonMisesConfig;
        std::string meshFile;

        float deltaAngle;
        const GLMVec4 zdir = GLMVec4(0.0f, 0.0f, 1.0f, 0.0f);
        std::string meshFilename;
        TreePtr tree;

        CameraGeneralConfiguration camConfig;

        ExhaustiveReportWriterPtr log;

        void fillTree();
        Polyhedron extractPolyhedron();
        TriangleList getTriangleList(Polyhedron &poly);
        OrderedPose convertAngles(OrderedPose poses);

    };

} // namespace opview

#endif // CAM_POSITION_GENERATOR_MCMC_CAM_GENERATOR_H
