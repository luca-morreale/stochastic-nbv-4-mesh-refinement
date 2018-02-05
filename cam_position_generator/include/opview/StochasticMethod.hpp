#ifndef CAM_POSITION_GENERATOR_STOCHASTIC_METHOD_H
#define CAM_POSITION_GENERATOR_STOCHASTIC_METHOD_H

#include <opview/alias_definition.h>
#include <opview/ExhaustiveReportWriter.hpp>
#include <opview/Formulation.hpp>
#include <opview/GaussianSampleGenerator.hpp>
#include <opview/orientation_utilities.hpp>
#include <opview/type_definition.h>
#include <opview/utilities.hpp>

namespace opview {

    #define orientationCycles(delta) for(int yaw = 0; yaw < 360; yaw+=delta)

    
    class StochasticMethod {
    public:
        StochasticMethod(CameraGeneralConfiguration &camConfig, std::string &meshFile, GLMVec3List &cams, 
                                    StochasticConfiguration &stoConfig, double offspring=0.9, double goalAngle=45, double dispersion=5);

        ~StochasticMethod();

        virtual DoubleList estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector) = 0;

        CameraGeneralConfiguration getCamConfig();
        GLMVec3List getCams();
        StochasticConfiguration getStochasticConfig();
        VonMisesConfiguration getVonMisesConfig();
        ExhaustiveReportWriterPtr getLogger();
        void setLogger(ExhaustiveReportWriterPtr log);
        GLMVec3 lowerBounds();
        GLMVec3 upperBounds();
        size_t getUniformParticles();
        size_t getResamplingParticles();
        size_t getResamplingSteps();

        
    protected:

        virtual void createTree();
        TreePtr getTree();
        FormulationPtr getFormulation();

        virtual OrderedPose uniformSamplingStep(GLMVec3 &centroid, GLMVec3 &normVector, int round);
        virtual EigVector5List uniformPointsGetter();

        virtual OrderedPose computeEnergyForPoses(EigVector5List &orientedPoints, GLMVec3 &centroid, GLMVec3 &normVector);
        virtual OrderedPose extractBestResults(OrderedPose &poses, int round);
        OrderedPose orderPoses(EigVector5List &orientedPoints, DoubleList &values);
        EigVector5List insertOrientation(GLMVec3List &points);
        

        void setOffspring(double offspring);
        double getOffspring();

        float offsetX();
        float offsetY();
        float offsetZ();

        float pitch;
        
    private:
        CameraGeneralConfiguration camConfig;
        GLMVec3List cams;
        std::string meshFile;
        StochasticConfiguration stoConfig;
        TreePtr tree;
        VonMisesConfiguration vonMisesConfig;

        FormulationPtr formulation;
        ExhaustiveReportWriterPtr log;

        double OFFSPRING;

        
        OrderedPose convertAngles(OrderedPose poses);
        
        Polyhedron extractPolyhedron();
        TriangleList getTriangleList(Polyhedron &poly);
        
    };

} // namespace opview

#endif // CAM_POSITION_GENERATOR_STOCHASTIC_METHOD_H
