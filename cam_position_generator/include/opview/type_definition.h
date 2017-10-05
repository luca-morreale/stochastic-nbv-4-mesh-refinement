#ifndef CAM_POSITION_GENERATOR_TYPE_DEFINITION_H
#define CAM_POSITION_GENERATOR_TYPE_DEFINITION_H

#include <map>

#include <opview/alias_definition.h>
#include <opview/DimensionDisagreementLists.hpp>

namespace opview {

    typedef struct VonMisesConfiguration {
        double goalAngle;
        double dispersion;

    public:
        VonMisesConfiguration(double goalAngle, double dispersion) : goalAngle(goalAngle), dispersion(dispersion) { }
        VonMisesConfiguration() : goalAngle(0.785398f), dispersion(0) { } // 45 deg
    } VonMisesConfiguration;
    typedef VonMisesConfiguration * VonMisesConfigurationPtr;

    /*** Structs for the optimization algos informations ***/

    typedef struct SpaceBounds {
        GLMVec3 lower;
        GLMVec3 upper;

        SpaceBounds(GLMVec3 lowerBounds, GLMVec3 upperBounds) : lower(lowerBounds), upper(upperBounds)
        { /*    */ }

        SpaceBounds() : lower(GLMVec3(-10, -10, -10)), upper(GLMVec3(10, 10, 10))
        { /*    */ }

    } SpaceBounds;
    typedef SpaceBounds* SpaceBoundsPtr;


    typedef struct HierarchicalDiscretizationConfiguration {
        size_t depth;
        size_t labels;
        SpaceBounds bounds;

    public:
        HierarchicalDiscretizationConfiguration(size_t depth, size_t labels, SpaceBounds &bounds) 
                        : depth(depth), labels(labels), bounds(bounds) { }
        HierarchicalDiscretizationConfiguration() : depth(5), labels(10) { }

    } HierarchicalDiscretizationConfiguration;
    typedef HierarchicalDiscretizationConfiguration* HierarchicalDiscretizationConfigPtr;


    typedef struct OrientationHierarchicalConfiguration {
        HierarchicalDiscretizationConfiguration config;
        float deltaAngle;
        FloatList deltaAngles;

    public:
        OrientationHierarchicalConfiguration(size_t depth, size_t labels, SpaceBounds &bounds, FloatList deltaAngles) : deltaAngles(deltaAngles)
        {
            config = {depth, labels, bounds};
            deltaAngle = deltaAngles[deltaAngles.size() - 1];
        }
        OrientationHierarchicalConfiguration(HierarchicalDiscretizationConfiguration &hconfig, FloatList deltaAngles) : config(hconfig), deltaAngles(deltaAngles)
        {
            deltaAngle = deltaAngles[deltaAngles.size() - 1];
        }
        OrientationHierarchicalConfiguration() : deltaAngle(10)
        {
            config = HierarchicalDiscretizationConfiguration();
        }

    } OrientationHierarchicalConfiguration;
    typedef OrientationHierarchicalConfiguration* OrientationHierarchicalConfigPtr;


    typedef struct ParticlesInformation {
        size_t num;
        size_t uniform;
        int deltaDegree;

        ParticlesInformation(size_t particles, size_t particleUniform, int deltaDegree) 
                    : num(particles), uniform(particleUniform), deltaDegree(deltaDegree)
        { /*    */ }
        ParticlesInformation() : num(1000), uniform(10), deltaDegree(45)
        { /*    */ }

    } ParticlesInformation;
    typedef ParticlesInformation* ParticlesInformationPtr;

    
    typedef struct StochasticConfiguration {
        ParticlesInformation particles;
        size_t resamplingNum;
        SpaceBounds bounds;

    public:
        StochasticConfiguration(size_t resamplingNum, ParticlesInformation &particles, SpaceBounds &bounds) 
                        : resamplingNum(resamplingNum), particles(particles), bounds(bounds)
        { /*    */ }
        StochasticConfiguration() : resamplingNum(10)
        { /*    */ }
        
    } StochasticConfiguration;
    typedef StochasticConfiguration* StochasticConfigurationPtr;



    /*** Structs for the camera and mesh information ***/

    typedef struct Camera {
        CameraMatrix P;
        CameraMatrix R;
        CameraMatrix E;
        CameraMatrix K;
        GLMVec4 t;

        Camera(CameraMatrix R, CameraMatrix K, GLMVec4 t) : R(R), K(K)
        {
            E = R;
            E[0][3] = t.x;
            E[1][3] = t.y;
            E[2][3] = t.z;
            E[3][3] = 1.0f;
            P = E * K;
            this->t = - R * t;
        }
    } Camera;

    typedef std::vector<Camera> CameraList;

    typedef struct CameraGeneralConfiguration {
        float f;
        int size_x;
        int size_y;

    public:
        CameraGeneralConfiguration(int size_x, int size_y, double f) : size_x(size_x), size_y(size_y), f(f)
        { /*    */ }

        CameraGeneralConfiguration() : size_x(1920), size_y(1080), f(959.9965)
        { /*    */ }

    } CameraGeneralConfiguration;
    typedef CameraGeneralConfiguration* CameraGeneralConfigPtr;


    typedef struct MeshConfiguration {
        std::string filename;
        GLMVec3List cams;
        GLMVec3List points;
        GLMVec3List normals;
        DoubleList uncertainty;

    public:
        MeshConfiguration(std::string &filename, GLMVec3List &cams, GLMVec3List &points, GLMVec3List &normals, DoubleList &uncertainty)
                            : filename(filename), cams(cams), points(points), normals(normals), uncertainty(uncertainty)
        {
            if (points.size() != uncertainty.size()) {
                throw DimensionDisagreementLists("Points and Accuracy lists should have the same size. " + 
                                    std::to_string(points.size()) + " != " + std::to_string(uncertainty.size()) + "\n");
            }
            if (points.size() != normals.size()) {
                throw DimensionDisagreementLists("Points and Normals lists should have the same size. " + 
                                    std::to_string(points.size()) + " != " + std::to_string(normals.size()) + "\n");
            }
        }
        MeshConfiguration(std::string &filename, GLMVec3List &cams)
                            : filename(filename), cams(cams), points(GLMVec3List()), normals(GLMVec3List()), uncertainty(DoubleList())
        { /*    */ }
    } MeshConfiguration;
    typedef MeshConfiguration* MeshConfigurationPtr;


    typedef boost::function<double(EigVector5 &, GLMVec3 &, GLMVec3 &, CameraGeneralConfiguration &, TreePtr)> BoostObjFunction;
}

#endif // CAM_POSITION_GENERATOR_TYPE_DEFINITION_H
