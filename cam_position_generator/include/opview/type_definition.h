#ifndef CAM_POSITION_GENERATOR_TYPE_DEFINITION_H
#define CAM_POSITION_GENERATOR_TYPE_DEFINITION_H

#include <map>

#include <opengm/functions/sparsemarray.hxx>
#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/graphicalmodel/space/simplediscretespace.hxx>
#include <opengm/inference/alphabetaswap.hxx>
#include <opengm/inference/alphaexpansion.hxx>
#include <opengm/inference/auxiliary/minstcutkolmogorov.hxx>
#include <opengm/inference/graphcut.hxx>
#include <opengm/inference/icm.hxx>
#include <opengm/inference/lazyflipper.hxx>
#include <opengm/inference/loc.hxx>
#include <opengm/operations/adder.hxx>
#include <opengm/operations/multiplier.hxx>
#include <opengm/inference/listbruteforce.hxx>

#include <opview/alias_definition.h>
#include <opview/DimensionDisagreementLists.hpp>

namespace opview {

    struct Face {
        FaceVertices face;
        PointD3 oppositeVertex;
    };

    typedef std::vector<Face> FaceList;

    typedef double LabelType;
    typedef size_t VariableIndexType;

    typedef std::vector<LabelType> LabelList;
    typedef std::vector<size_t> VarIndexList;


    typedef opengm::DiscreteSpace<> SimpleSpace;
    typedef opengm::ExplicitFunction<LabelType, size_t, VariableIndexType> GMExplicitFunction;
    typedef opengm::SparseFunction<LabelType, VariableIndexType, LabelType> GMSparseFunction;
    typedef OPENGM_TYPELIST_2(GMExplicitFunction, GMSparseFunction) FunctionTypeList;
    typedef opengm::GraphicalModel<LabelType, opengm::Adder, FunctionTypeList, SimpleSpace> GraphicalModelAdder;
    typedef opengm::GraphicalModel<LabelType, opengm::Multiplier, FunctionTypeList, SimpleSpace> GraphicalModelMultiplier;

    typedef std::vector<GMSparseFunction> GMSparseFunctionList;
    typedef std::vector<GMExplicitFunction> GMExplicitFunctionList;
    
    typedef GraphicalModelAdder::FunctionIdentifier GMAdderFID;
    typedef GraphicalModelMultiplier::FunctionIdentifier GMMultFID;

    typedef SimpleSpace * SimpleSpacePtr;
    typedef GraphicalModelAdder * GraphicalModelAdderPtr;
    typedef GraphicalModelMultiplier * GraphicalModelMultiplierPtr;

    typedef opengm::Inference<GraphicalModelAdder, opengm::Maximizer> AdderInference;
    typedef opengm::Inference<GraphicalModelMultiplier, opengm::Maximizer> MultiplierInference;
    typedef opengm::external::MinSTCutKolmogorov<size_t, double> MinStCutType;
    typedef opengm::GraphCut<GraphicalModelAdder, opengm::Maximizer, MinStCutType> MinGraphCut;
    typedef opengm::AlphaExpansion<GraphicalModelAdder, MinGraphCut> MinAlphaExpansion;
    typedef opengm::AlphaBetaSwap<GraphicalModelAdder, MinGraphCut> MinAlphaBetaSwap;
    typedef opengm::ICM<GraphicalModelAdder, opengm::Maximizer> ICM;
    typedef opengm::LazyFlipper<GraphicalModelAdder, opengm::Maximizer> LazyFlipper;
    typedef LazyFlipper::Parameter LazyFlipperParameter;
    typedef opengm::LOC<GraphicalModelAdder, opengm::Maximizer> LOC;
    typedef opengm::Bruteforce<GraphicalModelAdder, opengm::Maximizer> Bruteforce;
    typedef opengm::MultiBruteforce<GraphicalModelAdder, opengm::Maximizer> MultiBruteforce;


    typedef AdderInference* AdderInferencePtr;
    typedef MultiplierInference* MultiplierInferencePtr;
    typedef MinAlphaExpansion* MinAlphaExpansionPtr;
    typedef MinAlphaBetaSwap* MinAlphaBetaSwapPtr;
    typedef ICM* ICMPtr;
    typedef LazyFlipper* LazyFlipperPtr;
    typedef LOC* LOCPtr;
    typedef Bruteforce* BruteforcePtr;
    typedef MultiBruteforce* MultiBruteforcePtr;
    

    typedef struct VonMisesConfiguration {
        double goalAngle;
        double dispersion;

    public:
        VonMisesConfiguration(double goalAngle, double dispersion) : goalAngle(goalAngle), dispersion(dispersion) { }
        VonMisesConfiguration() : goalAngle(45), dispersion(0) { }
    } VonMisesConfiguration;
    typedef VonMisesConfiguration * VonMisesConfigurationPtr;

    typedef struct HierarchicalDiscretizationConfiguration {
        size_t depth;
        size_t labels;

    public:
        HierarchicalDiscretizationConfiguration(size_t depth, size_t labels) : depth(depth), labels(labels) { }
        HierarchicalDiscretizationConfiguration() : depth(5), labels(10) { }

    } HierarchicalDiscretizationConfiguration;
    typedef HierarchicalDiscretizationConfiguration* HierarchicalDiscretizationConfigPtr;

    typedef struct OrientationHierarchicalConfiguration {
        HierarchicalDiscretizationConfiguration config;
        float deltaAngle;
        FloatList deltaAngles;

    public:
        OrientationHierarchicalConfiguration(size_t depth, size_t labels, float deltaAngle) : deltaAngle(deltaAngle)
        {
            config = {depth, labels};
            deltaAngles.push_back(deltaAngle);
        }
        OrientationHierarchicalConfiguration(size_t depth, size_t labels, FloatList deltaAngles) : deltaAngles(deltaAngles)
        {
            config = {depth, labels};
            deltaAngle = deltaAngles[deltaAngles.size() - 1];
        }
        OrientationHierarchicalConfiguration() : deltaAngle(10)
        {
            config = HierarchicalDiscretizationConfiguration();
        }

    } OrientationHierarchicalConfiguration;
    typedef OrientationHierarchicalConfiguration* OrientationHierarchicalConfigPtr;

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

    typedef struct MCConfiguration {
        size_t resamplingNum;
        size_t particles;
        size_t particleUniform;

    public:
        MCConfiguration(size_t resamplingNum, size_t particles, size_t particleUniform) 
                        : resamplingNum(resamplingNum), particles(particles), particleUniform(particleUniform)
        { /*    */ }
        MCConfiguration() : resamplingNum(10), particles(1000), particleUniform(10)
        { /*    */ }
        
    } MCConfiguration;
    typedef MCConfiguration* MCConfigurationPtr;

}

#endif // CAM_POSITION_GENERATOR_TYPE_DEFINITION_H
