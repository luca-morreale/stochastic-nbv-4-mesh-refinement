#ifndef MESH_ACCURACY_CR_TUPLES_GENERATOR_H
#define MESH_ACCURACY_CR_TUPLES_GENERATOR_H

#include <manifoldReconstructor/SfMData.h>

#include <meshac/alias_definition.hpp>
#include <meshac/CrossRatioTuple.hpp>
#include <meshac/meshac_type_definition.hpp>
#include <meshac/utilities.hpp>

namespace meshac {

    #define CANNY_RATIO 3
    #define CANNY_LOW_THRESHOLD 100
    #define CANNY_KERNEL_SIZE 3

    class CRTuplesGenerator {
    public:
        CRTuplesGenerator(ImageFileMap &fileMap, GLMListArrayVec2 &camObservations);
        ~CRTuplesGenerator();


        /*
         * Getter and setter for Cameras' Observations.
         */
        GLMListArrayVec2 getCamObservations();
        void setCamObservations(GLMListArrayVec2 &camObservations);
        void setCamObservations(GLMListVec2 &list, int camIndex);
        void updateCamObservations(GLMListVec2 &list, int camIndex);
        void updateCamObservations(GLMListArrayVec2 &camObservations, IntList &camIndexs);


        
        void setImageFileMapping(ImageFileMap &fileMap);
        ImageFileMap getImageFileMapping();


        /*
         * Getter for already computed CrossRatioTupleSet.
         */
        CrossRatioTupleSet getComputedTuples();
        CrossRatioTupleSet getComputedTuplesForCam(int camIndex);
        ListCrossRatioTupleSet getCrossRatioTupleSetList();


        /*
         * Extracts quadruplets of collinear points for each image.
         */
        virtual CrossRatioTupleSet determineTupleOfFourPoints(ImageFileMap &fileMap, GLMListArrayVec2 &camObservations);
        virtual CrossRatioTupleSet determineTupleOfFourPoints();

        /*
         * Extracts quadruplets of collinear points for the image obtained by the given camera.
         */
        virtual CrossRatioTupleSet determineTupleOfFourPointsForCam(ImageFileMap &fileMap, GLMListArrayVec2 &camObservations, int camIndex);
        virtual CrossRatioTupleSet determineTupleOfFourPointsForCam(int camIndex);
        virtual ListCrossRatioTupleSet determineTupleOfFourPointsForAllCam();


        
    protected:
        void setTuples(CrossRatioTupleSet &tupleSet);
        void setTuplesPerCam(ListCrossRatioTupleSet &tupleSetPerCam);
        void setTuplesPerCam(CrossRatioTupleSet &tupleSet, int camIndex);

    private:

        CrossRatioTupleSet createsTuples(IntArrayList &combos, IntList &pointSet, GLMListVec2 &points2D);
        void computeEdges(int camIndex, CVMat &edges);
        CVListVec2 createLinesFromPoints(CVMat &edges);
        IntArrayList generateCorrespondances(CVListVec2 &lines, GLMListVec2 &points2D);

        CrossRatioTupleSet collapseListSet(ListCrossRatioTupleSet &tupleSetPerCam);

        ImageFileMap fileMap;
        GLMListArrayVec2 camObservations;

        ListCrossRatioTupleSet tupleSetPerCam;
        CrossRatioTupleSet tupleSet;

    };

    typedef CRTuplesGenerator * CRTuplesGeneratorPtr;
    

} // namespace meshac

#endif // MESH_ACCURACY_CR_TUPLES_GENERATOR_H
