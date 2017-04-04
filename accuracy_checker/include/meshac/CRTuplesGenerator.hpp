#ifndef MESH_ACCURACY_CR_TUPLES_GENERATOR_H
#define MESH_ACCURACY_CR_TUPLES_GENERATOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <meshac/alias_definition.hpp>
#include <meshac/CrossRatioTuple.hpp>
#include <meshac/meshac_type_definition.hpp>
#include <meshac/utilities.hpp>

namespace meshac {

    #define CANNY_RATIO 3
    #define CANNY_LOW_THRESHOLD 100
    #define CANNY_KERNEL_SIZE 3
    #define SKIP_RATE 0.8

    class CRTuplesGenerator {
    public:
        CRTuplesGenerator(StringList &fileList, GLMListArrayVec2 &camObservations);
        ~CRTuplesGenerator();

        /*
         * Getter and setter for Cameras' Observations.
         */
        GLMListArrayVec2 getCamObservations();
        void setCamObservations(GLMListArrayVec2 &camObservations);
        void setCamObservations(GLMListVec2 &list, int camIndex);
        void updateCamObservations(GLMListVec2 &list, int camIndex);
        void updateCamObservations(GLMListArrayVec2 &camObservations, IntList &camIndexs);


        
        void setFileList(StringList &fileList);
        StringList getFileList();


        /*
         * Getter for already computed CrossRatioTupleSet.
         */
        CrossRatioTupleSet getComputedTuples();
        CrossRatioTupleSet getComputedTuplesForCam(int camIndex);
        ListCrossRatioTupleSet getCrossRatioTupleSetList();


        /*
         * Extracts quadruplets of collinear points for each image.
         */
        virtual CrossRatioTupleSet determineTupleOfFourPoints(StringList &fileList, GLMListArrayVec2 &camObservations);
        virtual CrossRatioTupleSet determineTupleOfFourPoints();

        /*
         * Extracts quadruplets of collinear points for the image obtained by the given camera.
         */
        virtual CrossRatioTupleSet determineTupleOfFourPointsForCam(StringList &fileList, GLMListArrayVec2 &camObservations, int camIndex);
        virtual CrossRatioTupleSet determineTupleOfFourPointsForCam(int camIndex);
        virtual ListCrossRatioTupleSet determineTupleOfFourPointsForAllCam();
        
    protected:
        void setTuples(CrossRatioTupleSet &tupleSet);
        void setTuplesPerCam(ListCrossRatioTupleSet &tupleSetPerCam);
        void setTuplesPerCam(CrossRatioTupleSet &tupleSet, int camIndex);

    private:

        CrossRatioTupleSet createsTuples(IntArrayList &combos, IntList &pointSet, GLMListVec2 &points2D);
        void computeEdges(int camIndex, CVMat &edges);
        EigVector3List createLinesFromEdges(CVMat &edges);
        IntArrayList generateCorrespondances(std::vector<EigVector3> &lines, GLMListVec2 &points2D);
        CrossRatioTupleSet collapseListSet(ListCrossRatioTupleSet &tupleSetPerCam);

        StringList fileList;
        GLMListArrayVec2 camObservations;

        ListCrossRatioTupleSet tupleSetPerCam;
        CrossRatioTupleSet tupleSet;

    };

    typedef CRTuplesGenerator * CRTuplesGeneratorPtr;
    

} // namespace meshac

#endif // MESH_ACCURACY_CR_TUPLES_GENERATOR_H
