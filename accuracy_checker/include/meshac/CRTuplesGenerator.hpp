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
    #define CANNY_LOW_THRESHOLD 200
    #define CANNY_KERNEL_SIZE 3
    #define SKIP_TUPLE_RATE 0.8
    #define MAX_SAMPLE_SIZE 50

    class CRTuplesGenerator {
    public:
        CRTuplesGenerator(StringList &fileList, GLMVec2ArrayList &camObservations);
        ~CRTuplesGenerator();

        /*
         * Getter and setter for Cameras' Observations.
         */
        GLMVec2ArrayList getCamObservations();
        void setCamObservations(GLMVec2ArrayList &camObservations);
        void setCamObservations(GLMVec2List &list, int camIndex);
        void updateCamObservations(GLMVec2List &list, int camIndex);
        void updateCamObservations(GLMVec2ArrayList &camObservations, IntList &camIndexs);


        /*
         * Getter and setter for list of images path.
         */
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
        virtual CrossRatioTupleSet determineTupleOfFourPoints(StringList &fileList, GLMVec2ArrayList &camObservations);
        virtual CrossRatioTupleSet determineTupleOfFourPoints();

        /*
         * Extracts quadruplets of collinear points for the image obtained by the given camera.
         */
        virtual CrossRatioTupleSet determineTupleOfFourPointsForCam(StringList &fileList, GLMVec2ArrayList &camObservations, int camIndex);
        virtual CrossRatioTupleSet determineTupleOfFourPointsForCam(int camIndex);
        virtual ListCrossRatioTupleSet determineTupleOfFourPointsForAllCam();
        
    protected:
        void setTuples(CrossRatioTupleSet &tupleSet);
        void setTuplesPerCam(ListCrossRatioTupleSet &tupleSetPerCam);
        void setTuplesPerCam(CrossRatioTupleSet &tupleSet, int camIndex);

    private:

        CrossRatioTupleSet createsTuples(IntArrayList &combos, IntList &pointSet, GLMVec2List &points2D);
        void computeEdges(int camIndex, CVMat &edges);
        EigVector3List createLinesFromEdges(CVMat &edges);
        IntArrayList generateCorrespondances(EigVector3List &lines, GLMVec2List &points2D);
        CrossRatioTupleSet collapseListSet(ListCrossRatioTupleSet &tupleSetPerCam);

        StringList fileList;
        GLMVec2ArrayList camObservations;

        ListCrossRatioTupleSet tupleSetPerCam;
        CrossRatioTupleSet tupleSet;

    };

    typedef CRTuplesGenerator * CRTuplesGeneratorPtr;
    

} // namespace meshac

#endif // MESH_ACCURACY_CR_TUPLES_GENERATOR_H
