#ifndef MESH_ACCURACY_CR_TUPLES_GENERATOR_H
#define MESH_ACCURACY_CR_TUPLES_GENERATOR_H

#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>

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
    #define MIN_NUM_POINTS_IN_IMAGE 10

    class CRTuplesGenerator {
    public:
        CRTuplesGenerator(StringList &fileList, GLMVec2ArrayList &camObservations);
        virtual ~CRTuplesGenerator();

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

        virtual void fillQuadruplets(int camIndex, GLMVec2List &points2D, IntArrayList &quadruplets);

        virtual CrossRatioTupleSet createsTuples(IntArrayList &combos, IntList &pointSet, GLMVec2List &points2D);
        virtual bool enoughPoints(unsigned int size);

        virtual void computeEdges(int camIndex, CVMat &edges);
        virtual void extractSegmentsFromEdges(CVMat &edges, CVSegmentList &segments);
        virtual void collapseSegments(CVSegmentList &allSegments, CVSegmentList &segments);
        

        virtual void generateCorrespondances(CVSegmentList &segments, GLMVec2List &points2D, IntArrayList &quadruplets);

        float cosTheta(CVLine &a, CVLine &b);

    private:
        CrossRatioTupleSet collapseListSet(ListCrossRatioTupleSet &tupleSetPerCam);
        void getLines(CVSegmentList &segments, CVLineList &lines);
        void setProperGroup(IntList &groups, CVSegmentList &allSegments, CVSegmentList &segments, int lineIndex, int secondLineIndex);
        void setLongestSegment(CVSegmentList &allSegments, CVSegmentList &segments, int index, int secondLineIndex);
        void joinSegments(CVSegmentList &allSegments, CVSegmentList &segments, CVLineList &lines);


        StringList fileList;
        GLMVec2ArrayList camObservations;

        ListCrossRatioTupleSet tupleSetPerCam;
        CrossRatioTupleSet tupleSet;

        const float angleThreshold = std::cos(20.0 * M_PI / 180.0);

    };

    typedef CRTuplesGenerator * CRTuplesGeneratorPtr;
    

} // namespace meshac

#endif // MESH_ACCURACY_CR_TUPLES_GENERATOR_H
