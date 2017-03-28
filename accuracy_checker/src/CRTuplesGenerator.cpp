
#include <meshac/CRTuplesGenerator.hpp>

namespace meshac {

    CRTuplesGenerator::CRTuplesGenerator(StringList &fileList, GLMListArrayVec2 &camObservations)
    {
        this->fileList = fileList;
        this->camObservations = camObservations;
    }

    CRTuplesGenerator::~CRTuplesGenerator() 
    {
        this->fileList.clear();
        this->camObservations.clear();
        this->tupleSetPerCam.clear();
        this->tupleSet.clear(); 
    }


    /*
     * Extraction of quadruplets of collinear points for each image.
     */
    CrossRatioTupleSet CRTuplesGenerator::determineTupleOfFourPoints(StringList &fileList, GLMListArrayVec2 &camObservations)
    {
        this->setFileList(fileList);
        this->setCamObservations(camObservations);
        return this->determineTupleOfFourPoints();
    }

    CrossRatioTupleSet CRTuplesGenerator::determineTupleOfFourPoints()  // FIX missing subsampling
    {
        int N = this->camObservations.size();
        ListCrossRatioTupleSet listTupleSet(N);

        #pragma omp parallel for
        for (int camIndex = 0; camIndex < N; camIndex++) {
            listTupleSet[camIndex] = determineTupleOfFourPointsForCam(camIndex);
        }

        tupleSet = this->collapseListSet(listTupleSet);

        this->setTuplesPerCam(listTupleSet);
        this->setTuples(tupleSet);

        return tupleSet;
    }


    /*
     * Extraction quadruplets of collinear points for the image obtained by the given camera.
     */
    CrossRatioTupleSet CRTuplesGenerator::determineTupleOfFourPointsForCam(StringList &fileList, GLMListArrayVec2 &camObservations, int camIndex)
    {
        this->setFileList(fileList);
        this->setCamObservations(camObservations);
        return this->determineTupleOfFourPointsForCam(camIndex);
    }

    CrossRatioTupleSet CRTuplesGenerator::determineTupleOfFourPointsForCam(int camIndex)
    {
        GLMListVec2 points2D = this->camObservations[camIndex];

        CVMat edges;
        computeEdges(camIndex, edges);
        CVListVec2 lines = createLinesFromPoints(edges);
        
        IntArrayList correspondances = generateCorrespondances(lines, points2D);
        CrossRatioTupleSet tuples;

        #pragma omp parallel for
        for (IntList pointSet : correspondances) {
            IntArrayList combos = combination(pointSet.size(), 4);
            auto tmp = createsTuples(combos, pointSet, points2D);

            #pragma omp critical
            tuples.insert(tmp.begin(), tmp.end());
        }

        this->setTuplesPerCam(tuples, camIndex);

        return tuples;
    }

    ListCrossRatioTupleSet CRTuplesGenerator::determineTupleOfFourPointsForAllCam()
    {
        for (int i = 0; i < camObservations.size(); i++) {
            this->determineTupleOfFourPointsForCam(i);
        }
        return this->tupleSetPerCam;
    }

    /*
     * Private methods
     */
    CrossRatioTupleSet CRTuplesGenerator::createsTuples(IntArrayList &combos, IntList &pointSet, GLMListVec2 &points2D)
    {
        CrossRatioTupleSet tuples;

        #pragma omp parallel for
        for (auto combo : combos) {
            CrossRatioTuple tmp;
            for (int i=0; i < 4; i++) {
                int index = pointSet[combo[i]];
                tmp.append(points2D[index]);
            }
            
            #pragma omp critical
            tuples.insert(tmp);
        }
    }

    void CRTuplesGenerator::computeEdges(int camIndex, CVMat &edges)
    {
        std::string imagePath = this->fileList[camIndex];
        CVMat img = cv::imread(imagePath);
        cvtColor(img, img, CV_BGR2GRAY);

        edges.create(img.size(), img.type());

        blur(img, edges, cv::Size(3,3) );
        /// Canny detector
        Canny(edges, edges, CANNY_LOW_THRESHOLD, CANNY_LOW_THRESHOLD * CANNY_RATIO, CANNY_KERNEL_SIZE);

        /// Using Canny's output as a mask, we display our result
        //dst = Scalar::all(0);
        //img.copyTo( edges, edges);
    }


    CVListVec2 CRTuplesGenerator::createLinesFromPoints(CVMat &edges)
    {
        CVListVec2 lines;
        
        // discretization rho 1
        // discretization angle CV_PI/180
        // at least 4 votes
        //cv::HoughLines(logicalImg, lines, 1, CV_PI/180, 4, 0, 0 );
        cv::HoughLinesP(edges, lines, 1, CV_PI/180, 4, 0, 0);
        
        return lines;
    }


    IntArrayList CRTuplesGenerator::generateCorrespondances(CVListVec2 &lines, GLMListVec2 &points2D)
    {
        cv::Vec<float, 1> zero;
        IntArrayList correspondances(lines.size());

        #pragma omp parallel for collapse(2)
        for (unsigned int lineIndex = 0; lineIndex < lines.size(); lineIndex++) {
            cv::Vec2f line = lines[lineIndex];
            
            for (unsigned int pointIndex = 0; pointIndex < points2D.size(); pointIndex++) {
                cv::Vec2f point(points2D[pointIndex].x, points2D[pointIndex].y);
                if (line.t() * point == zero) {

                    #pragma omp critical
                    correspondances[lineIndex].push_back(pointIndex);
                }
            }
        }

        return correspondances;
    }


    CrossRatioTupleSet CRTuplesGenerator::collapseListSet(ListCrossRatioTupleSet &tupleSetPerCam)
    {
        CrossRatioTupleSet tupleSet;
        for (CrossRatioTupleSet cameraSet : tupleSetPerCam) {
            tupleSet.insert(cameraSet.begin(), cameraSet.end());
        }

        return tupleSet;
    }


    /*
     * Getter and setter for computed tuples.
     */
    CrossRatioTupleSet CRTuplesGenerator::getComputedTuples()
    {
        return this->tupleSet;
    }

    CrossRatioTupleSet CRTuplesGenerator::getComputedTuplesForCam(int camIndex)
    {
        return this->tupleSetPerCam[camIndex];
    }

    ListCrossRatioTupleSet CRTuplesGenerator::getCrossRatioTupleSetList()
    {
        return this->tupleSetPerCam;
    }

    void CRTuplesGenerator::setTuples(CrossRatioTupleSet &tupleSet)
    {
        this->tupleSet = tupleSet;
    }

    void CRTuplesGenerator::setTuplesPerCam(ListCrossRatioTupleSet &tupleSetPerCam)
    {
        this->tupleSetPerCam = tupleSetPerCam;
    }

    void CRTuplesGenerator::setTuplesPerCam(CrossRatioTupleSet &tupleSet, int camIndex)
    {
        this->tupleSetPerCam[camIndex] = tupleSet;
    }

    /*
     * Getter and setter for Camera's Observations.
     */
    void CRTuplesGenerator::setCamObservations(GLMListArrayVec2 &camObservations)
    {
        this->camObservations = camObservations;
    }

    void CRTuplesGenerator::setCamObservations(GLMListVec2 &list, int camIndex)
    {
        this->camObservations[camIndex].clear();
        this->camObservations[camIndex].insert(camObservations[camIndex].begin(), list.begin(), list.end());
    }

    void CRTuplesGenerator::updateCamObservations(GLMListVec2 &list, int camIndex)
    {
        this->camObservations[camIndex] = list;
        this->tupleSetPerCam[camIndex].clear();
    }

    void CRTuplesGenerator::updateCamObservations(GLMListArrayVec2 &camObservations, IntList &camIndexs)
    {
        for (int i = 0; i < camObservations.size(); i++) {
            this->updateCamObservations(camObservations[i], camIndexs[i]);
        }
    }

    GLMListArrayVec2 CRTuplesGenerator::getCamObservations()
    {
        return camObservations;
    }

    
    void CRTuplesGenerator::setFileList(StringList &fileList)
    {
        this->fileList = fileList;
    }

    StringList CRTuplesGenerator::getFileList()
    {
        return this->fileList;
    }

} // namespace meshac
