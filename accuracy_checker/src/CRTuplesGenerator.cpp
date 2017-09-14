
#include <meshac/CRTuplesGenerator.hpp>


namespace meshac {

    CRTuplesGenerator::CRTuplesGenerator(StringList &fileList, GLMVec2ArrayList &camObservations)
    {
        this->fileList = fileList;
        this->camObservations = camObservations;
        this->tupleSetPerCam.assign(this->camObservations.size(), CrossRatioTupleSet());
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
    CrossRatioTupleSet CRTuplesGenerator::determineTupleOfFourPoints(StringList &fileList, GLMVec2ArrayList &camObservations)
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
    CrossRatioTupleSet CRTuplesGenerator::determineTupleOfFourPointsForCam(StringList &fileList, GLMVec2ArrayList &camObservations, int camIndex)
    {
        this->setFileList(fileList);
        this->setCamObservations(camObservations);
        return this->determineTupleOfFourPointsForCam(camIndex);
    }

    CrossRatioTupleSet CRTuplesGenerator::determineTupleOfFourPointsForCam(int camIndex)
    {
        IntArrayList quadruplets;
        CrossRatioTupleSet tuples;
        GLMVec2List points2D = this->camObservations[camIndex];

        if (!enoughPoints(points2D.size())) return CrossRatioTupleSet();

        this->fillQuadruplets(camIndex, points2D, quadruplets);
        
        //IntArrayList correspondances = this->generateCorrespondances(lines, points2D);
        
        #pragma omp parallel for
        for (int i = 0; i < quadruplets.size(); i++) {
            IntList pointSet = quadruplets[i];
            IntArrayList combos = fixedSizeCombination(pointSet.size(), 4, SKIP_TUPLE_RATE, MAX_SAMPLE_SIZE);
            
            CrossRatioTupleSet tmp = this->createsTuples(combos, pointSet, points2D);

            #pragma omp critical
            for (auto el : tmp) {       // why this works and the one below do not??
                tuples.insert(el);
            }
            //tuples.insert(tmp.begin(), tmp.end());
        }

        return subsample(tuples, MAX_SAMPLE_SIZE);  // sampling
    }
    
    ListCrossRatioTupleSet CRTuplesGenerator::determineTupleOfFourPointsForAllCam()
    {
        #pragma omp parallel for
        for (int i = 0; i < camObservations.size(); i++) {
            this->determineTupleOfFourPointsForCam(i);
        }
        return this->tupleSetPerCam;
    }

    /*
     * Private methods
     */
    CrossRatioTupleSet CRTuplesGenerator::createsTuples(IntArrayList &combos, IntList &pointSet, GLMVec2List &points2D) 
    {
        CrossRatioTupleSet tuples;

        #pragma omp parallel for
        for (int i = 0; i < combos.size(); i++) {
            IntList combo = combos[i];
            CrossRatioTuple tmp;
            
            for (int i=0; i < 4; i++) {
                int index = pointSet[combo[i]];
                tmp.append(points2D[index]);
            }

            #pragma omp critical
            tuples.insert(tmp);
        }

        return tuples;
    }

    bool CRTuplesGenerator::enoughPoints(unsigned int size)
    {
        return size > MIN_NUM_POINTS_IN_IMAGE;
    }

    void CRTuplesGenerator::fillQuadruplets(int camIndex, GLMVec2List &points2D, IntArrayList &quadruplets)
    {
        CVMat edges;
        CVSegmentList segments;
        computeEdges(camIndex, edges);
        extractSegmentsFromEdges(edges, segments);
        generateCorrespondances(segments, points2D, quadruplets);
    }

    void CRTuplesGenerator::computeEdges(int camIndex, CVMat &edges)
    {
        std::string imagePath = this->fileList[camIndex];
        cv::Mat src = cv::imread(imagePath, 0);
    
        cv::Canny(src, edges, CANNY_LOW_THRESHOLD, CANNY_LOW_THRESHOLD * CANNY_RATIO, CANNY_KERNEL_SIZE);
    }

    void CRTuplesGenerator::extractSegmentsFromEdges(CVMat &edges, CVSegmentList &segments)
    {
        CVSegmentList allSegments;
        cv::Ptr<cv::line_descriptor::LSDDetector> segmentDetector = cv::line_descriptor::LSDDetector::createLSDDetector();
        segmentDetector->detect(edges, allSegments, 2, 2);

        collapseSegments(allSegments, segments);
    }

    void CRTuplesGenerator::collapseSegments(CVSegmentList &allSegments, CVSegmentList &segments) // TODO
    {
        // cos(angle) between two normalized lines is dot product of just the two main components
        // this angle might be obtuse or acute so must be checked and take the minimum!
        CVLineList lines;

        getLines(allSegments, lines);
        joinSegments(allSegments, segments, lines);
    
    }

    void CRTuplesGenerator::getLines(CVSegmentList &segments, CVLineList &lines)
    {
        #pragma omp parallel for
        for (unsigned int segmentIndex = 0; segmentIndex < segments.size(); segmentIndex++) {   // precomputes all the lines
            CVSegment segment = segments[segmentIndex];
            CVPoint2 start = segment.getStartPoint();
            CVPoint2 end = segment.getEndPoint();
            
            CVVector3 s = CVVector3(start.x, start.y, 1.0);
            CVVector3 t = CVVector3(end.x, end.y, 1.0);

            CVLine line = s.cross(t);
            line /= std::sqrt(std::pow(line[0], 2) + std::pow(line[1], 2));

            #pragma omp critical
            lines.push_back(line);
        }
    }

    void CRTuplesGenerator::joinSegments(CVSegmentList &allSegments, CVSegmentList &segments, CVLineList &lines)
    {
        IntList groups(allSegments.size(), -1);
        // hard to parallelize
        for (int lineIndex = 0; lineIndex < lines.size() - 1; lineIndex++) {
            for (int secondLineIndex = lineIndex + 1; secondLineIndex < lines.size(); secondLineIndex++) {
                if (cosTheta(lines[lineIndex], lines[secondLineIndex]) <= angleThreshold) {
                    setProperGroup(groups, allSegments, segments, lineIndex, secondLineIndex);
                    setLongestSegment(allSegments, segments, groups[lineIndex], secondLineIndex);
                }
            }
        }
    }

    void CRTuplesGenerator::setProperGroup(IntList &groups, CVSegmentList &allSegments, CVSegmentList &segments, int lineIndex, int secondLineIndex)
    {
        if (groups[lineIndex] == -1) {
            segments.push_back(allSegments[lineIndex]);
            groups[lineIndex] = segments.size()-1;
            groups[secondLineIndex] = segments.size()-1;
        }
    }

    void CRTuplesGenerator::setLongestSegment(CVSegmentList &allSegments, CVSegmentList &segments, int lineIndex, int secondLineIndex)
    {
        CVPoint2 startOld = segments[lineIndex].getStartPoint();
        CVPoint2 endOld = segments[lineIndex].getEndPoint();
        CVPoint2 startNew = allSegments[secondLineIndex].getStartPoint();
        CVPoint2 endNew = allSegments[secondLineIndex].getEndPoint();

        float refDistance = segments[lineIndex].lineLength;
        if (refDistance < cv::norm(startNew - endOld)) {
            segments[lineIndex].lineLength = cv::norm(startNew - endOld);
            refDistance = segments[lineIndex].lineLength;
            segments[lineIndex].startPointX = startNew.x;
            segments[lineIndex].startPointY = startNew.y;
        }
        if (refDistance < cv::norm(startOld - endNew)) {
            segments[lineIndex].lineLength = cv::norm(startOld - endNew);
            refDistance = segments[lineIndex].lineLength;
            segments[lineIndex].endPointX = endNew.x;
            segments[lineIndex].endPointY = endNew.y;
        }
        if (refDistance < allSegments[secondLineIndex].lineLength) {
            segments[lineIndex] = allSegments[secondLineIndex];
            refDistance = segments[lineIndex].lineLength;
        }
        if (refDistance < cv::norm(startNew - startOld)) {
            segments[lineIndex].lineLength = cv::norm(startOld - startNew);
            refDistance = segments[lineIndex].lineLength;
            segments[lineIndex].endPointX = startNew.x;
            segments[lineIndex].endPointY = startNew.y;
        }
        if (refDistance < cv::norm(endNew - endOld)) {
            segments[lineIndex].lineLength = cv::norm(endOld - endNew);
            refDistance = segments[lineIndex].lineLength;
            segments[lineIndex].startPointX = endNew.x;
            segments[lineIndex].startPointY = endNew.y;
        }
    }

    float CRTuplesGenerator::cosTheta(CVLine &a, CVLine &b)
    {
        return std::fabs(a[0] * b[0] + a[1] * b[1]);
    }

    void CRTuplesGenerator::generateCorrespondances(CVSegmentList &segments, GLMVec2List &points2D, IntArrayList &quadruplets)
    {
        quadruplets.assign(segments.size(), IntList());

        #pragma omp parallel for
        for (unsigned int segmentIndex = 0; segmentIndex < segments.size(); segmentIndex++) {
            CVSegment segment = segments[segmentIndex];
            CVPoint2 start = segment.getStartPoint();
            CVPoint2 end = segment.getEndPoint();

            double segmentSize = cv::norm(start - end);

            #pragma omp parallel for
            for (unsigned int pointIndex = 0; pointIndex < points2D.size(); pointIndex++) {
                CVPoint2 point = CVPoint2(points2D[pointIndex].x, points2D[pointIndex].y);

                double pointToStart = cv::norm(point - start);
                double pointToEnd = cv::norm(point - end);

                if (pointToStart + pointToEnd - segmentSize < 5.0) {    // 5 pixel is still close if line is long
                    #pragma omp critical
                    quadruplets[segmentIndex].push_back(pointIndex);
                }
            }
        }
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
        if(this->tupleSetPerCam[camIndex].size() < 1) {
            this->determineTupleOfFourPointsForCam(camIndex);            
        }
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
    void CRTuplesGenerator::setCamObservations(GLMVec2ArrayList &camObservations)
    {
        this->camObservations = camObservations;
    }

    void CRTuplesGenerator::setCamObservations(GLMVec2List &list, int camIndex)
    {
        this->camObservations[camIndex].clear();
        this->camObservations[camIndex].insert(camObservations[camIndex].begin(), list.begin(), list.end());
    }

    void CRTuplesGenerator::updateCamObservations(GLMVec2List &list, int camIndex)
    {
        this->camObservations[camIndex] = list;
        this->tupleSetPerCam[camIndex].clear();
    }

    void CRTuplesGenerator::updateCamObservations(GLMVec2ArrayList &camObservations, IntList &camIndexs)
    {
        for (int i = 0; i < camObservations.size(); i++) {
            this->updateCamObservations(camObservations[i], camIndexs[i]);
        }
    }

    GLMVec2ArrayList CRTuplesGenerator::getCamObservations()
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
