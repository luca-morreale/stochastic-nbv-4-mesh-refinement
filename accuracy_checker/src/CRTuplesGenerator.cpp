
#include <meshac/CRTuplesGenerator.hpp>


namespace meshac {

    CRTuplesGenerator::CRTuplesGenerator(StringList &fileList, GLMListArrayVec2 &camObservations)
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
        CVMat edges;
        CrossRatioTupleSet tuples;
        GLMListVec2 points2D = this->camObservations[camIndex];
        if (points2D.size() < 10) {
            //std::cout << "not enough" << std::endl;
            return CrossRatioTupleSet();
        }
        
        this->computeEdges(camIndex, edges);
        
        EigVector3List lines = this->createLinesFromEdges(edges);
        
        IntArrayList correspondances = this->generateCorrespondances(lines, points2D);
        
        for (IntList pointSet : correspondances) {
            
            IntArrayList combos = fixedSizeCombination(pointSet.size(), 4, SKIP_TUPLE_RATE, MAX_SAMPLE_SIZE);
            
            CrossRatioTupleSet tmp = this->createsTuples(combos, pointSet, points2D);
            for (auto el : tmp) {       // why this works and the one below do not??
                tuples.insert(el);
            }
            //tuples.insert(tmp.begin(), tmp.end());
        }

        return subsample(tuples, MAX_SAMPLE_SIZE);  // sampling
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

        for (auto combo : combos) {
            CrossRatioTuple tmp;
            for (int i=0; i < 4; i++) {
                int index = pointSet[combo[i]];
                tmp.append(points2D[index]);
            }
            
            tuples.insert(tmp);
        }

        return tuples;
    }

    void CRTuplesGenerator::computeEdges(int camIndex, CVMat &edges)
    {
        std::string imagePath = this->fileList[camIndex];
        cv::Mat src = cv::imread(imagePath, 0);
    
        cv::Canny(src, edges, CANNY_LOW_THRESHOLD, CANNY_LOW_THRESHOLD * CANNY_RATIO, CANNY_KERNEL_SIZE);
    }

    std::vector<EigVector3> CRTuplesGenerator::createLinesFromEdges(CVMat &edges)
    {
        CVListVec4 polarLines;
        std::vector<EigVector3> lines;
        
        // discretization rho 1
        // discretization angle CV_PI/180
        // at least 100 votes
        //cv::HoughLines(logicalImg, lines, 1, CV_PI/180, 4, 0, 0 );
        // votes must be fixed! maybe with threshold, or check to not take close lines
        cv::HoughLinesP(edges, polarLines, 1, CV_PI/180, 90, 0, 0);  // this just gives rho and theta not the line!!!


        for( size_t i = 0; i < polarLines.size(); i++ ) {
            auto l = polarLines[i];
            EigVector3 pt1(l[0], l[1], 1);
            EigVector3 pt2(l[2], l[3], 1);
            
            EigVector3 line = pt1.cross(pt2);
            line /= line[2];
            lines.push_back(line);
        }

        return lines;
    }


    IntArrayList CRTuplesGenerator::generateCorrespondances(std::vector<EigVector3> &lines, GLMListVec2 &points2D)
    {
        EigVector3 line, point;
        IntArrayList correspondances;
        correspondances.assign(lines.size(), IntList());

        for (unsigned int lineIndex = 0; lineIndex < lines.size(); lineIndex++) {
            EigVector3 line = lines[lineIndex];

            for (unsigned int pointIndex = 0; pointIndex < points2D.size(); pointIndex++) {
                EigVector3 point =  EigVector3(points2D[pointIndex].x, points2D[pointIndex].y, 1);
                if (std::abs(line.transpose() * point) < 1.0 ) {
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
