
#include <CRTuplesGenerator.hpp>

namespace meshac {


CRTuplesGenerator::CRTuplesGenerator() { }

CRTuplesGenerator::CRTuplesGenerator(SfMData *data)
{
    this->data = data;
}

CRTuplesGenerator::~CRTuplesGenerator() { }

/**
 * Getter and setter for SfMData.
 */
void CRTuplesGenerator::setSfMData(SfMData *data)
{
    this->data = data;
}

SfMData *CRTuplesGenerator::getSfMData()
{
    return data;
}

/**
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

void CRTuplesGenerator::setTuples(CrossRatioTupleSet tupleSet)
{
    this->tupleSet = tupleSet;
}

void CRTuplesGenerator::setTuplesPerCam(ListCrossRatioTupleSet tupleSetPerCam)
{
    this->tupleSetPerCam = tupleSetPerCam;
}

/**
 * Extraction of quadruplets of collinear points for each image.
 */
CrossRatioTupleSet CRTuplesGenerator::determineTupleOfFourPoints(SfMData *data)
{
    this->setSfMData(data);
    return this->determineTupleOfFourPoints();
}

CrossRatioTupleSet CRTuplesGenerator::determineTupleOfFourPoints()
{
    ListCrossRatioTupleSet listTupleSet(data->numCameras_);

    for (int camIndex = 0; camIndex < data->numCameras_; camIndex++) {
        listTupleSet[camIndex] = determineTupleOfFourPointsForCam(data, camIndex);
    }

    tupleSet = this->collapseListSet(listTupleSet);

    this->setTuplesPerCam(listTupleSet);
    this->setTuples(tupleSet);

    return tupleSet;
}


/**
 * Extraction quadruplets of collinear points for the image obtained by the given camera.
 */

CrossRatioTupleSet CRTuplesGenerator::determineTupleOfFourPointsForCam(SfMData *data, int camIndex)
{
    this->setSfMData(data);
    return this->determineTupleOfFourPointsForCam(camIndex);
}

CrossRatioTupleSet CRTuplesGenerator::determineTupleOfFourPointsForCam(int camIndex)
{
    GLMList2DVec points2D = this->data->point2DoncamViewingPoint_[camIndex];
    CVList2DVec lines = createLinesFromPoints(this->data->imageHeight_, this->data->imageWidth_, points2D);
    
    IntArrayList correspondances = generateCorrespondances(lines, points2D);
    CrossRatioTupleSet tuples;

    for (IntList pointSet : correspondances) {
        IntArrayList combos = combination(pointSet.size(), 4);
        auto tmp = createsTuples(combos, pointSet, points2D);
        tuples.insert(tmp.begin(), tmp.end());
    }

    return tuples;
}

/**
 * Private methods
 */
CrossRatioTupleSet CRTuplesGenerator::createsTuples(IntArrayList &combos, IntList &pointSet, GLMList2DVec &points2D)
{
    CrossRatioTupleSet tuples;

    #pragma omp parallel for
    for (auto combo : combos) {
        CrossRatioTuple tmp;
        for (int i=0; i < 4; i++) {
            int index = pointSet[combo[i]];
            tmp.append(&points2D[index]);
        }
        #pragma omp critical
        tuples.insert(tmp);
    }
}


CVList2DVec CRTuplesGenerator::createLinesFromPoints(int imgHeight, int imgWidth, GLMList2DVec &points2D)
{
    CVList2DVec lines;
    
    cv::Mat logicalImg(imgHeight, imgWidth, CV_8U);
    createFeatureImage(logicalImg, points2D);

    // discretization rho 1
    // discretization angle CV_PI/180
    // at least 4 votes
    //cv::HoughLines(logicalImg, lines, 1, CV_PI/180, 4, 0, 0 );
    cv::HoughLinesP(logicalImg, lines, 1, CV_PI/180, 4, 0, 0);
    
    return lines;
}


void CRTuplesGenerator::createFeatureImage(cv::Mat logicalImg, GLMList2DVec &points2D)
{
    for (glm::vec2 point : points2D) {
        logicalImg.at<uint>(point.x, point.y) += 1;
    }
}


IntArrayList CRTuplesGenerator::generateCorrespondances(CVList2DVec &lines, GLMList2DVec &points2D)
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


CrossRatioTupleSet CRTuplesGenerator::collapseListSet(ListCrossRatioTupleSet tupleSetPerCam)
{
    CrossRatioTupleSet tupleSet;
    for (CrossRatioTupleSet cameraSet : tupleSetPerCam) {
        tupleSet.insert(cameraSet.begin(), cameraSet.end());
    }

    return tupleSet;
}




} // namespace meshac
