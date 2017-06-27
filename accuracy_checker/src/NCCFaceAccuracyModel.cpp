#include <meshac/NCCFaceAccuracyModel.hpp>

namespace meshac {

    NCCFaceAccuracyModel::NCCFaceAccuracyModel(std::string &meshFile, SfMData &data, std::string &pathPrefix) : FaceAccuracyModel(meshFile)
    {
        this->imgFilepath = data.camerasPaths_;
        this->cameras = this->extractCameraMatrix(data.camerasList_);
        this->camObservations = data.camViewing2DPoint_;
        this->point3DTo2DThroughCam = data.point3DTo2DThroughCam_;
        
        this->fixImagesPath(pathPrefix);
        this->convertTriangleToIndex();

        this->initAffineTriangle();
    }

    NCCFaceAccuracyModel::~NCCFaceAccuracyModel()
    { /*    */ }

    void NCCFaceAccuracyModel::initAffineTriangle()
    {
        destTriangle[0] = CVPoint2(0, TRIANGLE_SIZE);   // NOTE triangle can be changed, expanded
        destTriangle[1] = CVPoint2(TRIANGLE_SIZE, TRIANGLE_SIZE);
        destTriangle[2] = CVPoint2(TRIANGLE_SIZE, 0);
    }

    void NCCFaceAccuracyModel::convertTriangleToIndex()
    {
        FaceIndexList newFaces;
        TriangleList faces = this->getFaces();

        #pragma omp parallel for
        for (int f = 0; f < faces.size(); f++) {
            FaceIndex face;
            #pragma omp parallel for
            for (int v = 0; v < 3; v++) {
                PointD3 vertex = faces[f].vertex(v);
                size_t index = retreiveIndex(vertex);
                face.set(v, index);
            }

            #pragma omp critical
            newFaces.push_back(face);
        }

        this->faces = newFaces;
    }

    size_t NCCFaceAccuracyModel::retreiveIndex(PointD3 &vertex)
    {
        GLMVec3 point(vertex[0], vertex[1], vertex[2]);
        return retreiveIndex(point);
    }

    size_t NCCFaceAccuracyModel::retreiveIndex(GLMVec3 &point)
    {
        size_t index = -1;
        #pragma omp parallel for
        for (size_t p = 0; p < points.size(); p++) {
            if (glm::all(glm::epsilonEqual(point, points[p], SENSIBILITY))) {
                #pragma omp critical
                index = p;
            }
        }

        if (index >= 0) {
            return index;
        }
        throw UnexpectedPointException(point);
    }

    void NCCFaceAccuracyModel::fixImagesPath(std::string &pathPrefix)
    {
        std::for_each(this->imgFilepath.begin(), this->imgFilepath.end(), [pathPrefix](std::string &path) { path.insert(0, pathPrefix); } );
    }

    CameraMatrixList NCCFaceAccuracyModel::extractCameraMatrix(CameraList &cameras)
    {
        CameraMatrixList matList;
        for (CameraType cam : cameras) {
            matList.push_back(cam.cameraMatrix);
        }
        return matList;
    }

    double NCCFaceAccuracyModel::getAccuracyForFace(GLMVec3 &a, GLMVec3 &b, GLMVec3 &c)
    {
        int ia = retreiveIndex(a);
        int ib = retreiveIndex(b);
        int ic = retreiveIndex(c);

        double acc = -1.0;

        #pragma omp parallel for
        for (int i = 0; i < faces.size(); i++) {
            if (faces[i].is(ia, ib, ic)) {
                acc = getAccuracyForFace(i);        // assume there are not duplicate triangles
            }           
        }

        if (acc >= 0) {
            return acc;
        }

        throw UnexpectedTriangleException(a, b, c);
    }

    double NCCFaceAccuracyModel::getAccuracyForFace(int faceIndex)
    {
        ListMappingGLMVec2 mappings = getMappings(faceIndex);
        IntList commonCams = selectCommonCameras(mappings);

        removeUnusedMapping(mappings, commonCams);

        CVMatList triangles = projectTriangles(mappings, commonCams);

        return computeNCC(triangles);
    }

    double NCCFaceAccuracyModel::computeNCC(CVMatList triangles)
    {
        CVMat result;
        int result_cols =  triangles[0].cols - triangles[0].cols + 1;
        int result_rows = triangles[0].rows - triangles[0].rows + 1;
        result.create(result_rows, result_cols, CV_32FC1);
        DoubleList maxs, mins;

        #pragma omp parallel for collapse(2) private(result)
        for (int i = 0; i < triangles.size() - 1; i++) {
            for (int j = i; j < triangles.size(); j++) {
                matchTemplate(triangles[i], triangles[j], result, CV_TM_CCORR_NORMED);
                // normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() ); // needed??

                // Localizing the best match with minMaxLoc
                double minVal, maxVal; 
                cv::Point minLoc, maxLoc;

                minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, CVMat());
                #pragma omp critical
                {
                    mins.push_back(minVal);
                    maxs.push_back(maxVal);
                }
            }
        }

        return *std::max_element(maxs.begin(), maxs.end());
    }

    CVMatList NCCFaceAccuracyModel::projectTriangles(ListMappingGLMVec2 &mappings, IntList &commonCams)
    {
        CVMatList projectedFace;

        #pragma omp parallel for
        for (int i = 0; i < commonCams.size(); i++) {
            GLMVec2 a = mappings[0][commonCams[i]];
            GLMVec2 b = mappings[1][commonCams[i]];
            GLMVec2 c = mappings[2][commonCams[i]];

            CVMat affine = generateAffineTransform(a, b, c);
            CVMat projected = applyAffine(affine, commonCams[i]);

            #pragma omp critical
            projectedFace.push_back(projected);
        }
    }

    CVMat NCCFaceAccuracyModel::applyAffine(CVMat &affine, int camIndex)
    {
        CVMat src, projected;
        src = cv::imread(this->imgFilepath[camIndex], 0);   // 0 is grey scale
        projected = CVMat::zeros(src.rows, src.cols, src.type());

        warpAffine(src, projected, affine, projected.size());

        return projected.rowRange(0, TRIANGLE_SIZE).colRange(0, TRIANGLE_SIZE);
    }

    CVMat NCCFaceAccuracyModel::generateAffineTransform(GLMVec2 &a, GLMVec2 &b, GLMVec2 &c)
    {
        CVPoint2 srcTri[3];
        CVMat warp_mat(2, 3, CV_32FC1);
        
        /// Set your 3 points to calculate the  Affine Transform
        srcTri[0] = CVPoint2(a.x, a.y);
        srcTri[1] = CVPoint2(b.x, b.y);
        srcTri[2] = CVPoint2(c.x, c.y);

        /// Get the Affine Transform
        warp_mat = getAffineTransform(srcTri, destTriangle);
        return warp_mat;
    }

    ListMappingGLMVec2 NCCFaceAccuracyModel::getMappings(int faceIndex)
    {
        FaceIndex face = faces[faceIndex];
        ListMappingGLMVec2 mappings;

        for (int i = 0; i < 3; i++) {
            int pointIndex = face.vs[i];
            mappings.push_back(point3DTo2DThroughCam[pointIndex]);
        }
        return mappings;
    }

    // Select cams where every point is present
    IntList NCCFaceAccuracyModel::selectCommonCameras(ListMappingGLMVec2 &mappings)
    {
        IntArrayList camsList;

        #pragma omp parallel for
        for (int i = 0; i < mappings.size(); i++) {
            IntList cams = keys(mappings[i]);
            std::sort(cams.begin(), cams.end());

            #pragma omp critical
            camsList.push_back(cams);
        }

        IntList commonCams = camsList[0];

        for (int i = 0; i < mappings.size(); i++) {
            IntList intersection;
            std::set_intersection(camsList[i].begin(), camsList[i].end(), commonCams.begin(), commonCams.end(), std::back_inserter(intersection));
            commonCams = intersection;
        }

        return commonCams;
    }

    void NCCFaceAccuracyModel::removeUnusedMapping(ListMappingGLMVec2 &mappings, IntList &commonCams)
    {
        IntList allCams = unionCamIndex(mappings);

        for (int camIndex : allCams) {
            if (std::find(commonCams.begin(), commonCams.end(), camIndex) != commonCams.end()) {
                #pragma omp parallel for
                for (int i = 0; i < mappings.size(); i++) {
                    auto it = mappings[i].find(camIndex);
                    if (it != mappings[i].end()) {
                        mappings[i].erase(it);
                    }
                }
            }
        }
    }

    IntList NCCFaceAccuracyModel::unionCamIndex(ListMappingGLMVec2 &mappings)
    {
        IntList commonCams;

        #pragma omp parallel for
        for (int i = 0; i < mappings.size(); i++) {
            IntList cams = keys(mappings[i]);
            std::sort(cams.begin(), cams.end());

            #pragma omp critical
            commonCams.insert(commonCams.end(), cams.begin(), cams.end());
        }

        return commonCams;
    }
    



} // namespace meshac
