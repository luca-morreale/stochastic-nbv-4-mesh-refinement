#include <meshac/NCCFaceAccuracyModel.hpp>

namespace meshac {

    NCCFaceAccuracyModel::NCCFaceAccuracyModel(std::string &meshFile, SfMData &data, std::string &pathPrefix) : FaceAccuracyModel(meshFile)
    {
        this->imgFilepath = data.camerasPaths_;
        this->points = data.points_;
        this->cams = data.camerasList_;
        this->point3DTo2DThroughCam.assign(this->points.size(), std::map<int, GLMVec2>());

        this->fixImagesPath(pathPrefix);
        this->initTree();       // NOTE do not move this below
        this->convertTriangleToIndex();
        this->initAffineTriangle();

        this->projectMeshPoints();
    }

    NCCFaceAccuracyModel::~NCCFaceAccuracyModel()
    {
        delete tree;
    }

    void NCCFaceAccuracyModel::initAffineTriangle()
    {
        destTriangle[0] = CVPoint2(0, 0);
        destTriangle[1] = CVPoint2(0, TRIANGLE_SIZE);
        destTriangle[2] = CVPoint2(TRIANGLE_SIZE, 0);
    }

    void NCCFaceAccuracyModel::initTree()
    {
        TriangleList faces = this->getFaces();
        this->tree = new Tree(faces.begin(), faces.end());
        tree->build();
    }

    void NCCFaceAccuracyModel::convertTriangleToIndex()
    {
        FaceIndexList newFaces;
        TriangleList faces = this->getFaces();
        
        // NOTE if parallelized this section will break during the execution (or at least on my machine)
        // #pragma omp parallel for
        for (int f = 0; f < faces.size(); f++) {
            try {
                FaceIndex face;
                // #pragma omp parallel for
                for (int v = 0; v < 3; v++) {
                    PointD3 vertex = faces[f].vertex(v);
                    size_t index = retreiveIndex(vertex);
                    face.set(v, index);
                }
                // #pragma omp critical
                newFaces.push_back(face);
            } catch (UnexpectedPointException &ex) {}
            TriangleList faces = this->getFaces();      // NOTE do not remove this or it won't work
            this->tree->rebuild(faces.begin(), faces.end());
        }

        this->faces = newFaces;
    }

    int NCCFaceAccuracyModel::retreiveIndex(PointD3 &vertex)
    {
        GLMVec3 point(vertex.x(), vertex.y(), vertex.z());
        return retreiveIndex(point);
    }

    int NCCFaceAccuracyModel::retreiveIndex(GLMVec3 &point)
    {
        int index = -1;
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
        if (faceIndex > faces.size()) {
            throw UndefinedFaceIndexException(faceIndex);
        }

        ListMappingGLMVec2 mappings = getMappings(faceIndex);

        IntList commonCams = selectCommonCameras(mappings);

        if (commonCams.size() < 2) return 0.0;

        mappings = removeUnusedMapping(mappings, commonCams);

        CVMatList triangles = projectTriangles(mappings, commonCams);

        return computeNCC(triangles);
    }

    double NCCFaceAccuracyModel::computeNCC(CVMatList triangles)
    {
        CVMat result;
        int result_cols =  triangles[0].cols - triangles[0].cols + 1;
        int result_rows = triangles[0].rows - triangles[0].rows + 1;
        result.create(result_rows, result_cols, 0);
        DoubleList maxs, mins;

        // NOTE if parallelized probably dies
        // #pragma omp parallel for
        for (int i = 0; i < triangles.size() - 1; i++) {
            for (int j = i+1; j < triangles.size(); j++) {

                cv::matchTemplate(triangles[i], triangles[j], result, CV_TM_CCORR_NORMED);
                double minVal, maxVal; 
                cv::Point minLoc, maxLoc;

                cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, CVMat());
                #pragma omp critical
                {
                    mins.push_back(minVal);
                    maxs.push_back(maxVal);
                }
            }
        }

        std::sort(maxs.rbegin(), maxs.rend());
        return maxs[0];
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
        return projectedFace;
    }

    CVMat NCCFaceAccuracyModel::applyAffine(CVMat &affine, int camIndex)
    {
        CVMat src, projected;
        src = cv::imread(this->imgFilepath[camIndex], 0);   // 0 is grey scale
        projected = CVMat::zeros(src.rows, src.cols, src.type());

        cv::warpAffine(src, projected, affine, projected.size());

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
        warp_mat = cv::getAffineTransform(srcTri, destTriangle);
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

        // NOTE i need the cams to be in order, so I can not do parallelization
        for (int i = 0; i < mappings.size(); i++) {
            IntList cams = keys(mappings[i]);
            std::sort(cams.begin(), cams.end());

            camsList.push_back(cams);
        }

        IntList commonCams = camsList[0];

        for (int i = 1; i < camsList.size(); i++) {
            IntList intersection;
            std::set_intersection(camsList[i].begin(), camsList[i].end(), commonCams.begin(), commonCams.end(), std::back_inserter(intersection));
            commonCams = intersection;
        }

        return commonCams;
    }

    ListMappingGLMVec2 NCCFaceAccuracyModel::removeUnusedMapping(ListMappingGLMVec2 &mappings, IntList &commonCams)
    {
        ListMappingGLMVec2 camMappings;
        camMappings.assign(mappings.size(), std::map<int, GLMVec2>());

        // #pragma omp parallel for collapse(2)
        for (int i = 0; i < commonCams.size(); i++) {
            for (int m = 0; m < mappings.size(); m++) {
                camMappings[m][commonCams[i]] = mappings[m][commonCams[i]];
            }
        }
        return camMappings;
    }

    IntList NCCFaceAccuracyModel::unionCamIndex(ListMappingGLMVec2 &mappings)
    {
        IntList commonCams;

        for (int i = 0; i < mappings.size(); i++) {
            IntList cams = keys(mappings[i]);
            std::sort(cams.begin(), cams.end());

            commonCams.insert(commonCams.end(), cams.begin(), cams.end());
        }

        return commonCams;
    }

    void NCCFaceAccuracyModel::projectMeshPoints()
    {
        // NOTE this section does not work if parallelized
        // #pragma omp parallel for
        for (int p = 0; p < this->points.size(); p++) {
            for(int c = 0; c < this->cams.size(); c++) {
                try {
                    GLMVec2 point = projectThrough(points[p], c);
                    // #pragma omp critical
                    this->point3DTo2DThroughCam[p].insert(std::make_pair(c, point));
                } catch (const UnprojectablePointThroughCamException &ex) 
                { } // do nothing because it makes no sense to project it
            }
        }
    }


    GLMVec2 NCCFaceAccuracyModel::projectThrough(GLMVec3 &meshPoint, int camIndex)
    {
        GLMVec2 point = getProjectedPoint(meshPoint, camIndex);

        if (!isPointInsideImage(point, camIndex)) {  // fast rejection, fast to compute
            throw UnprojectablePointThroughCamException();
        }
        if (!isMeaningfulPose(meshPoint, camIndex)) {
            throw UnprojectablePointThroughCamException();
        }

        return point;
    }

    bool NCCFaceAccuracyModel::isMeaningfulPose(GLMVec3 &meshPoint, int camIndex)
    {
        return !isIntersecting(meshPoint, camIndex) && !isOppositeView(meshPoint, camIndex);
    }

    bool NCCFaceAccuracyModel::isOppositeView(GLMVec3 &meshPoint, int camIndex)
    {
        GLMVec3 pose = getCameraCenter(camIndex);
        GLMVec3 ray = pose - meshPoint;
        
        RotationMatrix R = getRotationMatrix(camIndex);
        GLMVec3 zDirection = R * zdir;

        return glm::dot(ray, zDirection) > 0.0f;    // if < 0.0 than it sees the object, but we want to know when it is opposite.
    }

    bool NCCFaceAccuracyModel::isIntersecting(GLMVec3 &meshPoint, int camIndex)
    {
        #pragma omp critical
        tree->build();

        GLMVec3 pose = getCameraCenter(camIndex);
        PointD3 cam(pose.x, pose.y, pose.z);
        PointD3 point(meshPoint.x, meshPoint.y, meshPoint.z);
        
        Segment segment_query(cam, point);
        Segment_intersection intersection;
        #pragma omp critical
        intersection = tree->any_intersection(segment_query);  // gives the first intersected primitives, so probably the farer one

        if (intersection) {
            return !isMathemathicalError(intersection, point);
        } else {
            return false;
        }
    }

    bool NCCFaceAccuracyModel::isMathemathicalError(Segment_intersection &intersection, PointD3 &point)
    {
        const PointD3* intersectedPoint = boost::get<PointD3>(&(intersection->first));
        if(intersectedPoint) {
            return CGAL::squared_distance(*intersectedPoint, point) < 0.0001;
        }
        return false;
    }

    bool NCCFaceAccuracyModel::isPointInsideImage(GLMVec2 &point2D, int camIndex)
    {
        return point2D.x < (float)getImageWidth(camIndex) && point2D.x > 0.0f && point2D.y < (float)getImageHeight(camIndex) && point2D.y > 0.0f;
    }

    GLMVec2 NCCFaceAccuracyModel::getProjectedPoint(GLMVec3 &meshPoint, int camIndex)
    {
        GLMVec4 point3D = GLMVec4(meshPoint, 1.0f);
        CameraMatrix P = getCameraMatrix(camIndex);
        GLMVec4 point2D = P * point3D;

        point2D = point2D / point2D.z;

        return GLMVec2(point2D.x, point2D.y);
    }

    int NCCFaceAccuracyModel::getImageWidth(int camIndex)
    {
        return this->cams[camIndex].imageWidth;
    }

    int NCCFaceAccuracyModel::getImageHeight(int camIndex)
    {
        return this->cams[camIndex].imageHeight;
    }

    GLMVec3 NCCFaceAccuracyModel::getCameraCenter(int indexCam)
    {
        return this->cams[indexCam].center;
    }

    RotationMatrix NCCFaceAccuracyModel::getRotationMatrix(int indexCam)
    {
        return this->cams[indexCam].rotation;
    }

    CameraMatrix NCCFaceAccuracyModel::getCameraMatrix(int indexCam)
    {
        return this->cams[indexCam].cameraMatrix;
    }
    



} // namespace meshac
