#include <meshac/GeneralIndexFaceAccuracyModel.hpp>

namespace meshac {

    GeneralIndexFaceAccuracyModel::GeneralIndexFaceAccuracyModel(std::string &meshFile, SfMData &data, std::string &pathPrefix) : FaceAccuracyModel(meshFile)
    {
        this->imgFilepath = data.camerasPaths_;
        this->points = data.points_;
        this->cams = data.camerasList_;
        this->point3DTo2DThroughCam.assign(this->points.size(), std::map<int, GLMVec2>());

        this->fixImagesPath(pathPrefix);
        
        this->convertTriangleToIndex();
        
        this->initAffineTriangle();
        
        this->initTree();
        
        this->projectMeshPoints();
    }

    GeneralIndexFaceAccuracyModel::~GeneralIndexFaceAccuracyModel()
    {
        delete tree;
    }

    void GeneralIndexFaceAccuracyModel::initAffineTriangle()
    {
        float ax = (float)TRIANGLE_SIDE / 2.0f;
        float ay = (float)TRIANGLE_SIDE * std::sin(deg2rad(60.0f));

        destTriangle[0] = CVPoint2(0, 0);
        destTriangle[1] = CVPoint2(ax, ay);
        destTriangle[2] = CVPoint2(TRIANGLE_SIDE, 0);

        setTriangularMask();
    }

    void GeneralIndexFaceAccuracyModel::setTriangularMask()  // NOTE I can not know the right size of the mask ahead of time
    {
        CVPoint corners[1][3];
        corners[0][0] = destTriangle[0];
        corners[0][1] = destTriangle[1];
        corners[0][2] = destTriangle[2];

        const CVPoint* corner_list[1] = { corners[0] };

        const int num_points = 3;
        const int num_polygons = 1;
        const int line_type = 8;
        CVMat mask(getImageHeight(this->cams[0]), getImageWidth(this->cams[0]), CV_8UC3, cv::Scalar(0,0,0));
        cv::fillPoly(mask, corner_list, &num_points, num_polygons, cv::Scalar(255, 255, 255));

        this->triangularMask = mask;
    }

    void GeneralIndexFaceAccuracyModel::initTree()
    {
        TriangleList facets = this->getFaces();
        this->tree = new Tree(facets.begin(), facets.end());
    }

    void GeneralIndexFaceAccuracyModel::convertTriangleToIndex()
    {
        FaceIndexList newFaces;
        TriangleList facets = this->getFaces();

        #pragma omp parallel for
        for (int f = 0; f < facets.size(); f++) {
            try {
                FaceIndex face;
                for (int v = 0; v < 3; v++) {
                    
                    Point vertex = facets[f].vertex(v);
                    size_t index = retreiveIndex(vertex);
                    face.set(v, index);
                }
                #pragma omp critical
                newFaces.push_back(face);
            } catch (UnexpectedPointException &ex) { }
        }

        this->faces = newFaces;
    }

    int GeneralIndexFaceAccuracyModel::retreiveIndex(Point &vertex)
    {
        GLMVec3 point(vertex.x(), vertex.y(), vertex.z());
        return retreiveIndex(point);
    }

    int GeneralIndexFaceAccuracyModel::retreiveIndex(GLMVec3 &point)
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

    void GeneralIndexFaceAccuracyModel::fixImagesPath(std::string &pathPrefix)
    {
        std::for_each(this->imgFilepath.begin(), this->imgFilepath.end(), [pathPrefix](std::string &path) { path.insert(0, pathPrefix); } );
    }

    double GeneralIndexFaceAccuracyModel::getAccuracyForFace(GLMVec3 &a, GLMVec3 &b, GLMVec3 &c)
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

    double GeneralIndexFaceAccuracyModel::getAccuracyForFace(int faceIndex)
    {
        if (faceIndex >= faces.size() || faceIndex < 0) {
            throw UndefinedFaceIndexException(faceIndex);
        }

        ListMappingGLMVec2 mappings = getMappings(faceIndex);

        IntList commonCams = selectCommonCameras(mappings);

        if (commonCams.size() < 2) return -10.0;

        mappings = removeUnusedMapping(mappings, commonCams);

        CVMatList triangles = projectTriangles(mappings, commonCams);

        return computeIndex(triangles);
    }

    ListMappingGLMVec2 GeneralIndexFaceAccuracyModel::getMappings(int faceIndex)
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
    IntList GeneralIndexFaceAccuracyModel::selectCommonCameras(ListMappingGLMVec2 &mappings)
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

    ListMappingGLMVec2 GeneralIndexFaceAccuracyModel::removeUnusedMapping(ListMappingGLMVec2 &mappings, IntList &commonCams)
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

    CVMatList GeneralIndexFaceAccuracyModel::projectTriangles(ListMappingGLMVec2 &mappings, IntList &commonCams)
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

    CVMat GeneralIndexFaceAccuracyModel::generateAffineTransform(GLMVec2 &a, GLMVec2 &b, GLMVec2 &c)
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

    CVMat GeneralIndexFaceAccuracyModel::applyAffine(CVMat &affine, int camIndex)
    {
        CVMat src, projected;
        src = cv::imread(this->imgFilepath[camIndex]);   // 0 is grey scale but we need color!!
        projected = CVMat::zeros(src.rows, src.cols, src.type());

        cv::warpAffine(src, projected, affine, projected.size());

        return cropTriangle(projected).rowRange(0, TRIANGLE_SIDE).colRange(0, TRIANGLE_SIDE);
    }

    CVMat GeneralIndexFaceAccuracyModel::cropTriangle(CVMat &projectedImage)
    {
        CVMat result(projectedImage.rows, projectedImage.cols, projectedImage.type());        
        cv::bitwise_and(projectedImage, triangularMask, result);
        
        return result;
    }

    IntList GeneralIndexFaceAccuracyModel::unionCamIndex(ListMappingGLMVec2 &mappings)
    {
        IntList commonCams;

        for (int i = 0; i < mappings.size(); i++) {
            IntList cams = keys(mappings[i]);
            std::sort(cams.begin(), cams.end());

            commonCams.insert(commonCams.end(), cams.begin(), cams.end());
        }

        return commonCams;
    }

    void GeneralIndexFaceAccuracyModel::projectMeshPoints()
    {
        #pragma omp parallel for collapse(2)
        for (int p = 0; p < this->points.size(); p++) {
            for (int c = 0; c < this->cams.size(); c++) {
                try {
                    GLMVec2 point = projectThrough(points[p], cams[c], tree);
                    #pragma omp critical
                    this->point3DTo2DThroughCam[p].insert(std::make_pair(c, point));
                } catch (const UnprojectablePointThroughCamException &ex) 
                { } // do nothing because it makes no sense to project it
            }
        }
    }

} // namespace meshac
