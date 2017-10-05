#include <opview/MiddleburyDatasetScorer.hpp>

namespace opview {

    StochasticConfiguration tmpConfig;
    CameraGeneralConfiguration tmpCam;

    MiddleburyDatasetScorer::MiddleburyDatasetScorer(std::string &cameraPoses, 
                MeshConfiguration &meshConfig, size_t maxPoints, double goalAngle, double dispersion)
                : AutonomousStochasticMethod(tmpCam, meshConfig, tmpConfig, maxPoints, 1.0, goalAngle, dispersion)
    {
        this->cameraPoses = cameraPoses;
        setCameras();
    }

    MiddleburyDatasetScorer::~MiddleburyDatasetScorer()
    {
        cameras.clear();
        views.clear();
    }

    DoubleList MiddleburyDatasetScorer::estimateBestCameraPosition()
    {
        throw std::runtime_error("not implemented method");
    }

    void MiddleburyDatasetScorer::setCameras()
    {
        MiddleburyDatasetReader reader(cameraPoses);
        this->views = reader.getViews();                // views and cameras have the same order
        this->cameras = reader.getCamerasMatrices();
        // this->viewToCameraMap = reader.getViewToCameraMatrixMapping();
    }

    void MiddleburyDatasetScorer::removeView(std::string &view)
    {
        for (int i = 0; i < views.size(); i++) {
            if (views[i].compare(view) == 0) {
                views.erase(views.begin() + i);
                cameras.erase(cameras.begin() + i);
                return;
            }
        }
    }

    std::string MiddleburyDatasetScorer::estimateView()
    {
        StringDoubleMap map = evaluateCameras();
        DoubleList scores = values(map);

        int max = 0;
        for (int i = 1; i < scores.size(); i++) {
            if (scores[i] > scores[max]) {
                max = i;
            }
        }
        return views[max];
    }

    StringDoubleMap MiddleburyDatasetScorer::evaluateCameras()   // analyze all view, if some unnecessary erease it later
    {
        StringDoubleMap scores;

        #pragma omp parallel for
        for (int i = 0; i < cameras.size(); i++) {
            double score = computeScoreForCam(cameras[i]);
            
            #pragma omp critical
            scores.insert({views[i], score});
        }

        return scores;
    }

    double MiddleburyDatasetScorer::computeScoreForCam(Camera &cam)
    {
        GLMVec3ListPair worst = getWorstPointsList();
        GLMVec3List centroids = worst.first;
        GLMVec3List normals = worst.second;
        
        double score = 0.0;

        #pragma omp parallel for reduction(+:score)
        for (int i = 0; i < centroids.size(); i++) {
            score += computeScore(cam, centroids[i], normals[i]);
        }

        return score; 
    }

    double MiddleburyDatasetScorer::computeScore(Camera &cam, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        double vonMises, visibility, projection, constraints;
        auto config = getVonMisesConfig();
        GLMVec3List cams = getCams();
        GLMVec4 t = cam.t;

        #pragma omp parallel sections 
        {
            #pragma omp section
            vonMises = logVonMisesWrapper(t, centroid, normVector, config);
            #pragma omp section
            visibility = visibilityDistribution(cam, centroid, normVector, getTree());
            #pragma omp section
            projection = imageProjectionDistribution(cam, centroid, normVector, getTree());
            #pragma omp section
            constraints = computeBDConstraint(t, centroid, cams);
        }
        
        return vonMises + visibility + projection + constraints;
    }

    double MiddleburyDatasetScorer::logVonMisesWrapper(GLMVec4 &pose, GLMVec3 &centroid, GLMVec3 &normVector, VonMisesConfiguration &vonMisesConfig)
    {
        GLMVec3 point(pose.x, pose.y, pose.z);
        return -logVonMises(point, centroid, normVector, vonMisesConfig);
    }

    double MiddleburyDatasetScorer::visibilityDistribution(Camera &cam, GLMVec3 &centroid, GLMVec3 &normalVector, TreePtr tree)
    {
        if (isMeaningfulPose(cam, centroid, tree)){
            return 1.0;
        }
        return -10.0;
    }

    double MiddleburyDatasetScorer::imageProjectionDistribution(Camera &cam, GLMVec3 &centroid, GLMVec3 &normalVector, TreePtr tree)
    {
        if (!isPointInsideImage(cam, centroid)) {  // fast rejection, fast to compute.
            return -10.0;
        }
        if (!isMeaningfulPose(cam, centroid, tree)) {
            return -10.0;
        }
        
        GLMVec2 point = getProjectedPoint(cam, centroid);
        double centerx = (double)cam.K[0][2];   // also at line 207
        double centery = (double)cam.K[1][2];    // also at line 207
        double sigma_x = (double)cam.K[0][2] * 2.0 / 3.0;   // also at line 207
        double sigma_y = (double)cam.K[1][2] * 2.0 / 3.0;    // also at line 207

        return logBivariateGaussian(point.x, point.y, centerx, centery, sigma_x, sigma_y);  // positive because gaussian have highest value in the center
    }

    double MiddleburyDatasetScorer::computeBDConstraint(GLMVec4 &pose, GLMVec3 &centroid, GLMVec3List &cams)
    {
        GLMVec3 point(pose.x, pose.y, pose.z);
        double constraint = 0.0;

        #pragma omp parallel for reduction(+:constraint)
        for (int c = 0; c < cams.size(); c++) {
            constraint += computeBDConstraint(point, centroid, cams[c]);
        }
        return constraint;
    }

    double MiddleburyDatasetScorer::computeBDConstraint(GLMVec3 &pose, GLMVec3 &centroid, GLMVec3 &cam)
    {
        double D, B;
        #pragma omp parallel sections
        {
            #pragma omp section
            B = glm::distance(pose, cam);
            #pragma omp section
            D = std::min(glm::distance(cam, centroid), glm::distance(pose, centroid));
        }
        
        if (B / D < BD_TERRESTRIAL_ARCHITECTURAL) {
            return -10.0;
        }
        return 1.0;
    }


    bool MiddleburyDatasetScorer::isMeaningfulPose(Camera &cam, GLMVec3 &centroid, TreePtr tree)
    {
        return isPointInsideImage(cam, centroid) && !isIntersecting(cam, centroid, tree) && !isOppositeView(cam, centroid);
    }

    bool MiddleburyDatasetScorer::isOppositeView(Camera &cam, GLMVec3 &centroid)
    {
        CameraMatrix E = cam.E;

        GLMVec4 point = GLMVec4(centroid, 1.0f) * E;
        return point.z < 0.0;
    }

    bool MiddleburyDatasetScorer::isIntersecting(Camera &cam, GLMVec3 &centroid, TreePtr tree)
    {
        Point pose(cam.t.x, cam.t.y, cam.t.z);
        Point point(centroid.x, centroid.y, centroid.z);
        
        Segment segment_query(pose, point);
        if (segment_query.is_degenerate()) return true;

        Segment_intersection intersection = tree->any_intersection(segment_query);  // gives the first intersected primitives, so probably the farer one

        if (intersection) {
            return !isMathemathicalError(intersection, point);
        } else {
            return false;
        }
    }

    bool MiddleburyDatasetScorer::isMathemathicalError(Segment_intersection &intersection, Point &point)
    {
        const Point* intersectedPoint = boost::get<Point>(&(intersection->first));
        if(intersectedPoint) {
            return CGAL::squared_distance(*intersectedPoint, point) < 0.0001;
        }
        return false;
    }

    bool MiddleburyDatasetScorer::isPointInsideImage(Camera &cam, GLMVec3 &centroid)
    {
        float size_x = cam.K[0][2] * 2.0f;
        float size_y = cam.K[1][2] * 2.0f;
        GLMVec2 point2D = getProjectedPoint(cam, centroid);

        return point2D.x > 0.0f && point2D.x < size_x && point2D.y > 0.0f && point2D.y < size_y;
    }

    GLMVec2 MiddleburyDatasetScorer::getProjectedPoint(Camera &cam, GLMVec3 &centroid)
    {
        GLMVec4 point3D = GLMVec4(centroid, 1.0f);
        GLMVec4 point2D = point3D * cam.P;
        point2D = point2D / point2D.z;
        
        return GLMVec2(point2D.x, point2D.y);
    }


} // namespace opview
