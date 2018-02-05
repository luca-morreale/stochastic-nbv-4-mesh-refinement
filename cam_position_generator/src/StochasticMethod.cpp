#include <opview/StochasticMethod.hpp>


namespace opview {
    
    StochasticMethod::StochasticMethod(CameraGeneralConfiguration &camConfig, std::string &meshFile, GLMVec3List &cams, 
                                            StochasticConfiguration &stoConfig, double offspring, double goalAngle, double dispersion)
    {
        this->stoConfig = stoConfig;
        this->camConfig = camConfig;
        this->meshFile = meshFile;
        this->cams = cams;
        this->OFFSPRING = offspring;
        this->pitch = stoConfig.particles.pitch;
        this->vonMisesConfig = {deg2rad(goalAngle), dispersion};
        this->log = new ExhaustiveReportWriter("stochastic.json");

        createTree();
        formulation = new Formulation(vonMisesConfig, camConfig, tree, cams);
    }

    StochasticMethod::~StochasticMethod()
    {
        delete log;
        delete formulation;
        delete tree;
    }

    OrderedPose StochasticMethod::uniformSamplingStep(GLMVec3 &centroid, GLMVec3 &normVector, int round)
    {
        EigVector5List orientedPoints = uniformPointsGetter();
        OrderedPose poses = computeEnergyForPoses(orientedPoints, centroid, normVector);
        return this->extractBestResults(poses, round);
    }
    
    EigVector5List StochasticMethod::uniformPointsGetter()
    {  
        GLMVec3List points = GaussianSampleGenerator::extractUniformSamples(
                    {std::make_pair(stoConfig.bounds.lower.x, stoConfig.bounds.upper.x), 
                        std::make_pair(stoConfig.bounds.lower.y, stoConfig.bounds.upper.y), 
                        std::make_pair(stoConfig.bounds.lower.z, stoConfig.bounds.upper.z)}, 
                        stoConfig.particles.uniform);
        return insertOrientation(points);
    }

    EigVector5List StochasticMethod::insertOrientation(GLMVec3List &points)
    {
        EigVector5List orientedPoints;

        #pragma omp parallel for collapse(2)
        for (int p = 0; p < points.size(); p++) {
            orientationCycles(stoConfig.particles.deltaDegree) {
                EigVector5 pose;
                pose << points[p].x, points[p].y, points[p].z, this->pitch, deg2rad((float)yaw);
                #pragma omp critical
                orientedPoints.push_back(pose);
            }
        }
        return orientedPoints;
    }

    OrderedPose StochasticMethod::computeEnergyForPoses(EigVector5List &orientedPoints, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        DoubleList values(orientedPoints.size());

        #pragma omp parallel for
        for (int i = 0; i < orientedPoints.size(); i++) {
            values[i] = formulation->computeEnergy(orientedPoints[i], centroid, normVector);
        }

        return orderPoses(orientedPoints, values);
    }

    OrderedPose StochasticMethod::orderPoses(EigVector5List &orientedPoints, DoubleList &values)
    {
        OrderedPose poses;

        #pragma omp parallel for
        for (int p = 0; p < orientedPoints.size(); p++) {
            ValuePose el = ValuePose(values[p], orientedPoints[p]);
            #pragma omp critical
            poses.push(el);
        }
        return poses;
    }

    OrderedPose StochasticMethod::extractBestResults(OrderedPose &poses, int round)
    {
        OrderedPose optima = copy(poses, (size_t)(OFFSPRING*(double)stoConfig.particles.num));
        
        OrderedPose convertedAnglesOptima = convertAngles(optima);
        this->log->append(convertedAnglesOptima, round);

        return optima;
    }

    OrderedPose StochasticMethod::convertAngles(OrderedPose poses)
    {
        OrderedPose optima;
        int c = 0;
        while(!poses.empty() && c < (size_t)(OFFSPRING* getUniformParticles())) {
            auto tmp = poses.top();
            tmp.second[3] = rad2deg(tmp.second[3]);
            tmp.second[4] = rad2deg(tmp.second[4]);
            optima.push(tmp);
            poses.pop();
            c++;
        }
        return optima;
    }

    void StochasticMethod::createTree()
    {
        Polyhedron poly = extractPolyhedron();
        TriangleList triangles = getTriangleList(poly);

        tree = new Tree(triangles.begin(), triangles.end());
    }

    Polyhedron StochasticMethod::extractPolyhedron()
    {
        std::ifstream meshIn(meshFile);
        Polyhedron poly;
        meshIn >> poly;
        meshIn.close();

        return poly;
    }

    TriangleList StochasticMethod::getTriangleList(Polyhedron &poly)
    {
        TriangleList triangles;
        for (Facet_iterator it = poly.facets_begin(); it != poly.facets_end(); it++) {
            Halfedge_facet_circulator p = it->facet_begin();
            Vertex p0 = p->vertex();
            Vertex p1 = (++p)->vertex();
            Vertex p2 = (++p)->vertex();

            triangles.push_back(Triangle(p0->point(), p1->point(), p2->point()));
        }
        return triangles;
    }

    TreePtr StochasticMethod::getTree()
    {
        return tree;
    }

    GLMVec3List StochasticMethod::getCams()
    {
        return cams;
    }

    CameraGeneralConfiguration StochasticMethod::getCamConfig()
    {
        return camConfig;
    }

    StochasticConfiguration StochasticMethod::getStochasticConfig()
    {
        return stoConfig;
    }

    size_t StochasticMethod::getUniformParticles()
    {
        return stoConfig.particles.uniform;
    }

    size_t StochasticMethod::getResamplingParticles()
    {
        return stoConfig.particles.num;
    }

    size_t StochasticMethod::getResamplingSteps()
    {
        return stoConfig.resamplingNum;
    }

    VonMisesConfiguration StochasticMethod::getVonMisesConfig()
    {
        return vonMisesConfig;
    }

    FormulationPtr StochasticMethod::getFormulation()
    {
        return formulation;
    }

    void StochasticMethod::setOffspring(double offspring)
    {
        this->OFFSPRING = offspring;
    }

    double StochasticMethod::getOffspring()
    {
        return OFFSPRING;
    }
    
    GLMVec3 StochasticMethod::lowerBounds()
    {
        return stoConfig.bounds.lower;
    }

    GLMVec3 StochasticMethod::upperBounds()
    {
        return stoConfig.bounds.upper;
    }

    float StochasticMethod::offsetX()
    {
        return stoConfig.bounds.lower.x;
    }

    float StochasticMethod::offsetY()
    {
        return stoConfig.bounds.lower.y;
    }

    float StochasticMethod::offsetZ()
    {
        return 0.0f;
    }

    ExhaustiveReportWriterPtr StochasticMethod::getLogger()
    {
        return log;
    }

    void StochasticMethod::setLogger(ExhaustiveReportWriterPtr log)
    {
        delete this->log;
        this->log = log;
    }
    
} // namespace opview
