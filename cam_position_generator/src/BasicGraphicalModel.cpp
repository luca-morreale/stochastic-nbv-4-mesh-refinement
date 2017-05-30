#include <opview/BasicGraphicalModel.hpp>


namespace opview {

    BasicGraphicalModel::BasicGraphicalModel(SolverGeneratorPtr solver, GLMVec3List &cams, double goalAngle, double dispersion)
                                             : GraphicalModelBuilder(solver)
    {
        this->cams = cams;
        this->vonMisesConfig = {deg2rad(goalAngle), dispersion};
        this->initShapes();
    }

    BasicGraphicalModel::~BasicGraphicalModel()
    { /*    */ }

    void BasicGraphicalModel::initShapes()
    {
        variableIndices.clear();
        shape.clear();
        for (size_t i = 0; i < numVariables(); i++) {
            this->variableIndices.push_back(i);
            this->shape.push_back(numLabels());
        }
    }

    void BasicGraphicalModel::fillModel(GraphicalModelAdder &model, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        GMExplicitFunction vonMises(shape.begin(), shape.end());
        GMSparseFunction constraints(shape.begin(), shape.end(), 0.0);
        GMSparseFunction distances(shape.begin(), shape.end(), 0.0);

        fillObjectiveFunction(vonMises, centroid, normVector);
        addFunctionTo(vonMises, model, variableIndices);
        
        fillConstraintFunction(constraints, distances, centroid);
        addFunctionTo(constraints, model, variableIndices);
        addFunctionTo(distances, model, variableIndices);
    }

    void BasicGraphicalModel::fillObjectiveFunction(GMExplicitFunction &vonMises, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        #pragma omp parallel for collapse(3)
        coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) {
            GLMVec3 pos = scalePoint(GLMVec3(x, y, z));
            LabelType val = -logVonMises(pos, centroid, normVector);
            #pragma omp critical
            vonMises(x, y, z) = val;
        }
    }

    void BasicGraphicalModel::fillConstraintFunction(GMSparseFunction &constraints, GMSparseFunction &distances, GLMVec3 &centroid)
    {
        for (GLMVec3 cam : cams) {
            #pragma omp parallel for collapse(3)
            coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) {
                GLMVec3 pos = scalePoint(GLMVec3(x, y, z));
                addValueToConstraintFunction(constraints, pos, cam, centroid, GLMVec3(x, y, z));
            }
            
            addCameraPointConstrain(distances, cam);
        }
    }

    void BasicGraphicalModel::addValueToConstraintFunction(GMSparseFunction &function, GLMVec3 &point, GLMVec3 &cam, GLMVec3 &centroid, GLMVec3 spacePos)
    {
        double B = glm::distance(point, cam);
        double D = std::min(glm::distance(cam, centroid), glm::distance(point, centroid));

        if (B / D <= BD_TERRESTRIAL_ARCHITECTURAL) {
            LabelType vals[] = {spacePos.x, spacePos.y, spacePos.z};
            #pragma omp critical
            function.insert(vals, -1.0);
        }
    }

    void BasicGraphicalModel::addCameraPointConstrain(GMSparseFunction &distances, GLMVec3 &cam)
    {
        GLMVec3 discreteSpacePoint = unscalePoint(cam);
        #pragma omp parallel for collapse(3)
        coordinatecycles(0, 2, 0, 2, 0, 2) {
            size_t coords[] = {
                (x) ? (size_t)std::floor(discreteSpacePoint.x) : (size_t)std::ceil(discreteSpacePoint.x),
                (y) ? (size_t)std::floor(discreteSpacePoint.y) : (size_t)std::ceil(discreteSpacePoint.y),
                (z) ? (size_t)std::floor(discreteSpacePoint.z) : (size_t)std::ceil(discreteSpacePoint.z)
            };
            #pragma omp critical
            distances.insert(coords, -100.0);
        }
    }

    GLMVec3 BasicGraphicalModel::scalePoint(GLMVec3 point)
    {
        GLMVec3 scaledPoint = point * scale();
        GLMVec3 offset = GLMVec3(offsetX(), offsetY(), offsetZ());
        return scaledPoint + offset;
    }

    GLMVec3 BasicGraphicalModel::unscalePoint(GLMVec3 point)
    {
        GLMVec3 offset = GLMVec3(offsetX(), offsetY(), offsetZ());
        return (point - offset) / scale();
    }
    
    LabelType BasicGraphicalModel::logVonMises(GLMVec3 &point, GLMVec3 &centroid, GLMVec3 &normalVector)
    {
        GLMVec3 v = point - centroid;
        return logVonMises(v, normalVector);
    }

    LabelType BasicGraphicalModel::logVonMises(GLMVec3 &v, GLMVec3 &normalVector)
    { 
        double dotProduct = glm::dot(normalVector, v);
        double normProduct = glm::l2Norm(normalVector) * glm::l2Norm(v);
        double angle = dotProduct / normProduct;
 
        return logVonMises(angle);
    }

    LabelType BasicGraphicalModel::logVonMises(double angle)
    {
        return std::cos(angle - vonMisesConfig.goalAngle) * vonMisesConfig.dispersion - std::log(2 * M_PI) - logBessel0(vonMisesConfig.dispersion); 
    }

    LabelType BasicGraphicalModel::logBessel0(double k)   // log of I0(x) as approximately x − 1/2 log(2 *pi * x)     https://math.stackexchange.com/questions/376758/exponential-approximation-of-the-modified-bessel-function-of-first-kind-equatio 
    { 
        double logArg = 2.0f * M_PI * k; 
        return k - 0.5f * std::log(logArg);
    }

    VonMisesConfigurationPtr BasicGraphicalModel::vonMisesConfiguration()
    {
        return &vonMisesConfig;
    }

    void BasicGraphicalModel::setVonMisesConfiguration(VonMisesConfiguration vonMisesConfig)
    {
        this->vonMisesConfig = vonMisesConfig;
    }

    GLMVec3List BasicGraphicalModel::getCams()
    {
        return cams;
    }
    
    void BasicGraphicalModel::setCams(GLMVec3List &cams)
    {
        this->cams = cams;
    }

    size_t BasicGraphicalModel::numVariables()
    {
        return VARS;
    }

    size_t BasicGraphicalModel::numLabels()
    {
        return LABELS;
    }   

} // namespace opview
