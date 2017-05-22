#include <opview/BasicGraphicalModel.hpp>


namespace opview {

    BasicGraphicalModel::BasicGraphicalModel(SolverGeneratorPtr solver, GLMVec3List &cams, double goalAngle, double dispersion)
                                             : GraphicalModelBuilder(solver)
    {
        this->cams = cams;
        this->vonMisesConfig = {goalAngle, dispersion};
        this->initShapes();
    }

    BasicGraphicalModel::~BasicGraphicalModel()
    { /*    */ }

    void BasicGraphicalModel::initShapes()
    {
        for (size_t i = 0; i < numVariables(); i++) this->variableIndices.push_back(i);
        for (size_t i = 0; i < numVariables(); i++) this->shape.push_back(numLabels());
    }

    void BasicGraphicalModel::fillModel(GraphicalModelAdder &model, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        GMExplicitFunction vonMises(shape.begin(), shape.end());
        GMSparseFunction constraints(shape.begin(), shape.end(), -10.0);
        GMSparseFunction distances(shape.begin(), shape.end(), -100.0);

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
            LabelType val = logVonMises(pos, centroid, normVector);
            #pragma omp critical
            vonMises(pos[0], pos[1], pos[2]) = val;
        }
    }

    void BasicGraphicalModel::fillConstraintFunction(GMSparseFunction &constraints, GMSparseFunction &distances, GLMVec3 &centroid)
    {
        for (GLMVec3 cam : cams) {
            #pragma omp parallel for collapse(3)
            coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) {
                GLMVec3 pos = scalePoint(GLMVec3(x, y, z));
                addValueToConstraintFunction(constraints, pos, cam, centroid);
            }
            LabelType vals[] = {cam.x, cam.y, cam.z};
            distances.insert(vals, 0.0);
        }
    }

    void BasicGraphicalModel::addValueToConstraintFunction(GMSparseFunction &function, GLMVec3 &point, GLMVec3 &cam, GLMVec3 &centroid)
    {
        double B = glm::distance(point, cam);
        
        GLMVec3 midPoint = (point + cam) / 2.0f;
        GLMVec3 center = GLMVec3(centroid[0], centroid[1], centroid[2]);

        double D = glm::distance(midPoint, center);

        if (B / D > 0.7f) {
            LabelType vals[] = {point.x, point.y, point.z};
            #pragma omp critical
            function.insert(vals, 1.0);
        }
    }

    GLMVec3 BasicGraphicalModel::scalePoint(GLMVec3 point)
    {
        GLMVec3 scaledPoint = point * scale();
        GLMVec3 offset = GLMVec3(offsetX(), offsetY(), offsetZ());
        return scaledPoint + offset;
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

    size_t BasicGraphicalModel::numVariables()
    {
        return VARS;
    }

    size_t BasicGraphicalModel::numLabels()
    {
        return LABELS;
    }    

} // namespace opview
