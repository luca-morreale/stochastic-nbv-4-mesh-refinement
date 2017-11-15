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
        GMExplicitFunction constraints(shape.begin(), shape.end());
        
        fillObjectiveFunction(vonMises, centroid, normVector);
        addFunctionTo(vonMises, model, variableIndices);
        
        fillConstraintFunction(constraints, centroid);
        addFunctionTo(constraints, model, variableIndices);
    }

    void BasicGraphicalModel::fillObjectiveFunction(GMExplicitFunction &vonMises, GLMVec3 &centroid, GLMVec3 &normVector)
    {
        #pragma omp parallel for collapse(3)
        coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) {
            GLMVec3 pos = scalePoint(GLMVec3(x, y, z));
            LabelType val = Formulation::logVonMisesWrapper(pos, centroid, normVector, vonMisesConfig);

            #pragma omp critical
            vonMises(x, y, z) = val;
        }
    }

    void BasicGraphicalModel::fillConstraintFunction(GMExplicitFunction &constraints, GLMVec3 &centroid)
    {
        #pragma omp parallel for collapse(3)
        coordinatecycles(0, numLabels(), 0, numLabels(), 0, numLabels()) {
            size_t coords[] = {(size_t)x, (size_t)y, (size_t)z};
            GLMVec3 pos = scalePoint(GLMVec3(x, y, z));
            LabelType val = Formulation::computeBDConstraint(pos, centroid, cams);

            #pragma omp critical
            constraints(coords) = val;
        }
    }

    // from discrete to continuous space
    GLMVec3 BasicGraphicalModel::scalePoint(GLMVec3 point)
    {
        GLMVec3 scaledPoint = point * scale();
        GLMVec3 offset = GLMVec3(offsetX(), offsetY(), offsetZ());
        return scaledPoint + offset;
    }

    // from continuous space to discrete
    GLMVec3 BasicGraphicalModel::unscalePoint(GLMVec3 point)
    {
        GLMVec3 offset = GLMVec3(offsetX(), offsetY(), offsetZ());
        return (point - offset) / scale();
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
