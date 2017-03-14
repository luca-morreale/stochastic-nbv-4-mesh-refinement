
#include <CrossRatioTuple.hpp>


namespace meshac {
    
    void CrossRatioTuple::append(GLMVec2 point) 
    {
        if (points.size() < 4) {
            points.push_back(point);
        } else {
            std::sort(points.begin(), points.end(), point2DComparator);
            this->precomputeDistancesBetweenPoints();
        }
    }

    // Computes the cross ratio of the given four points
    double CrossRatioTuple::crossRatio() 
    {
        return (this->xz * this->yt) / (this->yz * this->xt);
    }

    EigVector CrossRatioTuple::jacobian()
    {
        EigVector jacobian(4);

        jacobian[0] = (this->yt * this->zt) / (this->xt * this->xt * this->yz);
        jacobian[1] = -(this->yt * this->xz) / (this->xt * this->yz * this->yz);
        jacobian[2] = (this->yt * this->xy) / (this->xt * this->yz * this->yz);
        jacobian[3] = -(this->xz * this->xy) / (this->xt * this->xt * this->yz);

        return jacobian;
    }


    double CrossRatioTuple::avgDistance()
    {
        return (this->xy + this->yz + this->zt) / 3;
    }

    bool CrossRatioTuple::isInTuple(GLMVec2 point)
    {
        for (GLMVec2 tuplePoint : points) {
            if (glm::epsilonEqual(point, tuplePoint, EPSILON)[0]) {
                return true;
            }
        }
        return false;
    }


    void CrossRatioTuple::precomputeDistancesBetweenPoints()
    {        
        this->xy = glm::distance2(points[0], points[1]);
        this->xz = glm::distance2(points[0], points[2]);
        this->xt = glm::distance2(points[0], points[3]);

        this->yz = glm::distance2(points[1], points[2]);
        this->yt = glm::distance2(points[1], points[3]);

        this->zt = glm::distance2(points[2], points[3]);
    }

    bool CrossRatioTuple::point2DComparator(const GLMVec2 &vecA, const GLMVec2 &vecB)
    {
        if (fabs(vecA[0] - vecB[0]) < SENSIBILITY) {
            return fabs(vecA[1] - vecB[1]) < SENSIBILITY;
        }

        return fabs(vecA[0] - vecB[0]) < SENSIBILITY;
    }
    
    bool CrossRatioTuple::operator<(const CrossRatioTuple &other) const
    {
        if (points.size() < other.points.size()) {
            return true;
        }

        // find better compare
        if (glm::epsilonEqual(points[0], other.points[0], EPSILON)[0] && 
                glm::epsilonEqual(points[1], other.points[1], EPSILON)[0] && 
                glm::epsilonEqual(points[2], other.points[2], EPSILON)[0] && 
                glm::epsilonEqual(points[3], other.points[3], EPSILON)[0]) {
            return true;
        }
        
        return false;
    }


} // namespace meshac

