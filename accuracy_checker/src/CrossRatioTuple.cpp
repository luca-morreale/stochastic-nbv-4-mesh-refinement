
#include <meshac/CrossRatioTuple.hpp>


namespace meshac {
    
    void CrossRatioTuple::append(GLMVec2 &point) 
    {
        if (points.size() < 4) {
            points.push_back(point);
        } 
        if (points.size() < 4) {
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

        return jacobian.transpose();
    }


    double CrossRatioTuple::avgDistance()
    {
        return (this->xy + this->yz + this->zt) / 3;
    }

    bool CrossRatioTuple::isInTuple(GLMVec2 &point)
    {
        for (GLMVec2 tuplePoint : points) {
            if (glm::all(glm::epsilonEqual(point, tuplePoint, EPSILON))) {
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

    std::string CrossRatioTuple::to_string()
    {
        std::string out;
        std::string letters[] = {"a", "b", "c", "d"};
        for (int i = 0; i < points.size(); i++) {
            out += letters[i] + ": [" + std::to_string(points[i].x) + "," + std::to_string(points[i].y) + "] ";
        }
        return out;
    }
    
    bool CrossRatioTuple::operator<(const CrossRatioTuple &other) const
    {
        for (int i = 0; i < points.size(); i++) {
            if (glm::any(glm::lessThan(points[i], other.points[i]))) {
                return true;
            }
        }
        return false;
    }


} // namespace meshac

