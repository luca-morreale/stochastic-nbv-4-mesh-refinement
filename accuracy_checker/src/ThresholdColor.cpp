
#include <meshac/ThresholdColor.hpp>

namespace meshac {

    Color::Color(float r, float g, float b, float a)
    {
        this->r = r;
        this->g = g;
        this->b = b;
        this->a = a;
    }

    std::string Color::string()
    {
        return std::to_string(this->r) + " " + std::to_string(this->g) + " " + std::to_string(this->b) + " " + std::to_string(this->a);
    }


    
    ThresholdColor::ThresholdColor()
    { /*    */ }

    ThresholdColor::ThresholdColor(std::vector<double> thresholds, std::vector<Color> colors)
    {
        for (int i = 0; i < thresholds.size(); i++) {
            this->colors.push_back(std::make_pair(thresholds[i], colors[i]));
        }

        std::sort(this->colors.begin(), this->colors.end(), &ThresholdColor::pairCompare);
    }

    ThresholdColor::~ThresholdColor()
    {
        this->colors.clear();
    }
        

    void ThresholdColor::addColor(double threshold, Color color)
    {
        // insert in position in order
    }

    Color ThresholdColor::getColorFor(double targetAccuracy)
    {
        int lowBound = 0;
        int pivot = this->colors.size() / 2;
        int upBound = this->colors.size() - 1;

        // Sanity check if 1 and 0 have not been inserted
        if (this->colors[upBound].first <= targetAccuracy) {
            return this->colors[upBound].second;    // best color
        } else if (this->colors[lowBound].first >= targetAccuracy) {
            return this->colors[lowBound].second;   // worst color
        }

        double refAccuracy = this->colors[pivot].first;

        while (1) {
            if (refAccuracy > targetAccuracy) {
                goLeft(lowBound, pivot, upBound, refAccuracy);
            } else if (hasFoundAccuracy(targetAccuracy, pivot)) {
                return this->colors[pivot].second;
            } else {
                goRight(lowBound, pivot, upBound, refAccuracy);
            }
        }

        //auto low = std::lower_bound(this->colors.begin(), this->colors.end(), std::make_pair(targetAccuracy, Color(0,0,0,0)), &ThresholdColor::pairCompare);
        //int index = low - this->colors.begin();
        //return this->colors[index].second;

    }

    bool ThresholdColor::hasFoundAccuracy(double targetAccuracy, int pivot)
    {
        return this->colors[pivot].first == targetAccuracy || (this->colors[pivot].first < targetAccuracy && targetAccuracy < this->colors[pivot+1].first);
    }

    void ThresholdColor::goLeft(int &lowBound, int &pivot, int &upBound, double &refAccuracy) 
    {
        upBound = pivot;
        pivot = getPivot(lowBound, upBound);
        refAccuracy = this->colors[pivot].first;
    }

    void ThresholdColor::goRight(int &lowBound, int &pivot, int &upBound, double &refAccuracy) 
    {
        lowBound = pivot;
        pivot = getPivot(lowBound, upBound);
        refAccuracy = this->colors[pivot].first;
    }

    int ThresholdColor::getPivot(int lowBound, int upBound)
    {
        return lowBound + (upBound - lowBound) / 2;
    }

    void ThresholdColor::clearColors()
    {
        this->colors.clear();
    }

}
