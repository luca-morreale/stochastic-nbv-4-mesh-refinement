
#include <meshac/ThresholdColor.hpp>

namespace meshac {
    
    ThresholdColor::ThresholdColor()
    { /*    */ }

    ThresholdColor::ThresholdColor(DoubleList &thresholds, ColorList &colors)
    {
        for (int i = 0; i < thresholds.size(); i++) {
            this->colors.push_back(std::make_pair(thresholds[i], colors[i]));
        }

        std::sort(this->colors.begin(), this->colors.end(), &ThresholdColor::pairCompare);
    }

    ThresholdColor::~ThresholdColor()
    {
        this->clearColors();
    }

    void ThresholdColor::addColor(double threshold, Color &color)
    {
        PairDoubleColor pair = std::make_pair(threshold, color);
        PairDoubleColorList::iterator it = this->getIteratorToPositionOf(threshold);

        colors.insert(it, pair);    // insert before iterator
    }

    Color ThresholdColor::getColorFor(double targetAccuracy)
    {
        int index = this->getIndexForAccuracy(targetAccuracy);
        return this->colors[index].second;
    }

    int ThresholdColor::getIndexForAccuracy(double targetAccuracy)
    {
        PairDoubleColorList::iterator low = this->getIteratorToPositionOf(targetAccuracy);
        return low - this->colors.begin();
    }

    ThresholdColor::PairDoubleColorList::iterator ThresholdColor::getIteratorToPositionOf(double targetAccuracy)
    {
        return std::upper_bound(this->colors.begin(), this->colors.end(), std::make_pair(targetAccuracy, Color(0,0,0,0)), &ThresholdColor::pairCompare);
    }

    void ThresholdColor::clearColors()
    {
        this->colors.clear();
    }

}
