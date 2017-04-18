#ifndef MESH_ACCURACY_THRESHOLD_COLOR_H
#define MESH_ACCURACY_THRESHOLD_COLOR_H

#include <algorithm>
#include <vector>

#include <meshac/alias_definition.hpp>
#include <meshac/Color.hpp>

namespace meshac {

    class ThresholdColor {
    public:
        ThresholdColor();
        ThresholdColor(DoubleList &thresholds, ColorList &colors);
        virtual ~ThresholdColor();

        void addColor(double threshold, Color &color);
        Color getColorFor(double targetAccuracy);
        void clearColors();

    protected:

    private:

        typedef std::pair<double, Color> PairDoubleColor;
        typedef std::vector<PairDoubleColor> PairDoubleColorList;

        static bool pairCompare(const PairDoubleColor& l, const PairDoubleColor& r){ return l.first < r.first; }

        int getIndexForAccuracy(double targetAccuracy);
        PairDoubleColorList::iterator getIteratorToPositionOf(double targetAccuracy);

        PairDoubleColorList colors;
    };

    typedef ThresholdColor * ThresholdColorPtr;

}

#endif // MESH_ACCURACY_THRESHOLD_COLOR_H
