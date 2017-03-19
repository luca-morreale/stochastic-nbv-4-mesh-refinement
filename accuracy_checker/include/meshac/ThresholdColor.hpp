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
        ~ThresholdColor();

        void addColor(double threshold, Color &color);
        Color getColorFor(double targetAccuracy);
        void clearColors();

    protected:
        virtual bool hasFoundAccuracy(double targetAccuracy, int pivot);
        virtual void goLeft(int &lowBound, int &pivot, int &upBound, double &refAccuracy);
        virtual void goRight(int &lowBound, int &pivot, int &upBound, double &refAccuracy);
        virtual int getPivot(int lowBound, int upBound);

    private:

        typedef std::pair<double, Color> PairDoubleColor;

        static bool pairCompare(const PairDoubleColor& l, const PairDoubleColor& r){ return l.first < r.first; }

        

        std::vector<PairDoubleColor> colors;
    };

    typedef ThresholdColor * ThresholdColorPtr;

}

#endif // MESH_ACCURACY_THRESHOLD_COLOR_H
