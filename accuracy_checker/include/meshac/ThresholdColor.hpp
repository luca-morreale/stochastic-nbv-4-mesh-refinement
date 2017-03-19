#ifndef MESH_ACCURACY_THRESHOLD_COLOR_H
#define MESH_ACCURACY_THRESHOLD_COLOR_H

#include <algorithm>
#include <vector>


namespace meshac {
    
    struct Color {
        float r = 0;
        float g = 0;
        float b = 0;
        float a = 0;

        Color(float r, float g, float b, float a);

        std::string string();
    };

    class ThresholdColor {
    public:
        ThresholdColor();
        ThresholdColor(std::vector<double> thresholds, std::vector<Color> colors);
        ~ThresholdColor();

        void addColor(double threshold, Color color);
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