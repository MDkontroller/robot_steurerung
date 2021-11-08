#include    <ros/ros.h>

namespace Filter
{

class SmoothingExponential
{
    public:
        SmoothingExponential(double smoothingFactor);

        void setNewSmootingFactor(double smoothingFactor);
        double getSmoothedValue(double input);
        void resetFilter();

    private:
        double _smoothing_factor;
        double _input_previous, _output_previous;
};

} // namespace Filter