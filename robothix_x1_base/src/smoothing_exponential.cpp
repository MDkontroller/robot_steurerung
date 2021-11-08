#include    <ros/ros.h>

#include    "robothix_x1_base/smoothing_exponential.h"

namespace Filter
{

SmoothingExponential::SmoothingExponential(double smoothingFactor):
    _smoothing_factor(smoothingFactor),
    _input_previous(0.0),
    _output_previous(0.0)
{}

void SmoothingExponential::setNewSmootingFactor(double smoothingFactor)
{
    _smoothing_factor = smoothingFactor;
}

double SmoothingExponential::getSmoothedValue(double input)
{
    double _output_now = _input_previous * _smoothing_factor + ( 1.0 - _smoothing_factor ) * _output_previous;

    _input_previous  = input;
    _output_previous = _output_now;

    return _output_now;
}

void SmoothingExponential::resetFilter()
{
    _input_previous  = 0.0;
    _output_previous = 0.0;
}
    
} // namespace Filter

