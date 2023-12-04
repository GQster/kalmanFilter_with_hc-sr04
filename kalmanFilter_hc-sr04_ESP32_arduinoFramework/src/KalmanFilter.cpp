#include <cmath>
#include "KalmanFilter.h"


/*
 * The KalmanFilter class constuctor takes three arguments:
 * 1. Measurement error: 
 *      Higher value will make the filter less responsive to measurements
 * 2. Estimate error: 
 *      Higher value will make the filter more responsive to measurements (lower confidence in the current estimate)
 * 3. Process noise:
        Higher value will make the filter more responsive to changes in the measurements (and more sensitive to noise)
*/
KalmanFilter::KalmanFilter(double measurement_error, double estimate_error, double process_noise)
    : _measurement_error(measurement_error),
      _estimate_error(estimate_error),
      _process_noise(process_noise),
      _k_gain(0),
      _latest_estimate(0),
      _prior_estimate(0)
{
}

double KalmanFilter::updateEstimate(double measurement)
{
    // Kalman gain calculation
    _k_gain = _estimate_error / (_estimate_error + _measurement_error);

    // Update estimate
    _latest_estimate = _prior_estimate + _k_gain * (measurement - _prior_estimate);

    // Update estimate error
    _estimate_error = (1.0 - _k_gain) * _estimate_error + std::fabs(_prior_estimate - _latest_estimate) * _process_noise;

    // Update prior estimate for the next iteration
    _prior_estimate = _latest_estimate;

    return _latest_estimate;
}

double KalmanFilter::returnEstimateError() const
{
    return _estimate_error;
}

double KalmanFilter::returnCurrentEstimate() const
{
    return _latest_estimate;
}
