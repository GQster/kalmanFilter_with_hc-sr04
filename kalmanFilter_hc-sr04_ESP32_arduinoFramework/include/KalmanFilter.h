#ifndef KalmanFilter_h
#define KalmanFilter_h

#include <math.h>

class KalmanFilter
{
private:
    double _measurement_error;
    double _estimate_error;
    double _process_noise;
    double _k_gain;
    double _latest_estimate;
    double _prior_estimate;

public:
    KalmanFilter(double measurement_error, double estimate_error, double process_noise);
    double updateEstimate(double measurement);
    double returnEstimateError() const;
    double returnCurrentEstimate() const;
};

#endif
