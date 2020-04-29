#include <Arduino.h>
#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(float mea_e, float est_e, float q) : _err_measure(mea_e), _err_estimate(est_e), _q(q) {}
double KalmanFilter::updateEstimate(float mea) {
    _kalman_gain = _err_estimate/(_err_estimate + _err_measure);
    _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
    _err_estimate =  (1.0 - _kalman_gain)*(_err_estimate + fabs(_last_estimate - _current_estimate)*_q);
    _last_estimate = _current_estimate;

    return _current_estimate;
}

void KalmanFilter::setMeasurementError(float mea_e) { _err_measure = mea_e; }
void KalmanFilter::setEstimateError(float est_e) { _err_estimate = est_e; }
void KalmanFilter::setProcessNoise(float q) { _q = q; }
double KalmanFilter::getKalmanGain() { return _kalman_gain; }
double KalmanFilter::getEstimateError() { return _err_estimate; }
