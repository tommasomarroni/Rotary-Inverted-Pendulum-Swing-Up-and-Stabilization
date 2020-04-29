#pragma once

class KalmanFilter {
    public:
        KalmanFilter(float mea_e, float est_e, float q);
        double updateEstimate(float mea);
        void setMeasurementError(float mea_e);
        void setEstimateError(float est_e);
        void setProcessNoise(float q);
        double getKalmanGain();
        double getEstimateError();

    private:
        double _err_measure;
        double _err_estimate;
        double _q;
        double _current_estimate;
        double _last_estimate;
        double _kalman_gain;
};
