#pragma once

#include <Eigen/Dense>

#ifdef PLATFORM_LINUX
#include <Navio2/ADC_Navio2.h>
#endif

class BatteryHandler {
public:
    BatteryHandler();
    Eigen::Matrix<double, 2, 1> read_battery();

    Eigen::Matrix<double, 2, 1> get_last_battery_data() const {
        return last_battery_data;
    }

private:
#ifdef PLATFORM_LINUX
    ADC_Navio2 adc;
#endif
    Eigen::Matrix<double, 2, 1> last_battery_data;
};
