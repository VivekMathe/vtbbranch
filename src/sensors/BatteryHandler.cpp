#include "sensors/BatteryHandler.h"

BatteryHandler::BatteryHandler() {
    last_battery_data = Eigen::Matrix<double, 2, 1>::Zero();
#ifdef PLATFORM_LINUX
    adc.initialize();
#endif
}

Eigen::Matrix<double, 2, 1> BatteryHandler::read_battery() {
#ifdef PLATFORM_LINUX
    // Read raw ADC millivolts
    double raw_voltage_mv = adc.read(2);
    double raw_current_mv = adc.read(3);

    // Apply calibration scaling
    // Using constants used in Navio2 SAS
    double actual_voltage_mv = raw_voltage_mv * 10.88;
    double actual_current_ma = raw_current_mv * 200.0;

    last_battery_data(0) = actual_voltage_mv;
    last_battery_data(1) = actual_current_ma;
#endif

    return last_battery_data;
}
