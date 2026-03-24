#include "telemetry/TelemetryTask.h"

TelemetryTask::TelemetryTask(UdpSender& udp_sender) : udp_(udp_sender), last_battery_time_(std::chrono::steady_clock::now()) {
}

TelemetryTask::~TelemetryTask() {
    stop();
}

void TelemetryTask::updateState(const TelemetryState& new_state) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_ = new_state;
}

void TelemetryTask::stop() {
    running_ = false;
}

void TelemetryTask::loop() {
    while (running_) {
        // Sleep first to maintain 25Hz loop rate
        std::this_thread::sleep_for(std::chrono::milliseconds(40));

        TelemetryState state_copy;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_copy = current_state_;
        }

        // We use the modified sendFromSim that takes primitives
        udp_.sendFromSim(
            state_copy.t,
            state_copy.dt,
            state_copy.Hz,
            state_copy.navState,
            state_copy.posCmd,
            state_copy.phase,
            state_copy.mode,
            state_copy.attCmd,
            state_copy.armed,
            state_copy.NIS,
            state_copy.PWMcmd
        );

        // Check if 1 second has elapsed to send battery data
        auto current_time = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_battery_time_).count() >= 1) {
            Eigen::Vector2d batt_data = battery_handler_.read_battery();
            udp_.sendBattery(state_copy.t, batt_data(0), batt_data(1));
            last_battery_time_ = current_time;
        }
    }
}
