// udp_sender.h
#pragma once
#include <nlohmann/json.hpp>
#include <cstdint>
#include <string>
#include <atomic> // Added for thread-safe sequence counter
#include <mutex>  // Added for std::once_flag
#include <optional>

#include "common/MathUtils.h"
#include "sensors/ImuSim.h"
#include "control/OuterLoop.h"
#include "guidance/ModeManager.h"

struct TelemetryData {
    double t = 0.0;
    double dt = 0.0;
    double Hz = 0.0;
    Vec<15> navState = Vec<15>::Zero();
    Vec<3> posCmd = Vec<3>::Zero();
    int phase = 0;
    int mode = 0;
    Vec<3> attCmd = Vec<3>::Zero();
    Vec<3> imuGyro = Vec<3>::Zero();
    Vec<3> imuAccel = Vec<3>::Zero();
    bool armed = false;
    double NIS = 0.0;
    Vec<4> pwmCmd = Vec<4>::Zero();
};

class TelemetryBuffer {
private:
    std::optional<TelemetryData> data_;
    std::mutex mtx_;

public:
    void update(const TelemetryData& new_data) {
        std::lock_guard<std::mutex> lock(mtx_);
        data_ = new_data;
    }

    std::optional<TelemetryData> getLatest() {
        std::lock_guard<std::mutex> lock(mtx_);
        return data_;
    }
};

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
using socklen_t = int;
using socket_handle_t = SOCKET;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR   (-1)
using socket_handle_t = int;
#endif

class UdpSender {
public:
    UdpSender(const std::string& receiver_ip, uint16_t receiver_port);
    ~UdpSender();

    UdpSender(const UdpSender&) = delete;
    UdpSender& operator=(const UdpSender&) = delete;
    UdpSender(UdpSender&&) noexcept;
    UdpSender& operator=(UdpSender&&) noexcept;

    void resetSeq(uint32_t seq = 0);

    // Sends the provided telemetry data.
    bool sendFromSim(const TelemetryData& data);

private:
    bool sendJson_(const nlohmann::json& j);
    void closeSocket_() noexcept;
    static void platformStartup_();
    static void platformCleanup_();

private:
    socket_handle_t sock_ = INVALID_SOCKET;
    sockaddr_in dst_{};

    // Thread-safe sequence counter
    std::atomic<uint32_t> seq_{ 0 };

#ifdef _WIN32
    // Thread-safe flag for Windows socket initialization
    static std::once_flag wsa_flag_;
#endif
};

// Thread task for sending telemetry data at 25Hz
void telemetryTask(TelemetryBuffer& shared_buffer, UdpSender& udp);