// udp_sender.cpp
#include "telemetry/udp_sender.h"
#include <stdexcept>
#include <utility>
#include <mutex> // Needed for std::call_once
#include <chrono>
#include <thread>

using json = nlohmann::json;

#ifdef _WIN32
// Define the flag for Windows WSA initialization
std::once_flag UdpSender::wsa_flag_;
#endif

static inline json vec3ToJson(const Vecf<3>& v) {
    return json::array({ v(0), v(1), v(2) });
}

static inline json vec4ToJson(const Veci<4>& v) {
    return json::array({ v(0), v(1), v(2), v(3) });
}

void UdpSender::platformStartup_() {
#ifdef _WIN32
    // std::call_once guarantees WSAStartup only runs exactly once, 
    // even if multiple threads create a UdpSender simultaneously.
    std::call_once(wsa_flag_, []() {
        WSADATA wsa{};
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
            throw std::runtime_error("WSAStartup failed");
        }
        });
#endif
}

void UdpSender::platformCleanup_() {
#ifdef _WIN32
    // no WSACleanup() here (safe/simple)
#endif
}

UdpSender::UdpSender(const std::string& receiver_ip, uint16_t receiver_port) {
    platformStartup_();

    sock_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ == INVALID_SOCKET) throw std::runtime_error("socket() failed");

    dst_ = {};
    dst_.sin_family = AF_INET;
    dst_.sin_port = htons(receiver_port);

    if (inet_pton(AF_INET, receiver_ip.c_str(), &dst_.sin_addr) != 1) {
        closeSocket_();
        throw std::runtime_error("inet_pton failed for receiver IP: " + receiver_ip);
    }
}

UdpSender::~UdpSender() {
    closeSocket_();
    platformCleanup_();
}

UdpSender::UdpSender(UdpSender&& other) noexcept { *this = std::move(other); }

UdpSender& UdpSender::operator=(UdpSender&& other) noexcept {
    if (this == &other) return *this;
    closeSocket_();
    sock_ = other.sock_; other.sock_ = INVALID_SOCKET;
    dst_ = other.dst_;

    // std::atomic variables cannot be explicitly copied/moved. 
    // We must manually load from the old and store into the new.
    seq_.store(other.seq_.load());
    other.seq_.store(0);

    return *this;
}

void UdpSender::closeSocket_() noexcept {
    if (sock_ == INVALID_SOCKET) return;
#ifdef _WIN32
    ::closesocket(sock_);
#else
    ::close(sock_);
#endif
    sock_ = INVALID_SOCKET;
}

void UdpSender::resetSeq(uint32_t seq) {
    seq_.store(seq); // Safely store the new sequence number
}

bool UdpSender::sendJson_(const json& j) {
    const std::string payload = j.dump();
    if (payload.size() > 1200) return false;

    const int sent = ::sendto(
        sock_, payload.data(), static_cast<int>(payload.size()), 0,
        reinterpret_cast<sockaddr*>(&dst_), static_cast<socklen_t>(sizeof(dst_))
    );
    return (sent != SOCKET_ERROR);
}

// =======================EDIT HERE IF YOU WANT TO ADD===============================

bool UdpSender::sendFromSim(const TelemetryData& data)
{
    // ---- Extract what MATLAB expects ----
    const Vecf<3> euler_est = data.navState.segment<3>(0).cast<float>();
    const Vecf<3> pos_est = data.navState.segment<3>(3).cast<float>();
    const Vecf<3> vel_est = data.navState.segment<3>(6).cast<float>();

    // ModeManager outputs
    const Vecf<3> pos_cmd = data.posCmd.cast<float>();

    // OuterLoop outputs (commanded attitude)
    const Vecf<3> euler_cmd = data.attCmd.cast<float>();
    const Veci<4> PWMCmd = data.pwmCmd.cast<int>();

    // IMU outputs
    const Vecf<3> omega_est = data.imuGyro.cast<float>();
    const Vecf<3> accel = data.imuAccel.cast<float>();

    const float tf = static_cast<float>(data.t);
    const float dtf = static_cast<float>(data.dt);
    const float Hzf = static_cast<float>(data.Hz);
    const float NISf = static_cast<float>(data.NIS);

    json j;

    // seq_++ is now a thread-safe atomic post-increment
    j["seq"] = seq_++;

    j["t"] = tf;
    j["dt"] = dtf;
    j["Hz"] = Hzf;

    j["phase"] = data.phase;
    j["mode"] = data.mode;
    j["armed"] = data.armed;
    j["EKF_Health"] = NISf;

    j["pos_cmd"] = vec3ToJson(pos_cmd);
    j["pos_est"] = vec3ToJson(pos_est);
    j["pwm_cmd"] = vec4ToJson(PWMCmd);
    j["vel_est"] = vec3ToJson(vel_est);
    j["euler_est"] = vec3ToJson(euler_est);

    j["omega_est"] = vec3ToJson(omega_est);
    j["accel"] = vec3ToJson(accel);

    // MATLAB prefers euler_cmd if present
    j["euler_cmd"] = vec3ToJson(euler_cmd);

    return sendJson_(j);
}

void telemetryTask(TelemetryBuffer& shared_buffer, UdpSender& udp) {
    while (true) {
        auto start_time = std::chrono::steady_clock::now();

        // Get the latest telemetry data from the thread-safe buffer
        TelemetryData latest_data = shared_buffer.getLatest();

        // Send telemetry if time has advanced
        if (latest_data.t > 0.0) {
            udp.sendFromSim(latest_data);
        }

        // Sleep until 35ms passed from start of execution (safety margin for ~25Hz)
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        if (elapsed < std::chrono::milliseconds(35)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(35) - elapsed);
        }
    }
}