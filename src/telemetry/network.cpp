#include "telemetry/network.h"
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <iostream>

namespace {
    constexpr int THISPORTNUM = 9001;
    constexpr int BUFFERSIZE = 1024;
    constexpr int DATALINK_MESSAGE_OPTITRACK = 230;

#pragma pack(push, 1)
    struct datalinkHeader_ref {
        uint8_t sync1, sync2, sync3, spare;
        int messageID;
        int messageSize;
        uint32_t hcsum, csum;
    };

    struct onboardMocapClient_ref {
        uint8_t sync1, sync2, sync3, spare;
        int messageID, messageSize;
        uint32_t hcsum, csum;
        float pos_x, pos_y, pos_z;
        float qx, qy, qz, qw;
        int frameNum, valid;
    };
#pragma pack(pop)

    int sock_fd = -1;
    uint8_t rx_buffer[BUFFERSIZE];
    int bytes_read = 0;

    unsigned int computeChecksum(const uint8_t* buf, int byteCount) {
        unsigned int sum1 = 0xffff, sum2 = 0xffff;
        int shortCount = byteCount / 2;
        bool oddLength = (byteCount % 2 != 0);

        while (shortCount > 0) {
            int tlen = std::min(shortCount, 360);
            shortCount -= tlen;
            while (tlen--) {
                sum1 += *buf++;
                sum1 += (*buf++ << 8);
                sum2 += sum1;
            }
        }
        if (oddLength) {
            sum1 += *buf++;
            sum2 += sum1;
        }

        sum1 = (sum1 & 0xffff) + (sum1 >> 16);
        sum2 = (sum2 & 0xffff) + (sum2 >> 16);
        sum1 = (sum1 & 0xffff) + (sum1 >> 16);
        sum2 = (sum2 & 0xffff) + (sum2 >> 16);
        return (sum2 << 16) | sum1;
    }

    bool initSocket() {
        sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd < 0) return false;

        int opt = 1;
        ioctl(sock_fd, FIONBIO, &opt);

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(THISPORTNUM);
        addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(sock_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(sock_fd);
            sock_fd = -1;
            return false;
        }
        return true;
    }
}

Vector5d readDatalink() {
    Vector5d result = Vector5d::Zero();

    if (sock_fd < 0 && !initSocket()) {
        return result;
    }

    int toread = 0;
    if (ioctl(sock_fd, FIONREAD, &toread) < 0 || toread <= 0) {
        return result;
    }

    int n = recvfrom(sock_fd, rx_buffer + bytes_read, BUFFERSIZE - bytes_read, 0, nullptr, nullptr);
    if (n > 0) bytes_read += n;

    int index = 0;
    while (index <= bytes_read - (int)sizeof(datalinkHeader_ref)) {
        if (rx_buffer[index] == 0xa1 && rx_buffer[index + 1] == 0xb2 && rx_buffer[index + 2] == 0xc3) {
            datalinkHeader_ref header;
            std::memcpy(&header, &rx_buffer[index], sizeof(header));

            if (header.messageSize >= (int)sizeof(header) && header.messageSize <= BUFFERSIZE &&
                (index + header.messageSize) <= bytes_read) {

                uint32_t calc_hcsum = computeChecksum(&rx_buffer[index], sizeof(header) - 8);
                uint32_t calc_csum = computeChecksum(&rx_buffer[index + sizeof(header)], header.messageSize - sizeof(header));

                if (calc_hcsum == header.hcsum && calc_csum == header.csum) {
                    if (header.messageID == DATALINK_MESSAGE_OPTITRACK && header.messageSize == sizeof(onboardMocapClient_ref)) {
                        onboardMocapClient_ref mocap;
                        std::memcpy(&mocap, &rx_buffer[index], sizeof(mocap));

                        double yaw = std::atan2(2.0 * (mocap.qw * mocap.qz + mocap.qx * mocap.qy),
                                                1.0 - 2.0 * (mocap.qy * mocap.qy + mocap.qz * mocap.qz));

                        result << mocap.pos_y, mocap.pos_x, -mocap.pos_z, yaw, mocap.valid;
                    }
                    index += header.messageSize;
                    continue;
                }
            }
        }
        index++;
    }

    if (index > 0) {
        if (index >= bytes_read) {
            bytes_read = 0;
        } else {
            std::memmove(rx_buffer, &rx_buffer[index], bytes_read - index);
            bytes_read -= index;
        }
    }

    return result;
}
