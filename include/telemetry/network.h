#pragma once

#include <Eigen/Dense>

using Vector5d = Eigen::Matrix<double, 5, 1>;

// Initializes the UDP socket for receiving Mocap data.
// Returns 0 on success, -1 on failure.
int openPort(void);

// Reads a datalink packet from the UDP socket and parses it into a Vector5d.
// Returns a Vector5d containing: [North(y), East(x), Down(-z), Yaw, ValidFlag]
// If no data is read or the checksum fails, returns a zero vector.
Vector5d readDatalink(void);
