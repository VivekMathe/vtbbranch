#pragma once

#include <Eigen/Dense>

using Vector5d = Eigen::Matrix<double, 5, 1>;

// Reads a datalink packet from the UDP socket and parses it into a Vector5d.
// Returns a Vector5d containing: [North(y), East(x), Down(-z), Yaw, ValidFlag]
// If no data is read or the checksum fails, returns a zero vector.
// Automatically initializes the port on the first call.
Vector5d readDatalink(void);
