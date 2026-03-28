#pragma once
#include "common//MathUtils.h"

class MotorModel {
public:
	void arm();
	void disarm();
	bool isArmed() const;

	Vec<4> step(double dt, const Vec<4>& thrust_cmd);
private:
	bool armed = false;
	double tau = 0.05;
	Vec<4> thrustAct = Vec<4>::Zero();
};