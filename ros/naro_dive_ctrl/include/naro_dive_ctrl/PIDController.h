/*
 * Naro Nanins
 * Jonas Eichenberger
 * jo.eichenberger@gmail.com
 * 2015
 */

class PIDController {
public:
	PIDController();
	~PIDController();

	void setGains(float Kp, float Ki, float Kd);
	void setTimestep(float dt);
	float updateControl(float error);

private:
	float Kp;
	float Ki;
	float Kd;
	float integralError;
	float lastError;
	float timeStep;
};
