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

	void setGains(double Kp, double Ki, double Kd);
	void setTimestep(double dt);
	double updateControl(double error);

private:
	double Kp;
	double Ki;
	double Kd;
	double integralError;
	double lastError;
	double timeStep;
};
