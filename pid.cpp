#include "pid.h"

struct Pid::ReservedData
{
public:
    ReservedData(float kpInp = 0.0, float kiInp = 0.0, float kdInp = 0.0, 
    float tauInp = 0.0, float outMinInp = 0.0, float outMaxInp = 0.0,
    float intMinInp = 0.0, float intMaxInp = 0.0, float sampleTInp = 0.0,
    float integratorInp = 0.0, float differentiatorInp = 0.0, 
    float prevErrInp = 0.0, float prevMeasurementInp = 0.0, 
    float outputInp = 0.0);
    float kp, ki, kd;
    float tau;
    float outMin, outMax;
    float integratorMin, integratorMax;
    float sampleT;
    float integrator, differentiator;
    float previousError, previousMeasurement;
    float output;
};


Pid::Pid()
{
    ReservedData();
    this->reserved = std::make_unique<Pid::ReservedData>();
}

void Pid::pidTune(float const& kpInp, float const& kiInp, float const& kdInp)
{
    setKp(kpInp);
    setKd(kdInp);
    setKi(kiInp);
}

Pid::ReservedData::ReservedData(float kpInp = 0.0, float kiInp = 0.0, float kdInp = 0.0, 
    float tauInp = 0.0, float outMinInp = 0.0, float outMaxInp = 0.0,
    float intMinInp = 0.0, float intMaxInp = 0.0, float sampleTInp = 0.0,
    float integratorInp = 0.0, float differentiatorInp = 0.0, 
    float prevErrInp = 0.0, float prevMeasurementInp = 0.0, 
    float outputInp = 0.0)
{
    kp = kpInp; 
    ki = kiInp; 
    kd = kdInp; 
    tau = tauInp; 
    outMin = outMinInp; 
    outMax = outMaxInp; 
    integratorMin = intMinInp; 
    integratorMax = intMaxInp; 
    sampleT = sampleTInp; 
    integrator = integratorInp; 
    differentiator = differentiatorInp; 
    previousError = prevErrInp; 
    previousMeasurement = prevMeasurementInp; 
    output = outputInp;

}


// Setters
void Pid::setKp(float const& kpInp) { this->reserved->kp = kpInp; }
void Pid::setKi(float const& KiInp) { this->reserved->ki = KiInp; }
void Pid::setKd(float const& kdInp) { this->reserved->kd = kdInp; }
void Pid::setTau(float const& tauInp) { this->reserved->tau = tauInp; }
void Pid::setOutMin(float const& outMinInp) { this->reserved->outMin = outMinInp; }
void Pid::setOutMax(float const& outMaxInp) { this->reserved->outMax = outMaxInp; }
void Pid::setIntegratorMin(float const& integratorMinInp) { this->reserved->integratorMin = integratorMinInp; }
void Pid::setIntegratorMax(float const& integratorMaxInp) { this->reserved->integratorMax = integratorMaxInp; }
void Pid::setSampleT(float const& sampleTInp) { this->reserved->sampleT = sampleTInp; }
void Pid::setIntegrator(float const& integratorInp) { this->reserved->integrator = integratorInp; }
void Pid::setDifferentiator(float const& differentiatorInp) { this->reserved->differentiator = differentiatorInp; }
void Pid::setPreviousError(float const& previousErrorInp) { this->reserved->previousError = previousErrorInp; }
void Pid::setPreviousMeasurement(float const& previousMeasurementInp) { this->reserved->previousMeasurement = previousMeasurementInp; }
void Pid::setOutput(float const& outputInp) { this->reserved->output = outputInp; }

// Getters
float Pid::getKp() const { return this->reserved->kp; }
float Pid::getKi() const { return this->reserved->Ki; }
float Pid::getKd() const { return this->reserved->kd; }
float Pid::getTau() const { return this->reserved->tau; }
float Pid::getOutMin() const { return this->reserved->outMin; }
float Pid::getOutMax() const { return this->reserved->outMax; }
float Pid::getIntegratorMin() const { return this->reserved->integratorMin; }
float Pid::getIntegratorMax() const { return this->reserved->integratorMax; }
float Pid::getSampleT() const { return this->reserved->sampleT; }
float Pid::getIntegrator() const { return this->reserved->integrator; }
float Pid::getDifferentiator() const { return this->reserved->differentiator; }
float Pid::getPreviousError() const { return this->reserved->previousError; }
float Pid::getPreviousMeasurement() const { return this->reserved->previousMeasurement; }
float Pid::getOutput() const { return this->reserved->output; }