

#include <memory>

class Pid
{
public:
    Pid();
    ~Pid();

    // Setters
    void setKp(float const& KpInp);
    void setKi(float const& KiInp);
    void setKd(float const& KdInp);
    void setTau(float const& tauInp);
    void setOutMin(float const& outMinInp);
    void setOutMax(float const& outMaxInp);
    void setIntegratorMin(float const& integratorMinInp);
    void setIntegratorMax(float const& integratorMaxInp);
    void setSampleT(float const& sampleTInp);
    void setIntegrator(float const& integratorInp);
    void setDifferentiator(float const& differentiatorInp);
    void setPreviousError(float const& previousErrorInp);
    void setPreviousMeasurement(float const& previousMeasurementInp);
    void setOutput(float const& outputInp);

    // Getters
    float getKp() const;
    float getKi() const;
    float getKd() const;
    float getTau() const;
    float getOutMin() const;
    float getOutMax() const;
    float getIntegratorMin() const;
    float getIntegratorMax() const;
    float getSampleT() const;
    float getIntegrator() const;
    float getDifferentiator() const;
    float getPreviousError() const;
    float getPreviousMeasurement() const;
    float getOutput() const;
private:
    struct ReservedData;
    std::unique_ptr<ReservedData> reserved{nullptr};
};


