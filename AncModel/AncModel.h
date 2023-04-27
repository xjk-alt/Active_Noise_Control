#pragma once
#include <ct/core/core.h> // as usual, include CT
// create a class that derives from ct::core::System
class AncModel : public ct::core::ControlledSystem<2, 1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    const static int STATE_DIM = 2;
    const static int CONTROL_DIM = 1;
    const static int MEASURE_DIM = 1;

    // constructor
    AncModel(const ct::core::StateVector<STATE_DIM> &init_state)
    {
        A_ << 0, 1,
            -3.098 * 1e4, -62.342;
        B_ << 0, 1;
        C_ << -2.638 * 1e4, -55.306;
        init_state_ = init_state;
    }
    // copy constructor
    AncModel(const AncModel &other) : A_(other.A_), B_(other.B_), C_(other.C_) {}
    AncModel *clone() const override
    {
        return new AncModel(*this); // calls copy constructor
    }
    // destructor
    ~AncModel() = default;
    // The system dynamics. We override this method which gets called by e.g. the Integrator
    void computeControlledDynamics(const ct::core::StateVector<STATE_DIM> &state,
                                   const time_t &t,
                                   const ct::core::ControlVector<CONTROL_DIM> &control,
                                   ct::core::StateVector<STATE_DIM> &derivative) override
    {
        derivative = A_ * state + B_ * control;
    }

    void computeMeasurements(const ct::core::StateVector<STATE_DIM> &state,
                             ct::core::StateVector<MEASURE_DIM> &measure) const
    {
        measure = C_.transpose() * state;
    }

    ct::core::StateVector<STATE_DIM> getInitState() const
    {
        return init_state_;
    }

private:
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> A_;
    Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> B_;
    Eigen::Matrix<double, STATE_DIM, MEASURE_DIM> C_;
    ct::core::StateVector<STATE_DIM> init_state_;
};