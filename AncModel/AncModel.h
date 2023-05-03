#pragma once
#include <ct/core/core.h> // as usual, include CT
#include <TrajectoryLogger.h>
// create a class that derives from ct::core::System

class AncModel : public ct::core::ControlledSystem<2, 1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    const static int STATE_DIM = 2;
    const static int CONTROL_DIM = 1;
    const static int MEASURE_DIM = 1;

    typedef ct::core::StateVector<STATE_DIM> state_t;
    typedef ct::core::ControlVector<CONTROL_DIM> control_t;
    typedef ct::core::OutputVector<MEASURE_DIM> output_t;

    // constructor
    AncModel(const state_t &init_state)
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
    void computeControlledDynamics(const state_t &state,
                                   const time_t &t,
                                   const control_t &control,
                                   state_t &derivative) override
    {
        derivative = A_ * state + B_ * control;
    }

    void computeOutput(const state_t &state,
                       output_t &measure) const
    {
        measure = C_.transpose() * state;
    }

    state_t getInitState() const
    {
        return init_state_;
    }

private:
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> A_;
    Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> B_;
    Eigen::Matrix<double, STATE_DIM, MEASURE_DIM> C_;
    ct::core::StateVector<STATE_DIM> init_state_;
};