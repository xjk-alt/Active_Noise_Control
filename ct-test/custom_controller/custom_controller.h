#pragma once
#include <ct/core/core.h>  // as usual, include CT
class CustomController : public ct::core::Controller<2, 1>
{
public:
    static const size_t state_dim = 2;    // two states
    static const size_t control_dim = 1;  // one control action
    CustomController(const ct::core::ControlVector<control_dim>& uff,  // feedforward control
        const double& kp,                                              // P gain
        const double& kd                                               // D gain
        )
        : uff_(uff), kp_(kp), kd_(kd)
    {
    }
    ~CustomController() {}
    CustomController(const CustomController& other) : uff_(other.uff_), kp_(other.kp_), kd_(other.kd_) {}
    CustomController* clone() const override
    {
        return new CustomController(*this);  // calls copy constructor
    }
    void computeControl(const ct::core::StateVector<state_dim>& state,
        const double& t,
        ct::core::ControlVector<control_dim>& controlAction) override
    {
        controlAction = uff_;                                 // apply feedforward control
        controlAction(0) -= kp_ * state(0) + kd_ * state(1);  // add feedback control
    }
private:
    ct::core::ControlVector<control_dim> uff_;  
    double kp_;                                 
    double kd_;                                 
};