#pragma once
#include <ct/core/core.h>  // as usual, include CT
class BlankController : public ct::core::Controller<2, 1>
{
public:
    static const size_t state_dim = 2;    // two states
    static const size_t control_dim = 1;  // one control action
    BlankController() {}
    ~BlankController() {}
    BlankController(const BlankController& other) {}
    BlankController* clone() const override
    {
        return new BlankController(*this);  // calls copy constructor
    }
    void computeControl(const ct::core::StateVector<state_dim>& state,
        const double& t,
        ct::core::ControlVector<control_dim>& controlAction) override
    {
        controlAction.setZero();
    }
private:
};