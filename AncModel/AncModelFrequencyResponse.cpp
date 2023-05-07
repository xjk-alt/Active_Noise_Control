#ifndef PLOTTING_ENABLED
#define PLOTTING_ENABLED
#endif

#include <ct/core/core.h>
#include <ct/optcon/optcon.h>
#include "AncModel.h"
#include "utils/utils.hpp"
#include <msgpack.hpp>
#include <vector>

typedef std::vector<double> array_t;

typedef struct
{
    array_t t;
    array_t x0;
    array_t x1;
    array_t y;
    array_t u;
    MSGPACK_DEFINE_MAP(t, x0, x1, y, u);
} datapack_t;

class SecondOrderOscillatorController : public ct::core::Controller<2, 1>
{
public:
    static const size_t state_dim = 2;   // two states
    static const size_t control_dim = 1; // one control action
    SecondOrderOscillatorController(double w_n, double dt, const ct::core::StateVector<2> &x) : w_n_(w_n), t_(0.0), dt_(dt), x_(x),
                                                                                                ptr_model_(new ct::core::SecondOrderSystem(w_n, 0.0, 0.0)),
                                                                                                ptr_integrator_(new ct::core::Integrator<2>(ptr_model_, ct::core::IntegrationType::RK4)) {}
    ~SecondOrderOscillatorController() {}
    SecondOrderOscillatorController(const SecondOrderOscillatorController &other) : ptr_model_(other.ptr_model_), ptr_integrator_(other.ptr_integrator_) {}
    SecondOrderOscillatorController *clone() const override
    {
        return new SecondOrderOscillatorController(*this); // calls copy constructor
    }
    void computeControl(const ct::core::StateVector<state_dim> &state,
                        const double &t,
                        ct::core::ControlVector<control_dim> &controlAction) override
    {
        ptr_integrator_->integrate_n_steps(x_, t_, 1, dt_);
        t_ += dt_;
        controlAction.setZero();
        controlAction(0) = x_(0);
    }

private:
    double w_n_;
    double t_;
    double dt_;
    ct::core::StateVector<2> x_;
    std::shared_ptr<ct::core::SecondOrderSystem> ptr_model_;
    std::shared_ptr<ct::core::Integrator<2>> ptr_integrator_;
};

int main(int argc, char **argv)
{
    std::ofstream stream("../../data/AncModelFrequencyResponse.bin", std::ios::binary);
    msgpack::packer<std::ofstream> packer(stream);
    datapack_t data;

    // a damped oscillator has two states, position and velocity
    const size_t state_dim = 2;   // = 2
    const size_t control_dim = 1; // = 1
    const size_t measure_dim = 1;
    // create a state
    ct::core::StateVector<state_dim> x;
    ct::core::OutputVector<measure_dim> y;
    ct::core::ControlVector<control_dim> u;
    u << 1.0;
    // we initialize it at a point with unit deflection and zero velocity
    x(0) = 0.0;
    x(1) = 0.0;

    std::shared_ptr<ct::core::AncModel> model(new ct::core::AncModel(x));
    // create our controller
    double kp = 10;
    double kd = 1;
    ct::core::ControlVector<control_dim> uff;
    uff << 2.0;

    ct::core::StateVector<2> w0;
    w0(0) = 1.0;
    w0(1) = 0.0;
    std::shared_ptr<SecondOrderOscillatorController> controller(new SecondOrderOscillatorController(5 * 2 * 3.1415926, 1e-3, w0));
    // assign our controller
    model->setController(controller);
    // create an integrator
    ct::core::Integrator<state_dim> integrator(model, ct::core::IntegrationType::RK4);
    // simulate 1000 steps
    double dt = 0.001;
    ct::core::Time t0 = 0.0;
    size_t nSteps = 1000;

    for (int i = 0; i < nSteps; i++)
    {
        integrator.integrate_n_steps(x, t0, 1, dt);
        t0 += dt;

        data.t.push_back(t0);
        data.x0.push_back(x(0));
        data.x1.push_back(x(1));

        y = model->computeOutput(x, t0);
        data.y.push_back(y(0));

        controller->computeControl(x, t0, u);
        data.u.push_back(u(0));
    }
    // print the new state
    std::cout << "state after integration: " << x.transpose() << std::endl;

    packer.pack(data);
    stream.close();

    return 0;
}