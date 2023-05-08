#ifndef PLOTTING_ENABLED
#define PLOTTING_ENABLED
#endif

#include <vector>

#include <ct/core/core.h>
#include <ct/optcon/optcon.h>
#include <msgpack.hpp>

#include "TransferFunc.h"
#include "MTController.h"
#include "ControlOutputSystem.h"

typedef std::vector<double> array_t;

typedef struct
{
    array_t t, x0, x1, w0, w1, d, y, u;
    MSGPACK_DEFINE_MAP(t, x0, x1, w0, w1, d, y, u);
} datapack_t;

using TF2 = ct::core::TransferFunc<2>;
using CloseLoopSys_ = ct::core::ControlOutputSystem<6, 0, 6>;
using Oscillator2 = ct::core::SecondOrderOscillator;
using ct::core::MTController;
using Eigen::Vector2d;
using Eigen::Vector3d;

class CloseLoopSys : public CloseLoopSys_
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Base = CloseLoopSys_;
    MAKE_TIME_STATE_CONTROL_OUTPUT_TYPE_FROM_BASE

    CloseLoopSys(const TF2 &plant, const MTController &controller, const Oscillator2 &exosys) : plant_(plant), controller_(controller), exosys_(exosys)
    {
        initState_ << plant_.getInitState(), controller_.getInitState(), exosys.getInitState();
    };

    CloseLoopSys(const CloseLoopSys &other) : Base(other), plant_(other.plant_), controller_(other.controller_), exosys_(other.exosys_) {}

    CloseLoopSys *clone() const override
    {
        return new CloseLoopSys(*this); // calls copy constructor
    }

    void computeControlledDynamics(const state_t &state,
                                   const time_t &t,
                                   const control_t &control,
                                   state_t &derivative) override
    {
        TF2::state_t plant_state, plant_derivative;
        plant_state << state.segment(0, 2);
        MTController::state_t controller_state, controller_derivative;
        controller_state << state.segment(2, 2);
        Oscillator2::state_t exosys_state, exosys_derivative;
        exosys_state << state.segment(4, 2);

        exosys_.computeDynamics(exosys_state, t, exosys_derivative);

        controller_.computeControlledDynamics(controller_state, t, plant_.computeOutput(plant_state), controller_derivative);

        plant_.computeControlledDynamics(plant_state, t, controller_.computeOutput(controller_state) - exosys_.computeOutput(exosys_state), plant_derivative);

        derivative << plant_derivative, controller_derivative, exosys_derivative;
    }

    output_t computeOutput(const state_t &state, const time_t &t) { return state; }

    // protected:
    TF2 plant_;
    MTController controller_;
    Oscillator2 exosys_;
};

int main(int argc, char **argv)
{
    std::ofstream stream("../../data/MTControllerTest.bin", std::ios::binary);
    msgpack::packer<std::ofstream> packer(stream);
    datapack_t data;

    const double w_star = 1.0 * 3.1415926 * 2.0;

    Vector2d w0, x0;
    w0 << 1.0, 0.0;
    x0 << 1.0, 0.5;

    MTController controller(w_star, -1.0);

    Oscillator2 oscillator(w_star, w0);

    TF2 plant((Vector3d() << 0, 2, -2).finished(), (Vector3d() << 1, 2, 3).finished(), x0);

    std::shared_ptr<CloseLoopSys> model(new CloseLoopSys(plant, controller, oscillator));

    ct::core::Integrator<6> integrator(model, ct::core::IntegrationType::RK4);
    // simulate 1000 steps
    double dt = 0.001;
    ct::core::Time t0 = 0.0;
    size_t nSteps = 100000;

    CloseLoopSys::state_t x = model->getInitState();

    for (int i = 0; i < nSteps; i++)
    {
        integrator.integrate_n_steps(x, t0, 1, dt);
        t0 += dt;

        data.t.push_back(t0);
        data.x0.push_back(x(0));
        data.x1.push_back(x(1));
        data.w0.push_back(x(2));
        data.w1.push_back(x(3));
        data.d.push_back(model->exosys_.computeOutput(x.segment(4, 2), t0)(0));
        data.y.push_back(model->plant_.computeOutput(x.segment(0, 2), t0)(0));
        data.u.push_back(model->controller_.computeOutput(x.segment(2, 2), t0)(0));
    }
    // print the new state
    std::cout << "state after integration: " << x.transpose() << std::endl;

    packer.pack(data);
    stream.close();

    return 0;
}