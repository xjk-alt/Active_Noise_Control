#ifndef PLOTTING_ENABLED
#define PLOTTING_ENABLED
#endif

#include <ct/core/core.h>
#include <ct/optcon/optcon.h>
#include "AncModel.h"
#include "BlankController.h"
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

int main(int argc, char **argv)
{
    std::ofstream stream("output.bin", std::ios::binary);
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
    // we initialize it at a point with unit deflection and zero velocity
    x(0) = 0.0;
    x(1) = 0.0;

    std::shared_ptr<AncModel> model(new AncModel(x));
    // create our controller
    double kp = 10;
    double kd = 1;
    ct::core::ControlVector<control_dim> uff;
    uff << 2.0;
    std::shared_ptr<BlankController> controller(new BlankController());
    // assign our controller
    model->setController(controller);
    // create an integrator
    ct::core::Integrator<state_dim> integrator(model, ct::core::IntegrationType::EULER);
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

        model->computeOutput(x, y);
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