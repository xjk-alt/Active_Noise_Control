#ifndef PLOTTING_ENABLED
#define PLOTTING_ENABLED
#endif

#include <ct/core/core.h>
#include <ct/optcon/optcon.h>
#include "AncModel.h"
#include "utils/utils.hpp"
#include <msgpack.hpp>
#include <vector>
#include <TransferFunc.h>

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

using AncModel = ct::core::AncModel;
using TF2 = ct::core::TransferFunc<2>;

int main(int argc, char **argv)
{
    std::ofstream stream("../../data/AncTFStepResponse.bin", std::ios::binary);
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

    Eigen::Vector3d N, D;
    N << 0, -55.306, -2.638*1e4;
    D << 1, 62.342, 3.098*1e4;

    std::shared_ptr<TF2> model(new TF2(N, D, x));
    // create our controller
    double kp = 10;
    double kd = 1;
    ct::core::ControlVector<control_dim> uff;
    uff << 2.0;
    std::shared_ptr<ct::core::ConstantController<state_dim, control_dim>> controller(new ct::core::ConstantController<state_dim, control_dim>(u));
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

        y = model->computeOutput(x);
        data.y.push_back(y(0));

        u = model->getLastControlAction();
        data.u.push_back(u(0));
    }
    // print the new state
    std::cout << "state after integration: " << x.transpose() << std::endl;

    packer.pack(data);
    stream.close();

    Eigen::Matrix<double, 2, 2> a;
    Eigen::Matrix<double, 2, 1> b;
    Eigen::Matrix<double, 1, 2> c;

    model->getSystemMatrices(a, b, c);
    std::cout << a << std::endl
              << std::endl;
    std::cout << b << std::endl
              << std::endl;
    std::cout << c << std::endl
              << std::endl;


    return 0;
}