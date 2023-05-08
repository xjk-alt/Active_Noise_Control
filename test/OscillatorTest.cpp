#include <TransferFunc.h>
#include <iostream>
#include <Eigen/Dense>
#include "msgpack.hpp"
typedef std::vector<double> array_t;

typedef struct
{
    array_t t, w0, w1;
    MSGPACK_DEFINE_MAP(t, w0, w1);
} datapack_t;

int main(void)
{
    const double w_star = 1.0 * 3.1415926 * 2.0;

    std::ofstream stream("../../data/mtTest.bin", std::ios::binary);
    msgpack::packer<std::ofstream> packer(stream);
    datapack_t data;

    using Oscillator2 = ct::core::SecondOrderOscillator;
    Oscillator2 oscillator(w_star, (Eigen::Vector2d() << 1.0, 0.0).finished());

    Eigen::Matrix<double, 2, 2> a;
    Eigen::Matrix<double, 2, 1> b;
    Eigen::Matrix<double, 1, 2> c;

    oscillator.getSystemMatrices(a, b, c);
    std::cout << a << std::endl
              << std::endl;
    std::cout << b << std::endl
              << std::endl;
    std::cout << c << std::endl
              << std::endl;

    std::shared_ptr<Oscillator2> model(new Oscillator2(oscillator));
    ct::core::Integrator<2> integrator(model, ct::core::IntegrationType::RK4);
    // simulate 1000 steps
    double dt = 0.001;
    ct::core::Time t0 = 0.0;
    size_t nSteps = 100000;

    Oscillator2::state_t x = model->getInitState();

    for (int i = 0; i < nSteps; i++)
    {
        integrator.integrate_n_steps(x, t0, 1, dt);
        t0 += dt;

        data.t.push_back(t0);
        data.w0.push_back(x(0));
        data.w1.push_back(x(1));
    }

    packer.pack(data);
    stream.close();


    return 0;
}