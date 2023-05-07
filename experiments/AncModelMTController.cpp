#ifndef PLOTTING_ENABLED
#define PLOTTING_ENABLED
#endif

#include <ct/core/core.h>
#include <ct/optcon/optcon.h>
#include "AncModel/AncModel.h"
#include "Controller/MTController.h"
#include "utils/utils.hpp"
#include <msgpack.hpp>
#include <vector>

typedef std::vector<double> array_t;

typedef struct
{
    array_t t;
    array_t z0;
    array_t z1;
    array_t z2;
    array_t z3;
    array_t y;
    array_t u;
    MSGPACK_DEFINE_MAP(t, z0, z1, z2, z3, y, u);
} datapack_t;

class Model : public ct::core::System<2 + 2>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    const static int STATE_DIM = 2;
    const static int CONTROL_DIM = 1;
    const static int MEASURE_DIM = 1;

    using state_t = ct::core::StateVector<STATE_DIM>;
    using control_t = ct::core::ControlVector<CONTROL_DIM>;
    using output_t = ct::core::OutputVector<MEASURE_DIM>;

    Model(const ct::core::AncModel &model, const ct::core::MTController &controller) : model_(model), controller_(controller) {}
    // copy constructor
    Model(const Model &other) : model_(other.model_), controller_(other.controller_) {}

    void computeDynamics(const ct::core::StateVector<2 + 2> &state,
                         const time_t &t,
                         ct::core::StateVector<2 + 2> &derivative) override
    {
        ct::core::StateVector<2 + 2> augmented_state;
        ct::core::StateVector<2> modelState, modelDerivate;
        ct::core::StateVector<2> controllerState, controllerDerivate;
        spliteStates(state, modelState, controllerState);
        output_t y;
        model_.computeOutput(modelState, y);
        output_t u_;
        control_t u;
        controller_.computeOutput(controllerState, u_);
        u = u_;
        model_.computeControlledDynamics(modelState, t, u, modelDerivate);
        controller_.computeControlledDynamics(controllerState, t, y * 0, controllerDerivate);
        derivative = mergeStates(modelDerivate, controllerDerivate);
    }

    Model *clone() const override
    {
        return new Model(*this); // calls copy constructor
    }
    // destructor
    ~Model() = default;

    ct::core::StateVector<4> getInitState() const
    {
        return mergeStates(model_.getInitState(), controller_.getInitState());
    }

    ct::core::AncModel model_;
    ct::core::MTController controller_;

private:
    void spliteStates(const ct::core::StateVector<4> &mergedStates, ct::core::StateVector<2> &modelStates, ct::core::StateVector<2> &controllerStates)
    {
        modelStates.segment(0, 2) = mergedStates.segment(0, 2);
        controllerStates.segment(0, 2) = mergedStates.segment(2, 2);
    }

    ct::core::StateVector<4> mergeStates(ct::core::StateVector<2> modelStates, ct::core::StateVector<2> controllerStates) const
    {
        ct::core::StateVector<4> mergedStates;
        mergedStates.segment(0, 2) = modelStates.segment(0, 2);
        mergedStates.segment(2, 2) = controllerStates.segment(0, 2);
        return mergedStates;
    }
};

template <size_t STATE_DIM, typename... Systems>
class ComplexModel : public ct::core::System<STATE_DIM>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ComplexModel(const Systems &...systems) : ct::core::System<STATE_DIM>(), systems_(systems)... {}

private:
    Systems... systems_;
}

int main(int argc, char **argv)
{
    std::ofstream stream("../../data/AncModelMTController.bin", std::ios::binary);
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

    ct::core::AncModel model(x);
    // create our controller
    double kp = 10;
    double kd = 1;
    ct::core::ControlVector<control_dim> uff;
    uff << 2.0;

    ct::core::StateVector<2> w0;
    w0(0) = 1.0;
    w0(1) = 0.0;
    ct::core::MTController controller(5 * 2 * 3.1415926, w0);
    // assign our controller

    std::shared_ptr<Model> ptr_sys(new Model(model, controller));
    // create an integrator
    ct::core::Integrator<4> integrator(ptr_sys, ct::core::IntegrationType::RK4);
    // simulate 1000 steps
    double dt = 0.001;
    ct::core::Time t0 = 0.0;
    size_t nSteps = 1000;
    ct::core::StateVector<4> z = ptr_sys->getInitState();

    for (int i = 0; i < nSteps; i++)
    {
        integrator.integrate_n_steps(z, t0, 1, dt);
        t0 += dt;

        data.t.push_back(t0);
        data.z0.push_back(z(0));
        data.z1.push_back(z(1));
        data.z2.push_back(z(2));
        data.z3.push_back(z(3));

        // model->computeOutput(x, y);
        // data.y.push_back(y(0));

        // controller->computeControl(x, t0, u);
        // data.u.push_back(u(0));
    }
    // print the new state
    std::cout << "state after integration: " << z.transpose() << std::endl;

    packer.pack(data);
    stream.close();

    return 0;
}