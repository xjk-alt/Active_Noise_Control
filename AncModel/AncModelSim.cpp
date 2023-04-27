#include <ct/core/core.h>
#include "AncModel.h"
#include "BlankController.h"

int main(int argc, char** argv)
{
    // a damped oscillator has two states, position and velocity
    const size_t state_dim = 2;      // = 2
    const size_t control_dim = 1;  // = 1
    const size_t measure_dim = 1;
    // create a state
    ct::core::StateVector<state_dim> x;
    // we initialize it at a point with unit deflection and zero velocity
    x(0) = 1.0;
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
    ct::core::Integrator<state_dim> integrator(model, ct::core::IntegrationType::RK4);
    // simulate 1000 steps
    double dt = 0.001;
    ct::core::Time t0 = 0.0;
    size_t nSteps = 1000;
    integrator.integrate_n_steps(x, t0, nSteps, dt);
    // print the new state
    std::cout << "state after integration: " << x.transpose() << std::endl;
    return 0;
}