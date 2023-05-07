#pragma once

#include <ct/core/core.h>
namespace ct
{
    namespace core
    {
        class MTController : public ct::core::ControlledSystem<2, 1>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            const static int STATE_DIM = 2;
            const static int CONTROL_DIM = 1;
            const static int MEASURE_DIM = 1;

            using state_t = ct::core::StateVector<STATE_DIM>;
            using control_t = ct::core::ControlVector<CONTROL_DIM>;
            using output_t = ct::core::OutputVector<MEASURE_DIM>;

            // constructor
            MTController(double w, const state_t &init_state) : w_(w), init_state_(init_state)
            {
                A_ << 0.0, w,
                    -w, 0.0;
                B_ << 1, 0;
                C_ << 1, 0;
            }
            // copy constructor
            MTController(const MTController &other) : A_(other.A_), B_(other.B_), C_(other.C_), w_(other.w_), init_state_(other.init_state_) {}

            MTController *clone() const override
            {
                return new MTController(*this); // calls copy constructor
            }
            // destructor
            ~MTController() = default;
            // The system dynamics. We override this method which gets called by e.g. the Integrator
            void computeControlledDynamics(const state_t &state,
                                           const time_t &t,
                                           const control_t &control,
                                           state_t &derivative) override
            {
                derivative = A_ * state + B_ * control;
            }

            void computeOutput(const state_t &state,
                               output_t &output) const
            {
                output = C_ * state;
            }

            state_t getInitState() const
            {
                return init_state_;
            }

        private:
            double w_;
            StateMatrix<STATE_DIM> A_;
            StateControlMatrix<STATE_DIM, CONTROL_DIM> B_;
            OutputStateMatrix<MEASURE_DIM, STATE_DIM> C_;
            StateVector<STATE_DIM> init_state_;
        };
    }
}
