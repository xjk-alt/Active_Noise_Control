#pragma once
#include <ControlOutputSystem.h>

namespace ct
{
    namespace core
    {
        template <size_t STATE_DIM, size_t CONTROL_DIM, size_t OUTPUT_DIM, typename SCALAR = double>
        class SystemWrapper : public ControlOutputSystem<STATE_DIM, CONTROL_DIM, OUTPUT_DIM, SCALAR>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            using Base = ControlOutputSystem<STATE_DIM, CONTROL_DIM, OUTPUT_DIM>;
            MAKE_TIME_STATE_CONTROL_OUTPUT_TYPE_FROM_BASE

            SystemWrapper() : Base() { data_.setZero(); }

            SystemWrapper(const SystemWrapper &other) : Base(), data_(other.data_) {}

            SystemWrapper *clone() const override
            {
                retrun new SystemWrapper(*this);
            }

            void setInput(OutputVector<OUTPUT_DIM> &data) { data_ = data; }

            virtual void computeControlledDynamics(const state_t &state,
                                                   const time_t &t,
                                                   const control_t &control,
                                                   state_t &derivative) override
            {
                derivative.setZero();
                // hardware control action ...
            }

            virtual output_t computeOutput(const state_t &state, const time_t &t = 0.0) { return data_; }

        private:
            OutputVector<OUTPUT_DIM> data_;
        }
    }
}
