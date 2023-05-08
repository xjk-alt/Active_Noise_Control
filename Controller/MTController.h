#pragma once
#include <ct/core/core.h>
#include <StateSpace.h>

// \dot w = S w - sign(Re{W(w)}) G y
// u = G^\top w

namespace ct
{
    namespace core
    {

        class MTController : public StateSpace<2, 1, 1>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            using Base = StateSpace<2, 1, 1>;
            MAKE_TIME_STATE_CONTROL_OUTPUT_TYPE_FROM_BASE

            // constructor
            MTController(double w_star, double gain, const state_t &initState = state_t()) : Base(initState), w_star_(w_star)
            {
                A_ << 0, w_star,
                    -w_star, 0;
                B_ << gain, 0;
                C_ << 1, 0;
            }
            // copy constructor
            MTController(const MTController &other) : Base(other) {}

            MTController *clone() const override
            {
                return new MTController(*this); // calls copy constructor
            }

        protected:
            double w_star_;
            double gain;
        };
    }
}