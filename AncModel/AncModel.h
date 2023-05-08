#pragma once
#include <ct/core/core.h>
#include <StateSpace.h>

namespace ct
{
    namespace core
    {

        class AncModel : public StateSpace<2, 1, 1>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            using Base = StateSpace<2, 1, 1>;
            MAKE_TIME_STATE_CONTROL_OUTPUT_TYPE_FROM_BASE

            // constructor
            AncModel(const state_t &initState) : Base(initState)
            {
                A_ << 0, 1,
                    -3.098 * 1e4, -62.342;
                B_ << 0, 1;
                C_ << -2.638 * 1e4, -55.306;
            }
            // copy constructor
            AncModel(const AncModel &other) : Base(other) {}

            AncModel *clone() const override
            {
                return new AncModel(*this); // calls copy constructor
            }
        };

    }
}