#pragma once
#include <StateSpace.h>
#include <Eigen/Dense>

namespace ct
{
    namespace core
    {
        template <size_t STATE_DIM>
        class TransferFunc : public StateSpace<STATE_DIM, 1, 1>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            using Base = StateSpace<STATE_DIM, 1, 1>;
            MAKE_TIME_STATE_CONTROL_OUTPUT_TYPE_FROM_BASE

            // constructor
            TransferFunc(const Eigen::Matrix<double, 1, STATE_DIM + 1> &N, const Eigen::Matrix<double, 1, STATE_DIM + 1> &D, const state_t &initState = state_t()) : Base(initState)
            {

                Base::A_.topRightCorner(STATE_DIM - 1, STATE_DIM - 1) = Eigen::Matrix<double, STATE_DIM - 1, STATE_DIM - 1>::Identity();
                Base::B_(STATE_DIM - 1) = 1;

                for (int i = 1; i <= STATE_DIM; i++)
                {
                    Base::C_(STATE_DIM - i) = N(i);
                    Base::A_(STATE_DIM - 1, STATE_DIM - i) = -D(i);
                }
            }
            // copy constructor
            TransferFunc(const TransferFunc &other) : Base(other) {}
        };

    }
}