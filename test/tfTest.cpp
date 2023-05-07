#include <TransferFunc.h>
#include <iostream>
#include <Eigen/Dense>

int main(void)
{
    Eigen::Vector4d N, D;
    N << 5, 6, 7, 8;
    D << 1, 2, 3, 4;

    ct::core::TransferFunc<3> model(N.transpose(), D.transpose());

    Eigen::Matrix<double, 3, 3> a;
    Eigen::Matrix<double, 3, 1> b;
    Eigen::Matrix<double, 1, 3> c;

    model.getSystemMatrices(a, b, c);
    std::cout << a << std::endl
              << std::endl;
    std::cout << b << std::endl
              << std::endl;
    std::cout << c << std::endl
              << std::endl;
    return 0;
}