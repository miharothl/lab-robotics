#include <iostream>
#include <math.h>
#include <tuple>
#include <Eigen/Core> // Eigen Library
#include <Eigen/LU>   // Eigen Library

// Read
// https://eigen.tuxfamily.org/dox/group__QuickRefPage.html

using namespace std;
using namespace Eigen;

float measurements[3] = { 1, 2, 3 };

tuple<MatrixXf, MatrixXf> kalman_filter(MatrixXf x, MatrixXf P, MatrixXf u, MatrixXf F, MatrixXf H, MatrixXf R, MatrixXf I)
{
    for (int n = 0; n < sizeof(measurements) / sizeof(measurements[0]); n++) {

        // Measurement Update
        MatrixXf Z(1, 1);
        Z << measurements[n];

        MatrixXf y(1, 1);
        y << Z - (H * x);

        MatrixXf S(1, 1);
        S << H * P * H.transpose() + R;

        MatrixXf K(2, 1);
        K << P * H.transpose() * S.inverse();

        x << x + (K * y);

        P << (I - (K * H)) * P;

        // Prediction
        x << (F * x) + u;
        P << F * P * F.transpose();
    }

    return make_tuple(x, P);
}

void test_kalman_filter() {
    MatrixXf x(2, 1);// Initial state (location and velocity)
    x << 0,
            0;
    MatrixXf P(2, 2);//Initial Uncertainty
    P << 100, 0,
            0, 100;
    MatrixXf u(2, 1);// External Motion
    u << 0,
            0;
    MatrixXf F(2, 2);//Next State Function
    F << 1, 1,
            0, 1;
    MatrixXf H(1, 2);//Measurement Function
    H << 1,
            0;
    MatrixXf R(1, 1); //Measurement Uncertainty
    R << 1;
    MatrixXf I(2, 2);// Identity Matrix
    I << 1, 0,
            0, 1;

    tie(x, P) = kalman_filter(x, P, u, F, H, R, I);

    cout << "x= " << x << endl;
    cout << "P= " << P << endl;

    MatrixXf expected_x(2, 1);// Expected state (location and velocity)
    expected_x << 3.9966447353363037,
            0.99998360872268677;

    MatrixXf expected_P(2, 2);//Expected Uncertainty
    expected_P << 2.3190402984619141, 0.99175989627838134,
            0.9917597770690918, 0.49505758285522461;

    (x == expected_x) ? cout << "Passed" << endl : cout << "Failed" << endl;
    (P == expected_P) ? cout << "Passed" << endl : cout << "Failed" << endl;
}

int main ()
{
    test_kalman_filter();
    return 0;
}

