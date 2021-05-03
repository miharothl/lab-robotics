#include <iostream>
#include <math.h>
#include <tuple>

using namespace std;

double gaussian_probability(double mu, double sigma2, double x)
{
    //Use mu, sigma2 (sigma squared), and x to code the 1-dimensional Gaussian
    //Put your code here

    // https://en.wikipedia.org/wiki/Normal_distribution
    double prob = 1.0 / sqrt(2.0 * M_PI * sigma2) * exp(-0.5 * pow((x - mu), 2.0) / sigma2);
    return prob;
}

tuple<double, double> measurement_update(double mean1, double var1, double mean2, double var2)
{
    double new_mean = (var2 * mean1 + var1 * mean2) / (var1 + var2) ;
    double new_var =  1 / (1 / var1 + 1 / var2);
    return make_tuple(new_mean, new_var);
}

tuple<double, double> state_prediction(double mean1, double var1, double mean2, double var2)
{
    double new_mean = mean1 + mean2;
    double new_var = var1 + var2;
    return make_tuple(new_mean, new_var);
}

void test_gaussian()
{
    double expected = 0.12098536225957168;
    double prob = gaussian_probability(10.0, 4.0, 8.0);
    prob == expected ? cout << "Passed" << endl : cout << "Failed" << endl;
}

void test_measurement_update_1()
{
    double new_mean;
    double new_var;
    tie(new_mean, new_var) = measurement_update(10, 8, 13, 2);

    double expected_mean = 12.4;
    double expected_var = 1.6;

    (new_mean == expected_mean) && (new_var == expected_var) ? cout << "Passed" << endl : cout << "Failed" << endl;
}

void test_measurement_update_2()
{
    double new_mean;
    double new_var;
    tie(new_mean, new_var) = measurement_update(20, 5, 30, 5);

    double expected_mean = 25.0;
    double expected_var = 2.5;

    (new_mean == expected_mean) && (new_var == expected_var) ? cout << "Passed" << endl : cout << "Failed" << endl;
}

void test_state_prediction()
{
    double new_mean;
    double new_var;
    tie(new_mean, new_var) = state_prediction(10, 4, 12, 4);

    double expected_new_mean = 22.;
    double expected_new_var = 8.;

    (new_mean == expected_new_mean) && (new_var == expected_new_var) ? cout << "Passed" << endl : cout << "Failed" << endl;
}

void test_kalman_filter()
{
    //Measurements and measurement variance
    double measurements[5] = { 5, 6, 7, 9, 10 };
    double measurement_sig = 4;

    //Motions and motion variance
    double motion[5] = { 1, 1, 2, 1, 1 };
    double motion_sig = 2;

    //Initial state
    double mu = 0;
    double sig = 1000;

    //######TODO: Put your code here below this line######//

    // Loop through all the measurments
    for (int i = 0; i < sizeof(measurements)/sizeof(measurements[0]); i++)
    {
        // Apply a measurment update
        tie(mu, sig) = measurement_update(mu, sig, measurements[i], measurement_sig);
        printf("update:  [%f, %f]\n", mu, sig);
        // Apply a state prediction
        tie(mu, sig) = state_prediction(mu, sig, motion[i], motion_sig);
        printf("predict: [%f, %f]\n", mu, sig);
    }

    double expected_new_mean = 10.99906346214631;
    double expected_new_var = 4.0058299481392163;

    (mu == expected_new_mean) && (sig == expected_new_var) ? cout << "Passed" << endl : cout << "Failed" << endl;
}

int main()
{
    test_gaussian();
    test_measurement_update_1();
    test_measurement_update_2();
    test_state_prediction();
    test_kalman_filter();
    return 0;
}