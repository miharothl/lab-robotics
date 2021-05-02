//#include "include/matplotlibcpp.h"
#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <stdexcept> // throw errors
#include <random> //C++ 11 Random Numbers

//namespace plt = matplotlibcpp;
using namespace std;

// Landmarks
double landmarks[8][2] = { { 20.0, 20.0 }, { 20.0, 80.0 }, { 20.0, 50.0 },
    { 50.0, 20.0 }, { 50.0, 80.0 }, { 80.0, 80.0 },
    { 80.0, 20.0 }, { 80.0, 50.0 } };

// Map size in meters
double world_size = 100.0;

// Random Generators
random_device rd;
mt19937 gen(rd());

// Global Functions
double mod(double first_term, double second_term);
double gen_real_random();

class Robot {
public:
    Robot()
    {
        // Constructor
        x = gen_real_random() * world_size; // robot's x coordinate
        y = gen_real_random() * world_size; // robot's y coordinate
        orient = gen_real_random() * 2.0 * M_PI; // robot's orientation

        forward_noise = 0.0; //noise of the forward movement
        turn_noise = 0.0; //noise of the turn
        sense_noise = 0.0; //noise of the sensing
    }

    void set(double new_x, double new_y, double new_orient)
    {
        // Set robot new position and orientation
        if (new_x < 0 || new_x >= world_size)
            throw std::invalid_argument("X coordinate out of bound");
        if (new_y < 0 || new_y >= world_size)
            throw std::invalid_argument("Y coordinate out of bound");
        if (new_orient < 0 || new_orient >= 2 * M_PI)
            throw std::invalid_argument("Orientation must be in [0..2pi]");

        x = new_x;
        y = new_y;
        orient = new_orient;
    }

    void set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise)
    {
        // Simulate noise, often useful in particle filters
        forward_noise = new_forward_noise;
        turn_noise = new_turn_noise;
        sense_noise = new_sense_noise;
    }

    vector<double> sense()
    {
        // Measure the distances from the robot toward the landmarks
        vector<double> z(sizeof(landmarks) / sizeof(landmarks[0]));
        double dist;

        for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++) {
            dist = sqrt(pow((x - landmarks[i][0]), 2) + pow((y - landmarks[i][1]), 2));
            dist += gen_gauss_random(0.0, sense_noise);
            z[i] = dist;
        }
        return z;
    }

    Robot move(double turn, double forward)
    {
        if (forward < 0)
            throw std::invalid_argument("Robot cannot move backward");

        // turn, and add randomness to the turning command
        orient = orient + turn + gen_gauss_random(0.0, turn_noise);
        orient = mod(orient, 2 * M_PI);

        // move, and add randomness to the motion command
        double dist = forward + gen_gauss_random(0.0, forward_noise);
        x = x + (cos(orient) * dist);
        y = y + (sin(orient) * dist);

        // cyclic truncate
        x = mod(x, world_size);
        y = mod(y, world_size);

        // set particle
        Robot res;
        res.set(x, y, orient);
        res.set_noise(forward_noise, turn_noise, sense_noise);

        return res;
    }

    string show_pose()
    {
        // Returns the robot current position and orientation in a string format
        return "[x=" + to_string(x) + " y=" + to_string(y) + " orient=" + to_string(orient) + "]";
    }

    string read_sensors()
    {
        // Returns all the distances from the robot toward the landmarks
        vector<double> z = sense();
        string readings = "[";
        for (int i = 0; i < z.size(); i++) {
            readings += to_string(z[i]) + " ";
        }
        readings[readings.size() - 1] = ']';

        return readings;
    }

    double measurement_prob(vector<double> measurement)
    {
        // Calculates how likely a measurement should be
        double prob = 1.0;
        double dist;

        for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++) {
            dist = sqrt(pow((x - landmarks[i][0]), 2) + pow((y - landmarks[i][1]), 2));
            prob *= gaussian(dist, sense_noise, measurement[i]);
        }

        return prob;
    }

    double x, y, orient; //robot poses
    double forward_noise, turn_noise, sense_noise; //robot noises

private:
    double gen_gauss_random(double mean, double variance)
    {
        // Gaussian random
        normal_distribution<double> gauss_dist(mean, variance);
        return gauss_dist(gen);
    }

    double gaussian(double mu, double sigma, double x)
    {
        // Probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(-(pow((mu - x), 2)) / (pow(sigma, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(sigma, 2)));
    }
};

// Functions
double gen_real_random()
{
    // Generate real random between 0 and 1
    uniform_real_distribution<double> real_dist(0.0, 1.0); //Real
    return real_dist(gen);
}

double mod(double first_term, double second_term)
{
    // Compute the modulus
    return first_term - (second_term)*floor(first_term / (second_term));
}

double evaluation(Robot r, Robot p[], int n)
{
    //Calculate the mean error of the system
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        //the second part is because of world's cyclicity
        double dx = mod((p[i].x - r.x + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double dy = mod((p[i].y - r.y + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double err = sqrt(pow(dx, 2) + pow(dy, 2));
        sum += err;
    }
    return sum / n;
}
double max(double arr[], int n)
{
    // Identify the max element in an array
    double max = 0;
    for (int i = 0; i < n; i++) {
        if (arr[i] > max)
            max = arr[i];
    }
    return max;
}

//void visualization(int n, Robot robot, int step, Robot p[], Robot pr[])
//{
//	//Draw the robot, landmarks, particles and resampled particles on a graph
//
//    //Graph Format
//    plt::title("MCL, step " + to_string(step));
//    plt::xlim(0, 100);
//    plt::ylim(0, 100);
//
//    //Draw particles in green
//    for (int i = 0; i < n; i++) {
//        plt::plot({ p[i].x }, { p[i].y }, "go");
//    }
//
//    //Draw resampled particles in yellow
//    for (int i = 0; i < n; i++) {
//        plt::plot({ pr[i].x }, { pr[i].y }, "yo");
//    }
//
//    //Draw landmarks in red
//    for (int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++) {
//        plt::plot({ landmarks[i][0] }, { landmarks[i][1] }, "ro");
//    }
//
//    //Draw robot position in blue
//    plt::plot({ robot.x }, { robot.y }, "bo");
//
//	//Save the image and close the plot
//    plt::save("./Images/Step" + to_string(step) + ".png");
//    plt::clf();
//}


//####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####

void step_01_first_interaction()
{
    cout << "Step 01" << endl;

    ///////////////////////////////////////////////////////////////////////////////
    // Instantiating a robot object from the Robot class
    Robot myrobot;

    // TODO: Set robot new position to x=10.0, y=10.0 and orientation=0
    // Fill in the position and orientation values in myrobot.set() function
    double x = 10.0;
    double y = 10.0;
    double orientation = 0.;
    myrobot.set(x, y, orientation);

    // Printing out the new robot position and orientation
    cout << myrobot.show_pose() << endl;

    // TODO: Rotate the robot by PI/2.0 and then move him forward by 10.0
    // Use M_PI for the pi value
    double rotate = M_PI / 2.0;
    double forward = 10.0;

    myrobot.move(rotate, forward);

    // TODO: Print out the new robot position and orientation
    cout << myrobot.show_pose() << endl;

    // Printing the distance from the robot toward the eight landmarks
    cout << myrobot.read_sensors() << endl;
}

void step_02_motion_and_sensing()
{
    cout << "Step 02" << endl;

    ///////////////////////////////////////////////////////////////////////////////
    // TODO: Instantiate a robot object from the Robot class
    Robot robot;

    // TODO: Set robot new position to x=30.0, y=50.0 and orientation=PI/2
    double x = 30.0;
    double y = 50.0;
    double orientation = M_PI / 2;

    robot.set(x, y, orientation);
    cout << robot.show_pose() << endl;

    // TODO: Turn clockwise by PI/2 and move by 15 meters
    double rotate = -M_PI / 2.0;
    double forward = 15.0;

    robot.move(rotate, forward);
    cout << robot.show_pose() << endl;

    // TODO: Print the distance from the robot toward the eight landmarks
    cout << robot.read_sensors() << endl;

    // TODO: Turn clockwise by PI/2 and move by 10 meters
    rotate = -M_PI / 2.0;
    forward = 10.0;
    robot.move (rotate, forward);
    cout << robot.show_pose() << endl;

    // TODO: Print the distance from the robot toward the eight landmarks
    cout << robot.read_sensors() << endl;
}

void step_03_noise()
{
    cout << "Step 03" << endl;

    Robot robot;
    ///////////////////////////////////////////////////////////////////////////////
    // TODO: Simulate Noise
    // Forward Noise=5.0, Turn Noise=0.1,Sense Noise=5.0
    double forward_noise = 5.0;
    double turn_noise = 0.1;
    double sense_noise = 5.0;

    robot.set_noise(forward_noise, turn_noise, sense_noise);

    robot.set(30.0, 50.0, M_PI / 2.0);
    cout << robot.show_pose() << endl;
    robot.move(-M_PI / 2.0, 15.0);
    cout << robot.show_pose() << endl;
    cout << robot.read_sensors() << endl;

    robot.move(-M_PI / 2.0, 10.0);
    cout << robot.show_pose() << endl;
    cout << robot.read_sensors() << endl;
}

void step_04_particle_filter_generate_particles()
{
    cout << "Step 04" << endl;

    //Practice Interfacing with Robot Class
    Robot robot;
    robot.set_noise(5.0, 0.1, 5.0);
    robot.set(30.0, 50.0, M_PI / 2.0);
    robot.move(-M_PI / 2.0, 15.0);
    cout << robot.read_sensors() << endl;
    robot.move(-M_PI / 2.0, 10.0);
    cout << robot.read_sensors() << endl;

    ///////////////////////////////////////////////////////////////////////////////
    // Instantiating 1000 Particles each with a random position and orientation
    int n = 1000;
    Robot p[n];

    double forward_noise = 0.05;
    double turn_noise = 0.05;
    double sense_noise = 5.0;

    //TODO: Your job is to loop over the set of particles
    for (int i=0; i<n; i++)
    {
        //TODO: For each particle add noise: Forward_Noise=0.05, Turn_Noise=0.05, and Sense_Noise=5.0
        p[i].set_noise(forward_noise, turn_noise, sense_noise);

        //TODO: And print its pose on a single line
        cout << p[i].show_pose() << endl;
    }
}

void step_05_particle_filter_simulate_motion()
{
    cout << "Step 05" << endl;

    //Practice Interfacing with Robot Class
    Robot robot;
    robot.set_noise(5.0, 0.1, 5.0);
    robot.set(30.0, 50.0, M_PI / 2.0);
    robot.move(-M_PI / 2.0, 15.0);
    cout << robot.read_sensors() << endl;
    robot.move(-M_PI / 2.0, 10.0);
    cout << robot.read_sensors() << endl;

    // Instantiating 1000 Particles each with a random position and orientation
    int n = 1000;
    Robot p[n];

    double forward_noise = 0.05;
    double turn_noise = 0.05;
    double sense_noise = 5.0;

    for (int i=0; i<n; i++)
    {
        p[i].set_noise(forward_noise, turn_noise, sense_noise);
        //cout << p[i].show_pose() << endl;
    }

    ///////////////////////////////////////////////////////////////////////////////
    //TODO: Create a new particle set 'p2'
    Robot p2[n];

    double rotate = 0.1;
    double forward = 5.0;

    for (int i=0; i<n; i++)
    {
        //TODO: Rotate each particle by 0.1 and move it forward by 5.0
        p2[i] =  p[i].move(rotate, forward);

        //TODO: Assign 'p2' to 'p' and print the particle poses, each on a single line
        p[i] = p2[i];
        cout << p[i].show_pose() << endl;
    }
}

void step_06_importance_weight()
{
    cout << "Step 06" << endl;

    //Practice Interfacing with Robot Class
    Robot robot;
    robot.set_noise(5.0, 0.1, 5.0);
    robot.set(30.0, 50.0, M_PI / 2.0);
    robot.move(-M_PI / 2.0, 15.0);
    cout << robot.read_sensors() << endl;
    robot.move(-M_PI / 2.0, 10.0);
    cout << robot.read_sensors() << endl;

    // Instantiating 1000 Particles each with a random position and orientation
    int n = 1000;
    Robot p[n];

    double forward_noise = 0.05;
    double turn_noise = 0.05;
    double sense_noise = 5.0;

    for (int i=0; i<n; i++)
    {
        p[i].set_noise(forward_noise, turn_noise, sense_noise);
        //cout << p[i].show_pose() << endl;
    }

    Robot p2[n];

    double rotate = 0.1;
    double forward = 5.0;

    for (int i=0; i<n; i++)
    {
        p2[i] =  p[i].move(rotate, forward);

        p[i] = p2[i];
        cout << p[i].show_pose() << endl;
    }

    ///////////////////////////////////////////////////////////////////////////////
    //Re-initialize myrobot object and Initialize a measurment vector
    robot = Robot();
    vector<double> z;

    //Move the robot and sense the environment afterwards
    robot = robot.move(0.1, 5.0);
    z = robot.sense();

    //TODO: Generate particle weights depending on robot's measurement
    //TODO: Print particle weights, each on a single line
    double w[n];

    for (int i=0; i<n; i++)
    {
        w[i] = p[i].measurement_prob(z);
        cout << w[i] << endl;
    }
}

void compute_prob(double w[], int n)
{
    double sum = 0.;

    for (int i = 0; i < n; i++) {
        sum = sum + w[i];
    }
    for (int j = 0; j < n; j++) {
        w[j] = w[j] / sum;
        cout << "P" << j + 1 << "=" << w[j] << endl;
    }
}

void step_07_resampling()
{
    cout << "Step 07" << endl;

    double w[] = { 0.6, 1.2, 2.4, 0.6, 1.2 };
    int n = sizeof(w) / sizeof(w[0]);

    compute_prob(w, n);
}

void step_08_resampling_wheel()
{
    cout << "Step 08" << endl;

    //Practice Interfacing with Robot Class
    Robot robot;
    robot.set_noise(5.0, 0.1, 5.0);
    robot.set(30.0, 50.0, M_PI / 2.0);
    robot.move(-M_PI / 2.0, 15.0);
    cout << robot.read_sensors() << endl;
    robot.move(-M_PI / 2.0, 10.0);
    cout << robot.read_sensors() << endl;

    // Instantiating 1000 Particles each with a random position and orientation
    int n = 1000;
    Robot p[n];

    double forward_noise = 0.05;
    double turn_noise = 0.05;
    double sense_noise = 5.0;

    for (int i=0; i<n; i++)
    {
        p[i].set_noise(forward_noise, turn_noise, sense_noise);
        //cout << p[i].show_pose() << endl;
    }

    Robot p2[n];

    double rotate = 0.1;
    double forward = 5.0;

    for (int i=0; i<n; i++)
    {
        p2[i] =  p[i].move(rotate, forward);

        p[i] = p2[i];
        cout << p[i].show_pose() << endl;
    }

    //Re-initialize myrobot object and Initialize a measurment vector
    robot = Robot();
    vector<double> z;

    //Move the robot and sense the environment afterwards
    robot = robot.move(0.1, 5.0);
    z = robot.sense();

    double w[n];

    for (int i=0; i<n; i++)
    {
        w[i] = p[i].measurement_prob(z);
        cout << w[i] << endl;
    }

    ///////////////////////////////////////////////////////////////////////////////
    //Resample the particles with a sample probability proportional to the importance weight
    //TODO: Resample the particles with a sample probability proportional to the importance weight
    Robot p3[n];
    int index = gen_real_random() * n;
    //cout << index << endl;
    double beta = 0.0;
    double mw = max(w, n);
    //cout << mw;
    for (int i = 0; i < n; i++) {
        beta += gen_real_random() * 2.0 * mw;
        while (beta > w[index]) {
            beta -= w[index];
            index = mod((index + 1), n);
        }
        p3[i] = p[index];
    }
    for (int k=0; k < n; k++) {
        p[k] = p3[k];
        cout << p[k].show_pose() << endl;
    }
}

void step_09_error()
{
    cout << "Step 09" << endl;

    //Practice Interfacing with Robot Class
    Robot robot;
    robot.set_noise(5.0, 0.1, 5.0);
    robot.set(30.0, 50.0, M_PI / 2.0);
    robot.move(-M_PI / 2.0, 15.0);
    cout << robot.read_sensors() << endl;
    robot.move(-M_PI / 2.0, 10.0);
    cout << robot.read_sensors() << endl;

    // Instantiating 1000 Particles each with a random position and orientation
    int n = 1000;
    Robot p[n];

    double forward_noise = 0.05;
    double turn_noise = 0.05;
    double sense_noise = 5.0;

    for (int i=0; i<n; i++)
    {
        p[i].set_noise(forward_noise, turn_noise, sense_noise);
        //cout << p[i].show_pose() << endl;
    }

    //Re-initialize myrobot object and Initialize a measurment vector
    robot = Robot();
    vector<double> z;

    for (int t=0; t<50; t++)
    {
        Robot p2[n];

        double rotate = 0.1;
        double forward = 5.0;

        for (int i=0; i<n; i++)
        {
            p2[i] =  p[i].move(rotate, forward);

            p[i] = p2[i];
            //cout << p[i].show_pose() << endl;
        }

    // move so i can step
    //    //Re-initialize myrobot object and Initialize a measurment vector
    //    robot = Robot();
    //    vector<double> z;

        //Move the robot and sense the environment afterwards
        robot = robot.move(0.1, 5.0);
        z = robot.sense();

        double w[n];

        for (int i=0; i<n; i++)
        {
            w[i] = p[i].measurement_prob(z);
            //cout << w[i] << endl;
        }

        //Resample the particles with a sample probability proportional to the importance weight
        Robot p3[n];
        int index = gen_real_random() * n;
        //cout << index << endl;
        double beta = 0.0;
        double mw = max(w, n);
        //cout << mw;
        for (int i = 0; i < n; i++) {
            beta += gen_real_random() * 2.0 * mw;
            while (beta > w[index]) {
                beta -= w[index];
                index = mod((index + 1), n);
            }
            p3[i] = p[index];
        }
        for (int k=0; k < n; k++) {
            p[k] = p3[k];
            //cout << p[k].show_pose() << endl;
        }

        ///////////////////////////////////////////////////////////////////////////////////
        // TODO: Evaluate the error by priting it in this form:
        double error_value = evaluation(robot, p, n);

        cout << "Step = " << t << ", Evaluation = " << error_value << endl;
    }
}

int main()
{
    cout << "I am ready for coding the MCL!" << endl;

    step_01_first_interaction();
    step_02_motion_and_sensing();
    step_03_noise();
    step_04_particle_filter_generate_particles();
    step_05_particle_filter_simulate_motion();
    step_06_importance_weight();
    step_07_resampling();
    step_08_resampling_wheel();
    step_09_error();

    return 0;
}