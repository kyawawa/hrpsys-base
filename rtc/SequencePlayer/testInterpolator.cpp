/* -*- coding:utf-8-unix; mode:c++; -*- */
#include <iostream>
#include <fstream>
#include "interpolator.h"

#define EPS (1e-6)

double target_pos(double t)
{
    // return sin(t);
    //return 1/(1+exp(-2*(t-4)));
    //return 1/(1+exp(-2*(t-4)))+0.01*sin(10*t);
    if (t + EPS < 2) return 4;
    else if (t + EPS < 4) return 3;
    else return 0;
}

int main (int argc, char** argv) {
    bool use_gnuplot = true;
    if (argc >= 2) {
        if ( std::string(argv[1])== "--use-gnuplot" ) {
            use_gnuplot = (std::string(argv[2])=="true");
        }
    }
    // file to write
    std::string fname("/tmp/tracking.dat");
    ofstream outputfile(fname.c_str());

    //make instance
    double m_dt = 0.002;
    interpolator* i = new interpolator(1, m_dt, interpolator::QUINTICSPLINE, 0.5);

    double start = 0.0;
    // double goal = 1.0, goalv = 0;
    double test_time = 6;
    double t = 0;
    double x = 0, v = 0, a = 0;
    double rt;

    //i->load("data.csv");
    // i->set(&start);

    for (int j = 0; j < test_time/m_dt; ++j) {
        // set goal
        double target, c_target;
        double interpolation_time;
        // double interpolation_gain = 10;
        // double Kp = 0.1;
        target = target_pos(t);
        // c_target = Kp * (target - x) + x;
        interpolation_time = 2; //interpolation_gain*fabs(x-target);
        //std::cout << fabs(x-target) << std::endl;
        // std::cout << interpolation_time << std::endl;
        // std::cerr << "t: " << t << ", remain_t: " << i->remain_time() << std::endl;
        if (i->remain_time() < EPS) {
            std::cerr << "set goal to " << target << std::endl;
            i->setGoal(&target, &v, interpolation_time, true);
        }
        // get interpolation val
        if (i->isEmpty()) std::cout << "empty!" << std::endl;
        i->get(&x, &v, &a,true);
        outputfile << t << " " << x << " " << v << " "<< a << " " << target <<std::endl;
        t += i->deltaT();
    }
    outputfile.close();

    size_t num_graphs = 1;
    if (use_gnuplot) {
        FILE* gp[num_graphs];
        std::string titles[1] = {"tracking"};
        for (size_t ii = 0; ii < num_graphs; ++ii) {
            gp[ii] = popen("gnuplot", "w");
            fprintf(gp[ii], "set title \"%s\"\n", titles[ii].c_str());
            fprintf(gp[ii], "set grid\n");
            fprintf(gp[ii], "plot \"%s\" using 1:%zu with lines title \"pos\"\n", fname.c_str(), ( ii * 3 + 2));
            fprintf(gp[ii], "replot \"%s\" using 1:%zu with lines title \"vel\"\n", fname.c_str(), ( ii * 3 + 3));
            fprintf(gp[ii], "replot \"%s\" using 1:%zu with lines title \"acc\"\n", fname.c_str(), ( ii * 3 + 4));
            fflush(gp[ii]);
        }
        double tmp;
        std::cin >> tmp;
        for (size_t j = 0; j < num_graphs; j++) pclose(gp[j]);
    }
    return 0;
};
