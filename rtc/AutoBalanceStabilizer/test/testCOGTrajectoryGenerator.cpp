// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  testCOGTrajectoryGenerator.cpp
 * @brief
 * @date  $Date$
 */

#include <cstdio>
#include <fstream>
#include "../COGTrajectoryGenerator.h"

namespace
{
constexpr double dt = 0.002;
constexpr double max_time = 8.0;
constexpr size_t max_count = static_cast<size_t>(std::round(max_time / dt));
const hrp::Vector3 init_cog(0, 0, 0.8);

std::vector<hrp::Vector3> cog_list;
std::vector<hrp::Vector3> cogvel_list;
std::vector<hrp::Vector3> cogacc_list;
std::vector<double> time_list;
}

void testPreviewController()
{
    constexpr double preview_time = 1.6;
    constexpr size_t preview_window = static_cast<size_t>(std::round(preview_time / dt));

    cog_list.resize(max_count);
    cogvel_list.resize(max_count);
    cogacc_list.resize(max_count);
    time_list.resize(max_count);

    std::deque<hrp::Vector3> ref_zmp_list;
    ref_zmp_list.emplace_back(0, 0, 0);

    hrp::COGTrajectoryGenerator cog_traj_gen(init_cog);
    cog_traj_gen.initPreviewController(dt, ref_zmp_list.front());

    for (size_t i = 0; i < max_count; ++i) {
        const double cur_time = i * dt;
        time_list.push_back(cur_time);

        while (ref_zmp_list.size() < preview_window) {
            double preview_time = (i + ref_zmp_list.size()) * dt;

            hrp::Vector3 zmp;
            if (preview_time < 1) {
                zmp << 0, 0, 0;
            } else if (preview_time < 3) {
                zmp << 0, 0.02, 0;
            } else if (preview_time < 5) {
                zmp << 0.02, -0.02, 0;
            } else if (preview_time < 7) {
                zmp << 0.04, 0.02, 0;
            } else {
                zmp << 0.04, 0, 0;
            }
            ref_zmp_list.push_back(zmp);
        }

        cog_traj_gen.calcCogFromZMP(ref_zmp_list);

        cog_list[i]    = cog_traj_gen.getCog();
        cogvel_list[i] = cog_traj_gen.getCogVel();
        cogacc_list[i] = cog_traj_gen.getCogAcc();
        time_list[i]   = cur_time;

        ref_zmp_list.pop_front();
    }
}

void testFootGuidedRunning()
{
    constexpr double _max_time = 4.0;
    constexpr size_t _max_count = static_cast<size_t>(std::round(_max_time / dt));
    cog_list.resize(_max_count);
    cogvel_list.resize(_max_count);
    cogacc_list.resize(_max_count);
    time_list.resize(_max_count);

    std::vector<hrp::Vector3> landing_points;
    std::vector<size_t> landing_counts;
    std::vector<size_t> supporting_counts;

    const hrp::Vector3 _init_cog = hrp::Vector3(0, 0.1, 0.8); // Start from left kicking
    hrp::Vector3 one_step(0.3, -0.2, 0.0);

    // 0.5[s] for one step, including supporting time 0.2[s]
    constexpr double step_time = 1.0;
    constexpr double support_time = 0.7;
    constexpr size_t step_count = static_cast<size_t>(std::round(step_time / dt));
    constexpr size_t support_count = static_cast<size_t>(std::round(support_time / dt));

    landing_points.emplace_back(_init_cog[0], _init_cog[1], 0);
    landing_counts.push_back(0);
    supporting_counts.push_back(support_count);

    hrp::COGTrajectoryGenerator cog_traj_gen(_init_cog);
    for (double count = step_count; count < _max_count * 2; count += step_count) {
        landing_points.push_back(landing_points.back());
        landing_points.back() += one_step;
        landing_counts.push_back(landing_counts.back());
        landing_counts.back() += step_count;
        supporting_counts.push_back(support_count);
        one_step[1] *= -1;

        std::cerr << "landing point: " << landing_points.back().transpose() << std::endl;
        std::cerr << "landing time:  " << landing_counts.back() * dt << std::endl;
    }

    size_t cur_idx = 0;
    for (size_t i = 0; i < _max_count; ++i) {
        const double cur_time = i * dt;

        if (i == landing_counts[cur_idx + 1]) {
            ++cur_idx;
            // _init_cog = cog_list[i - 1];
            std::cerr << "count: " << i << std::endl;
            // std::cerr << "init cog: " << _init_cog.transpose() << std::endl;
            std::cerr << "next landing: " << landing_points[cur_idx].transpose() << std::endl;
        }

        cog_traj_gen.calcCogFromLandingPoints(landing_points[cur_idx],
                                              landing_points[cur_idx + 1],
                                              // _init_cog,
                                              dt,
                                              landing_counts[cur_idx] * dt,
                                              supporting_counts[cur_idx] * dt,
                                              landing_counts[cur_idx + 1] * dt,
                                              i * dt);

        constexpr double g_acc = 9.80665;
        constexpr double omega = std::sqrt(g_acc / 0.8);
        // cog_list[i] = cog_traj_gen.getCog() + cog_traj_gen.getCogVel() / omega; // CP
        cog_list[i]    = cog_traj_gen.getCog();
        cogvel_list[i] = cog_traj_gen.getCogVel();
        cogacc_list[i] = cog_traj_gen.getCogAcc();
        time_list[i]   = cur_time;
    }
}

int main(int argc, char **argv)
{
    std::vector<std::string> arg_strs;
    for (int i = 1; i < argc; ++i) {
        arg_strs.push_back(std::string(argv[i]));
    }

    bool use_gnuplot = true;

    if (argc > 2) {
        if (std::string(argv[1]) == "--use-gnuplot") {
            use_gnuplot = (std::string(argv[2]) == "true");
        }
    }

    // testPreviewController(cog_list, time_list);
    testFootGuidedRunning();

    const std::string fname("/tmp/testCOGTrajectoryGenerator.dat");
    std::ofstream ofs(fname);

    for (size_t i = 0; i < cog_list.size(); ++i) {
        ofs << time_list[i] << " "
            << cog_list[i][0] << " " << cog_list[i][1] << " " << cog_list[i][2] << " "
            << cogvel_list[i][0] << " " << cogvel_list[i][1] << " " << cogvel_list[i][2] << " "
            << cogacc_list[i][0] << " " << cogacc_list[i][1] << " " << cogacc_list[i][2] << " "
            << std::endl;
    }

    ofs.close();

    if (use_gnuplot) {
        FILE* gp;
        gp = popen("gnuplot", "w");

        fprintf(gp, "set multiplot layout 2, 1\n");
        fprintf(gp, "set xlabel \"time [s]\"\n");
        fprintf(gp, "set ylabel \"COG [m]\"\n");
        fprintf(gp, "set title \"COG X\"\n");
        fprintf(gp, "plot \"%s\" using 1:2 with lines title \"Cog X\"\n", fname.c_str());
        fprintf(gp, "set title \"COG Y\"\n");
        fprintf(gp, "plot \"%s\" using 1:3 with lines title \"Cog Y\"\n", fname.c_str());
        fprintf(gp, "unset multiplot\n");
        fflush(gp);

        std::cerr << "Type some character to finish this test: " << std::flush;
        double tmp;
        std::cin >> tmp;
        pclose(gp);
    }

    return 0;
}
