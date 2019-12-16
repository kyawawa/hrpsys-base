// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  COGTrajectoryGenerator.h
 * @brief
 * @date  $Date$
 */

#include "COGTrajectoryGenerator.h"

namespace hrp {

void COGTrajectoryGenerator::initPreviewController(const double dt, const hrp::Vector3& cur_ref_zmp)
{
    std::cerr << "cog z: " << cog(2) << " ref cog z: " << cog(2) - cur_ref_zmp(2) << std::endl;
    preview_controller.reset(new ExtendedPreviewController(dt, cog(2) - cur_ref_zmp(2), cur_ref_zmp));
}

void COGTrajectoryGenerator::calcCogFromZMP(const std::deque<hrp::Vector3>& refzmp_list)
{
    if (calculation_type == PREVIEW_CONTROL) {
        preview_controller->calc_x_k(refzmp_list);
        cog     = preview_controller->getRefCog();
        cog_vel = preview_controller->getRefCogVel();
        cog_acc = preview_controller->getRefCogAcc();
    }
}

void COGTrajectoryGenerator::calcCogFromLandingPoints(const hrp::Vector3& support_point,
                                                      const hrp::Vector3& landing_point,
                                                      const double dt,
                                                      const double start_time,
                                                      const double supporting_time,
                                                      const double landing_time,
                                                      const double cur_time)
{
    const double g_acc = 9.80665;
    const double height = 0.05;
    const double take_off_z_vel = std::sqrt(2 * g_acc * height);
    double flight_time = 2 * take_off_z_vel / g_acc;

    const double cog_height = 0.8;
    const double omega = std::sqrt(g_acc / cog_height);
    const double rel_cur_time = cur_time - start_time;
    const double rel_land_time = landing_time - start_time;
    const double omega_tau = omega * supporting_time;

    // TODO: 本当はx,yのみ
    const hrp::Vector3 cp = cog + cog_vel / omega;
    double omega_T = omega * flight_time;
    hrp::Vector3 c_1 = hrp::Vector3::Zero();
    hrp::Vector3 ref_zmp = hrp::Vector3::Zero();

    // hrp::Vector3 c_1 = cp - support_point + (landing_point - support_point) / (1 + omega_T) * std::exp(-omega * (supporting_time - rel_cur_time));
    // c_1 /= -omega * (1 / flight_time + omega / 2) * std::exp((2 * rel_land_time - rel_cur_time) * omega) +
    //     0.5 * omega * omega * omega * (1 / omega + 2 * rel_cur_time) * std::exp(omega * rel_cur_time) +
    //     (omega * (1 / flight_time + omega / 2) * (1 - omega_T)) / (1 + omega_T) * std::exp((2 * rel_land_time - 2 * supporting_time + rel_cur_time) * omega) -
    //     0.5 * omega * omega * omega * (1 / omega + 2 * supporting_time + 3 * flight_time + 2 * omega_T * supporting_time) * std::exp(omega_T);

    // ddot(x) の条件でといた
    // hrp::Vector3 c_1 = 2 * (cp - support_point - (landing_point - support_point) / (omega_T + 1) * std::exp(-omega * (supporting_time - rel_cur_time)));
    // c_1 /= omega * omega * (1 + 2 * omega * rel_cur_time) * std::exp(omega * rel_cur_time) +
    //     -(omega_T + 2) / flight_time * omega * std::exp(omega * (2 * rel_land_time - rel_cur_time)) +
    //     -omega * omega / (omega_T + 1) * (1 + 2 * omega * supporting_time + 2 * omega * flight_time + 2 * omega * supporting_time * flight_time) * std::exp(omega * rel_cur_time) +
    //     (omega_T + 2) / (flight_time * (omega_T + 1)) * omega * (1 - omega_T) * std::exp(omega * (2 * rel_land_time - 2 * supporting_time + rel_cur_time));

    // これは CP の1階微分方程式を解いた結果
    // hrp::Vector3 c_1 = cp - support_point - (landing_point - support_point) / (omega_T + 1) * std::exp(-omega * (supporting_time - rel_cur_time));
    // c_1 /= ((flight_time + supporting_time + omega * supporting_time * flight_time) / (omega_T + 1) + rel_cur_time) * omega * omega * omega * std::exp(omega * rel_cur_time) +
    //     (omega + flight_time / 2 * omega * omega - 1 / flight_time - 0.5 * omega) / (omega_T + 1) * omega * std::exp(omega * (2 * rel_land_time - 2 * supporting_time + rel_cur_time)) +
    //     omega * omega * omega * rel_cur_time * std::exp(omega * rel_cur_time) +
    //     -(omega / flight_time + 0.5 * omega * omega) * std::exp(omega * (2 * rel_land_time - rel_cur_time));

    // 3区間に分けて考える
    // std::vector<double> tau(3);
    // double diff_tau = supporting_time / 5.0;
    // diff_tau = diff_tau - std::fmod(diff_tau, dt);
    // tau[0] = diff_tau;
    // tau[1] = tau[0] + diff_tau * 3;
    // tau[2] = supporting_time;

    // hrp::Vector3 start_cp = support_point;


    const double y_offset = support_point[1] > 0 ? -0.05 : 0.05;
    const hrp::Vector3 start_zmp_offset = hrp::Vector3(-0.1, y_offset, 0);
    const hrp::Vector3 end_zmp_offset = hrp::Vector3(0.1, y_offset, 0);
    // const hrp::Vector3 start_zmp_offset = hrp::Vector3(-0.05, 0, 0);
    // const hrp::Vector3 end_zmp_offset = hrp::Vector3(0.05, 0, 0);

    // std::vector<hrp::Vector3> an(3);
    // std::vector<hrp::Vector3> bn(3);
    // // TODO: input_zmpの接続の確認、cog_velの接続も
    // an.back() = (end_zmp_offset - start_zmp_offset) * 2 / 5.0 / diff_tau;
    // // an.back() = hrp::Vector3::Zero();
    // // bn.back() = support_point;
    // bn.back() = (support_point + end_zmp_offset) - an.back() * supporting_time;

    // an[0] = an.back();
    // // bn[0] = support_point;
    // bn[0] = support_point + start_zmp_offset;

    // // an[1] = an.back();
    // an[1] = (an.back() * tau[1] - an[0] * tau[0] + bn.back() - bn[0]) / (tau[1] - tau[0]);
    // bn[1] = bn[0] + an[0] * tau[0] - an[1] * tau[0];

    // {
    //     Eigen::Matrix<double, 12, 12> A;
    //     A <<
    //         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, exp(2 * omega * tau[2]), 1,
    //         0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
    //         0, 0, 0, 0, 0, 0, exp(omega * tau[0]), exp(-omega * tau[0]), -exp(omega * tau[0]), -exp(-omega * tau[0]), 0, 0,
    //         0, 0, 0, 0, 0, 0, 0, 0, -exp(omega * tau[1]), -exp(-omega * tau[1]), exp(omega * tau[1]), exp(-omega * tau[1]),
    //         1, 0, -1, 0, 0, 0, (exp(omega * tau[0]) + exp(-omega * tau[0])) * omega, 0, -omega * exp(omega * tau[0]), omega * exp(-omega * tau[0]), 0, 0,
    //         0, 0, -1, 0, 1, 0, 0, 0, -omega * exp(omega * tau[1]), omega * exp(-omega * tau[1]), (exp(omega * tau[1]) + exp(omega * (2 * tau[2] - tau[1]))) * omega, 0,
    //         1 / omega, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0,
    //         0, 0, 0, 0, flight_time + 1 / omega, 0, 0, 0, 0, 0, 2 * omega * (flight_time + 1 / omega) * exp(omega * tau[2]), 0,
    //         0, 0, 0, 0, tau[2], 1, 0, 0, 0, 0, 0, 0,
    //         tau[0], 1, -tau[0], -1, 0, 0, 0, 0, 0, 0, 0, 0,
    //         0, 0, tau[1], 1, -tau[1], -1, 0, 0, 0, 0, 0, 0,
    //         0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    //     const auto A_lu = A.partialPivLu();
    //     for (size_t i = 0; i < 3; ++i) {
    //         Eigen::Matrix<double, 12, 1> B;
    //         Eigen::Matrix<double, 12, 1> ans;
    //         B << 0, 0, 0, 0, 0, 0, support_point[i], (landing_point - (support_point + end_zmp_offset))[i], (support_point + end_zmp_offset)[i], 0, 0, (support_point + start_zmp_offset)[i];
    //         ans = A_lu.solve(B);

    //         an[0][i] = ans[0];
    //         an[1][i] = ans[2];
    //         an[2][i] = ans[4];
    //         bn[0][i] = ans[1];
    //         bn[1][i] = ans[3];
    //         bn[2][i] = ans[5];
    //     }
    // }

    // const auto calcC1FromStartCp = [&](const double a, const double b, const hrp::Vector3& start_cp) {
    //     return (cp - (b + (1 / omega + rel_cur_time) * a + (start_cp - b - (1 / omega + tau) * a) * std::exp(omega * (rel_cur_time - tau)))) /
    //     (omega * omega * omega * rel_cur_time * std::exp(omega * rel_cur_time) +
    //      -(omega * omega * flight_time + 2 * omega) / (2 * flight_time) * std::exp(omega * (2 * rel_land_time - rel_cur_time)) +
    //      -omega * omega * omega * tau * std::exp(omega * rel_cur_time) +
    //      (omega * omega * flight_time + 2 * omega) / (2 * flight_time) * std::exp(omega * (2 * rel_land_time + rel_cur_time - 2 * tau)));
    // }

    // 3区間線形
    // const auto calcC1OfCurrentPhase = [&](const size_t idx) {
    //     hrp::Vector3 Dn = hrp::Vector3::Zero();
    //     for (size_t i = idx; i < an.size() - 1; ++i) {
    //         Dn += (an[idx + 1] - an[idx]) * std::exp(-omega * tau[idx]);
    //     }
    //     Dn /= omega;

    //     return (cp - (bn[idx] + (rel_cur_time + 1 / omega) * an[idx] +
    //                   Dn * std::exp(omega * rel_cur_time) +
    //                   (landing_point - (flight_time + 1 / omega - supporting_time) * an.back() - bn.back()) / (omega_T + 1) * std::exp(-omega * (supporting_time - rel_cur_time)))) /
    //     (omega * omega * omega * rel_cur_time * std::exp(omega * rel_cur_time) +
    //      -(omega_T + 2) / (2 * flight_time) * omega * std::exp(omega * (2 * rel_land_time - rel_cur_time)) +
    //      -(omega * flight_time * supporting_time + supporting_time + flight_time) / (omega_T + 1) * omega * omega * omega * std::exp(omega * rel_cur_time) +
    //      -(omega * omega * flight_time * flight_time + omega_T - 2) / (2 * flight_time * (omega_T + 1)) * omega * std::exp(omega * (2 * rel_land_time - 2 * supporting_time + rel_cur_time)));
    // };

    // size_t idx = 0;
    // while (rel_cur_time > tau[idx] && idx < tau.size() - 1) ++idx;
    // c_1 = calcC1OfCurrentPhase(idx);
    // ref_zmp = an[idx] * rel_cur_time + bn[idx];

    // 3次関数 + T
    // {
    //     const double tau = supporting_time;
    //     std::vector<double> T(3);

    //     for (size_t i = 0; i < 3; ++i) {
    //         const double X0 = support_point[i];
    //         const double X1 = landing_point[i];
    //         const double xz0 = X0 + start_zmp_offset[i];
    //         const double xz1 = X0 + end_zmp_offset[i];
    //         // const double next_xz0 = X1 + start_zmp_offset[i];
    //         const double next_xz0 = X1 + (i == 1 ? -start_zmp_offset[i] : start_zmp_offset[i]);

    //         const double tau2 = tau * tau;
    //         const double tau3 = tau2 * tau;
    //         const double omega2 = omega * omega;

    //         a_var[i] = (X0*tau*omega*exp(tau*omega) + X0*tau*omega - 2*X0*exp(tau*omega) + 2*X0 + X1*tau*omega*exp(tau*omega) + X1*tau*omega + 2*X1*exp(tau*omega) - 2*X1 - tau*omega*xz0*exp(tau*omega) - tau*omega*xz0 - tau*omega*next_xz0*exp(tau*omega) - tau*omega*next_xz0 + 4*xz0*exp(tau*omega) - 2*xz1*exp(tau*omega) - 2*xz1 - 2*next_xz0*exp(tau*omega) + 2*next_xz0) / (tau3*(exp(tau*omega) + 1));

    //         b_var[i] = -(2*X0*tau2*omega2*exp(2*tau*omega) - 2*X0*tau2*omega2 - 6*X0*tau*omega*exp(2*tau*omega) - 6*X0*tau*omega + 6*X0*exp(2*tau*omega) - 6*X0 + X1*tau2*omega2*exp(2*tau*omega) - X1*tau2*omega2 - 12*X1*tau*omega*exp(tau*omega) - 6*X1*exp(2*tau*omega) + 6*X1 - 2*tau2*omega2*xz0*exp(2*tau*omega) + 2*tau2*omega2*xz0 - tau2*omega2*next_xz0*exp(2*tau*omega) + tau2*omega2*next_xz0 + 9*tau*omega*xz0*exp(2*tau*omega) + 3*tau*omega*xz0 - 3*tau*omega*xz1*exp(2*tau*omega) + 3*tau*omega*xz1 + 12*tau*omega*next_xz0*exp(tau*omega) - 12*xz0*exp(2*tau*omega) - 12*xz0*exp(tau*omega) + 6*xz1*exp(2*tau*omega) + 12*xz1*exp(tau*omega) + 6*xz1 + 6*next_xz0*exp(2*tau*omega) - 6*next_xz0) / (tau3*omega*(exp(2*tau*omega) - 1));

    //         c_var[i] = (X0*tau2*omega2*exp(2*tau*omega) - X0*tau2*omega2 - 4*X0*tau*omega*exp(2*tau*omega) - 4*X0*tau*omega*exp(tau*omega) - 4*X0*tau*omega + 6*X0*exp(2*tau*omega) - 6*X0 - 2*X1*tau*omega*exp(2*tau*omega) - 8*X1*tau*omega*exp(tau*omega) - 2*X1*tau*omega - 6*X1*exp(2*tau*omega) + 6*X1 - tau2*omega2*xz0*exp(2*tau*omega) + tau2*omega2*xz0 + 4*tau*omega*xz0*exp(2*tau*omega) + 4*tau*omega*xz0*exp(tau*omega) + 4*tau*omega*xz0 + 2*tau*omega*next_xz0*exp(2*tau*omega) + 8*tau*omega*next_xz0*exp(tau*omega) + 2*tau*omega*next_xz0 - 12*xz0*exp(2*tau*omega) - 12*xz0*exp(tau*omega) + 6*xz1*exp(2*tau*omega) + 12*xz1*exp(tau*omega) + 6*xz1 + 6*next_xz0*exp(2*tau*omega) - 6*next_xz0) / (tau2*omega*(exp(2*tau*omega) - 1));

    //         d_var[i] = xz0;

    //         // (-2*X0*tau2*omega2*exp(tau*omega) - 4*X0*tau2*omega2 - 12*X0*tau*omega + 12*X0*exp(tau*omega) - 12*X0 - 4*X1*tau2*omega2*exp(tau*omega) - 2*X1*tau2*omega2 - 12*X1*tau*omega*exp(tau*omega) - 12*X1*exp(tau*omega) + 12*X1 + 2*tau2*omega2*xz0*exp(tau*omega) + 4*tau2*omega2*xz0 + 4*tau2*omega2*next_xz0*exp(tau*omega) + 2*tau2*omega2*next_xz0 - 6*tau*omega*xz0*exp(tau*omega) + 6*tau*omega*xz0 + 6*tau*omega*xz1*exp(tau*omega) + 6*tau*omega*xz1 + 12*tau*omega*next_xz0*exp(tau*omega) - 24*xz0*exp(tau*omega) + 12*xz1*exp(tau*omega) + 12*xz1 + 12*next_xz0*exp(tau*omega) - 12*next_xz0)/(tau3*omega**3*(exp(2*tau*omega) - 1));

    //         // (2*X0*tau2*omega2*exp(tau*omega) + X0*tau2*omega2 - 6*X0*tau*omega*exp(tau*omega) + 6*X0*exp(tau*omega) - 6*X0 + X1*tau2*omega2*exp(tau*omega) + 2*X1*tau2*omega2 - 6*X1*tau*omega - 6*X1*exp(tau*omega) + 6*X1 - 2*tau2*omega2*xz0*exp(tau*omega) - tau2*omega2*xz0 - tau2*omega2*next_xz0*exp(tau*omega) - 2*tau2*omega2*next_xz0 + 9*tau*omega*xz0*exp(tau*omega) + 3*tau*omega*xz0 - 3*tau*omega*xz1*exp(tau*omega) - 3*tau*omega*xz1 + 6*tau*omega*next_xz0 - 12*xz0*exp(tau*omega) + 6*xz1*exp(tau*omega) + 6*xz1 + 6*next_xz0*exp(tau*omega) - 6*next_xz0)/(tau3*omega**3*staud::sinh(tau*omega));

    //         T[i] = (-xz1 + next_xz0) / (omega * (X1 - next_xz0));
    //         std::cerr << "T " << i << " " << T[i] << std::endl;
    //     }

    //     flight_time = std::max(flight_time, std::max(T[0], T[1]));
    //     omega_T = omega * flight_time;
    //     // std::cerr << "flight_time: " << flight_time << std::endl;
    // }

    // 3次関数
    {
        const double tau = supporting_time;
        const double tau2 = tau * tau;
        const double tau3 = tau2 * tau;
        const double omega2 = omega * omega;
        const double omega3 = omega2 * omega;

        hrp::Vector3 a_var;
        hrp::Vector3 b_var;
        hrp::Vector3 c_var;
        hrp::Vector3 d_var;

        {
            Eigen::Matrix<double, 6, 6> A;
            A <<
                0, 0, 0, 1, 0, 0,
                tau3, tau2, tau, 1, 0, 0,
                6 / omega3, 2 / omega2, 1 / omega, 1, 0, 2,
                tau3 + (flight_time + 1 / omega) * (3 * tau2 + 6 / omega2), tau2 + 2 * (flight_time + 1 / omega) * tau, tau + flight_time + 1 / omega, 1, (omega_T + 1) * exp(omega * tau), -(omega_T + 1) * exp(-omega * tau),
                0, 2 / omega2, 0, 0, 1, 1,
                6 * tau / omega2, 2 / omega2, 0, 0, exp(omega * tau), exp(-omega * tau);

            const auto A_lu = A.partialPivLu();
            for (size_t i = 0; i < 3; ++i) {
                Eigen::Matrix<double, 6, 1> B;
                Eigen::Matrix<double, 6, 1> ans;
                B << (support_point + start_zmp_offset)[i], (support_point + end_zmp_offset)[i], support_point[i], landing_point[i], 0, 0;
                ans = A_lu.solve(B);

                a_var[i] = ans[0];
                b_var[i] = ans[1];
                c_var[i] = ans[2];
                d_var[i] = ans[3];
            }
        }

        const hrp::Vector3 A = d_var + (rel_cur_time + 1 / omega) * c_var + (rel_cur_time * rel_cur_time + 2 * rel_cur_time / omega + 2 / omega2) * b_var + (rel_cur_time + rel_cur_time + rel_cur_time + 3 * rel_cur_time * rel_cur_time / omega + 6 * rel_cur_time / omega2 + 6 / omega3) * a_var;
        const hrp::Vector3 B_tau = c_var + 2 * b_var * (rel_cur_time + 1 / omega) + 3 * a_var * (rel_cur_time * rel_cur_time + 2 * rel_cur_time / omega + 2 / omega2);

        c_1 = (cp - A - (landing_point - (A + B_tau * flight_time)) / (omega_T + 1) * std::exp(-omega * (supporting_time - rel_cur_time))) / (-(omega * supporting_time * flight_time + flight_time + supporting_time) / (omega_T + 1) * omega * omega * omega * std::exp(omega * rel_cur_time) - (omega * omega * flight_time * flight_time + omega * flight_time - 2) / (2 * flight_time * (omega_T + 1)) * std::exp(omega * (2 * rel_land_time - 2 * supporting_time + rel_cur_time)) + omega * omega * omega * rel_cur_time * std::exp(omega * rel_cur_time) - (omega * omega * flight_time + 2 * omega) / (2 * flight_time) * std::exp(omega * (2 * rel_land_time - rel_cur_time)));

        ref_zmp = a_var * rel_cur_time * rel_cur_time * rel_cur_time + b_var * rel_cur_time * rel_cur_time + c_var * rel_cur_time + d_var;
    }

    // 4次関数
    // {
    //     const double tau = supporting_time;
    //     const double tau2 = tau * tau;
    //     const double tau3 = tau2 * tau;
    //     const double tau4 = tau2 * tau2;
    //     const double omega2 = omega * omega;
    //     const double omega3 = omega2 * omega;
    //     const double omega4 = omega2 * omega2;

    //     hrp::Vector3 a_var = hrp::Vector3::Zero();
    //     hrp::Vector3 b_var = hrp::Vector3::Zero();
    //     hrp::Vector3 c_var = hrp::Vector3::Zero();
    //     hrp::Vector3 d_var = hrp::Vector3::Zero();
    //     hrp::Vector3 e_var = hrp::Vector3::Zero();

    //     {
    //         Eigen::Matrix<double, 7, 7> A;
    //         A <<
    //             0, 0, 0, 0, 1, 0, 0,
    //             tau4, tau3, tau2, tau, 1, 0, 0,
    //             24 / omega4, 6 / omega3, 2 / omega2, 1 / omega, 1, 2, 0,
    //             4 * tau3 + 24 * tau / omega2, 3 * tau2 + 6 / omega2, 2 * tau, 1, 0, omega * std::exp(omega * tau), -omega * std::exp(-omega * tau),
    //             24 / omega4, 0, 2 / omega2, 0, 0, 1, 1,
    //             12 * tau2 / omega2 + 24 / omega4, 6 * tau / omega2, 2 / omega2, 0, 0, std::exp(omega * tau), std::exp(-omega * tau),
    //             4 * tau3 + 24 * tau / omega2, 3 * tau2 + 6 / omega2, 2 * tau, 1, 0, omega * std::exp(omega * tau), -omega * std::exp(-omega * tau);

    //         const auto A_qr = A.colPivHouseholderQr();
    //         for (size_t i = 0; i < 2; ++i) {
    //             const double next_offset = i == 1 ? -start_zmp_offset[i] : start_zmp_offset[i];
    //             Eigen::Matrix<double, 7, 1> B;
    //             B << (support_point + start_zmp_offset)[i], (support_point + end_zmp_offset)[i], support_point[i], next_offset * omega, 0, 0, (landing_point[i] + next_offset - (support_point[i] + start_zmp_offset[i])) / flight_time;

    //             const Eigen::Matrix<double, 7, 1> ans = A_qr.solve(B);

    //             a_var[i] = ans[0];
    //             b_var[i] = ans[1];
    //             c_var[i] = ans[2];
    //             d_var[i] = ans[3];
    //             e_var[i] = ans[4];
    //         }

    //         std::cerr << "e: " << (support_point + start_zmp_offset)[0] << std::endl;
    //     }

    //     const double t = rel_cur_time;
    //     const double t2 = t * t;
    //     const double t3 = t2 * t;
    //     const double t4 = t2 * t2;
    //     const hrp::Vector3 A = e_var + d_var * (1 / omega + t) + c_var * (t2 + 2 * t / omega + 2 / omega2) + b_var * (t3 + 3 * t2 / omega + 6 * t / omega2 + 6 / omega3) + a_var * (t4 + 4 * t3 / omega + 12 * t2 / omega2 + 24 * t / omega3 + 24 / omega4);
    //     const hrp::Vector3 B_tau = d_var + c_var * (2 * tau + 2 / omega) + b_var * (3 * tau2 + 6 * tau / omega + 6 / omega2) + a_var * (4 * tau3 + 12 * tau2 / omega + 24 * tau / omega2 + 24 / omega3);

    //     c_1 = (cp - A - (landing_point - (A + B_tau * flight_time)) / (omega_T + 1) * std::exp(-omega * (supporting_time - rel_cur_time))) / (-(omega * supporting_time * flight_time + flight_time + supporting_time) / (omega_T + 1) * omega * omega * omega * std::exp(omega * rel_cur_time) - (omega * omega * flight_time * flight_time + omega * flight_time - 2) / (2 * flight_time * (omega_T + 1)) * std::exp(omega * (2 * rel_land_time - 2 * supporting_time + rel_cur_time)) + omega * omega * omega * rel_cur_time * std::exp(omega * rel_cur_time) - (omega * omega * flight_time + 2 * omega) / (2 * flight_time) * std::exp(omega * (2 * rel_land_time - rel_cur_time)));

    //     ref_zmp = a_var * t4 + b_var * t3 + c_var * t2 + d_var * t + e_var;
    //     std::cerr << "a_var: " << a_var[0] << "b_var: " << b_var[0] << "c_var: " << c_var[0] << "d_var: " << d_var[0] << "e_var: " << e_var[0] << std::endl;
    // }

    const hrp::Vector3 lambda = -(std::exp(omega * rel_cur_time) + (omega_T + 2) / (omega_T) * std::exp(omega * (2 * rel_land_time - rel_cur_time))) * c_1;

    hrp::Vector3 input_zmp = ref_zmp + omega * omega * lambda;

    if (rel_cur_time < supporting_time) cog_acc = omega * omega * (cog - input_zmp); // TODO: X, Y, Zに分解
    else {
        cog_acc = hrp::Vector3(0, 0, -g_acc); // Flight phase
        input_zmp.setZero();
        ref_zmp.setZero();
        c_1.setZero();
    }

    cog_acc[2] = 0;
    cog += cog_vel * dt + cog_acc * dt * dt * 0.5;
    cog_vel += cog_acc * dt;
    // cog_acc = input_zmp;
    // cog_acc = ref_zmp;
    // cog_acc[0] = Dn[0];
    // cog_acc[1] = c_1[0];
    cog_acc[1] = input_zmp[0];
    cog_acc[2] = ref_zmp[0];
}

}
