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
    const double flight_time = 2 * take_off_z_vel / g_acc;

    const double cog_height = 0.8;
    const double omega = std::sqrt(g_acc / cog_height);
    const double rel_cur_time = cur_time - start_time;
    const double rel_land_time = landing_time - start_time;
    const double omega_tau = omega * supporting_time;

    // TODO: 本当はx,yのみ
    const hrp::Vector3 cp = cog + cog_vel / omega;
    const double omega_T = omega * flight_time;
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
    std::vector<double> tau(3);
    const double diff_tau = supporting_time / 3.0;
    tau[0] = diff_tau;
    tau[1] = tau[0] + diff_tau;
    tau[2] = supporting_time;

    hrp::Vector3 start_cp = support_point;
    hrp::Vector3 c_1 = hrp::Vector3::Zero();

    const hrp::Vector3 start_zmp_offset = hrp::Vector3(-0.1, 0, 0);
    const hrp::Vector3 end_zmp_offset = hrp::Vector3(0.1, 0, 0);
    // const hrp::Vector3 start_zmp_offset = hrp::Vector3(-0.05, 0, 0);
    // const hrp::Vector3 end_zmp_offset = hrp::Vector3(0.05, 0, 0);

    std::vector<hrp::Vector3> an(3);
    std::vector<hrp::Vector3> bn(3);

    // TODO: input_zmpの接続の確認、cog_velの接続も

    // an.back() = (end_zmp_offset - start_zmp_offset) * 2 / 5.0 / diff_tau;
    an.back() = (end_zmp_offset - start_zmp_offset) * -1 / 20.0 / diff_tau;
    // an.back() = (end_zmp_offset - start_zmp_offset) / 3.0 / diff_tau;
    // an.back() = hrp::Vector3::Zero();
    // bn.back() = support_point;
    bn.back() = (support_point + end_zmp_offset) - an.back() * supporting_time;

    an[0] = an.back();
    // bn[0] = support_point;
    bn[0] = support_point + start_zmp_offset;

    // an[1] = an.back();
    an[1] = ((end_zmp_offset - start_zmp_offset) - an.back() * (tau[2] - tau[1]) - an[0] * (tau[2] - tau[1])) / (tau[1] - tau[0]);
    bn[1] = bn[0] + an[0] * tau[0] - an[1] * tau[0];

    // const auto calcC1FromStartCp = [&](const double a, const double b, const hrp::Vector3& start_cp) {
    //     return (cp - (b + (1 / omega + rel_cur_time) * a + (start_cp - b - (1 / omega + tau) * a) * std::exp(omega * (rel_cur_time - tau)))) /
    //     (omega * omega * omega * rel_cur_time * std::exp(omega * rel_cur_time) +
    //      -(omega * omega * flight_time + 2 * omega) / (2 * flight_time) * std::exp(omega * (2 * rel_land_time - rel_cur_time)) +
    //      -omega * omega * omega * tau * std::exp(omega * rel_cur_time) +
    //      (omega * omega * flight_time + 2 * omega) / (2 * flight_time) * std::exp(omega * (2 * rel_land_time + rel_cur_time - 2 * tau)));
    // }

    const auto calcC1OfCurrentPhase = [&](const size_t idx) {
        // hrp::Vector3 Dn = an.back() * std::exp(-omega * tau[tau.size() - 2]) - an[idx] * std::exp(-omega * tau[idx]);
        // for (size_t i = idx; i < an.size() - 2; ++i) {
        //     Dn += an[i + 1] * (std::exp(-omega * tau[i]) - std::exp(-omega * tau[i + 1]));
        // }
        // Dn /= omega;

        hrp::Vector3 Dn = hrp::Vector3::Zero();
        for (size_t i = idx; i < an.size() - 1; ++i) {
            Dn += (an[idx + 1] - an[idx]) * std::exp(-omega * tau[idx]);
        }
        Dn /= omega;

        return (cp - (bn[idx] + (rel_cur_time + 1 / omega) * an[idx] +
                      Dn * std::exp(omega * rel_cur_time) +
                      (landing_point - (flight_time + 1 / omega - supporting_time) * an.back() - bn.back()) / (omega_T + 1) * std::exp(-omega * (supporting_time - rel_cur_time)))) /
        (omega * omega * omega * rel_cur_time * std::exp(omega * rel_cur_time) +
         -(omega_T + 2) / (2 * flight_time) * omega * std::exp(omega * (2 * rel_land_time - rel_cur_time)) +
         -(omega * flight_time * supporting_time + supporting_time + flight_time) / (omega_T + 1) * omega * omega * omega * std::exp(omega * rel_cur_time) +
         -(omega * omega * flight_time * flight_time + omega_T - 2) / (2 * flight_time * (omega_T + 1)) * omega * std::exp(omega * (2 * rel_land_time - 2 * supporting_time + rel_cur_time)));
    };

    size_t idx = 0;
    while (rel_cur_time >= tau[idx] && idx < tau.size() - 1) ++idx;
    c_1 = calcC1OfCurrentPhase(idx);

    hrp::Vector3 ref_zmp = an[idx] * rel_cur_time + bn[idx]; // Debug

    // for (size_t i = 0; i < tau.size(); ++i) {
    //     if (rel_cur_time <
    // if (rel_cur_time < diff_tau) {
    //     c_1 = calcC1OfCurrentPhase(a1, b1, an, bn);
    //     ref_zmp = a1 * rel_cur_time + b1;
    // } else if (rel_cur_time < diff_tau * 2) {
    //     c_1 = calcC1OfCurrentPhase(a2, b2, an, bn);
    //     ref_zmp = a2 * rel_cur_time + b2;
    // } else {
    //     c_1 = calcC1OfCurrentPhase(an, bn, an, bn);
    //     ref_zmp = an * rel_cur_time + bn;
    // }

    const hrp::Vector3 lambda = -(std::exp(omega * rel_cur_time) + (omega_T + 2) / (omega_T) * std::exp(omega * (2 * rel_land_time - rel_cur_time))) * c_1;

    // const hrp::Vector3 lambda = -(cp - support_point + (landing_point - support_point) / (1 + omega_T) * std::exp(-omega * (rel_land_time - rel_cur_time))) * (std::exp(omega * rel_cur_time) + (omega_T + 2) / omega_T * std::exp(omega * (2 * rel_land_time - rel_cur_time))) /
    //     (-omega * (1 / flight_time + omega / 2) * std::exp(omega * (2 * rel_land_time - rel_cur_time)) +
    //      0.5 * omega * omega * omega * (1 / omega + 2 * rel_cur_time) * std::exp(omega * rel_cur_time) +
    //      omega * (1 / rel_cur_time + omega / 2) * (1 - omega_T) / (1 + omega_T) * std::exp(omega * (2 * rel_land_time - 2 * supporting_time + rel_cur_time)) -
    //      0.5 * omega * omega * omega * (1 / omega + 2 * supporting_time + 3 * flight_time + 2 * omega_T * supporting_time) * std::exp(omega * rel_cur_time));

    hrp::Vector3 input_zmp = support_point + omega * omega * lambda;

    // hrp::Vector3 input_zmp = support_point + 2 * (landing_point - support_point) * (std::exp(omega * rel_cur_time) - std::exp(-omega * rel_cur_time)) / ((omega_T + 3) * std::exp(omega * supporting_time) + (3 * omega_T + 1) * std::exp(-omega * supporting_time));

    if (rel_cur_time < supporting_time) cog_acc = omega * omega * (cog - input_zmp); // TODO: X, Y, Zに分解
    else {
        cog_acc = hrp::Vector3(0, 0, -g_acc); // Flight phase
        input_zmp = hrp::Vector3::Zero();
        ref_zmp = hrp::Vector3::Zero();
    }

    cog_acc[2] = 0;
    cog += cog_vel * dt + cog_acc * dt * dt * 0.5;
    cog_vel += cog_acc * dt;
    // cog_acc = input_zmp;
    // cog_acc = ref_zmp;
    cog_acc[1] = input_zmp[0];
    cog_acc[2] = ref_zmp[0];
}

}
