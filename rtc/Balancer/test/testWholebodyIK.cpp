// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  testWholebodyIK.cpp
 * @brief
 * @author Tatsuya Ishikawa
 */

#include <iostream>
#include <cnoid/Body>
#include <cnoid/JointPath>
#include <cnoid/EigenTypes>
#include "../util/Kinematics.h"
#include <cnoid/BodyLoader>
#include <cnoid/EigenUtil>
#include <cnoid/Config>

namespace {
constexpr double G_ACC = -9.80665;
inline double deg2rad (double deg) { return deg * M_PI / 180.0; }

struct EETrans {
    std::string target_name, sensor_name;
    cnoid::JointPathPtr ee_path;
};

unsigned loop = 0;
cnoid::BodyPtr ioBody;
std::vector<EETrans> ee_trans_vec;
std::vector<IKParam> ik_params;
cnoid::Vector3 target_com_pos;
cnoid::Vector3 target_com_ang_vel;
std::vector<cnoid::Position> target_ee;
inline const std::vector<double> reset_pose_angles()
{
    std::vector<double> reset_pose_angles_deg{-0.004457, -21.692900, -0.012020, 47.672300, -25.930000, 0.014025,
            17.835600, -9.137590, -6.611880, -36.456000, 0.000000, 0.000000, 0.000000,
            -0.004457, -21.692900, -0.012020, 47.672300, -25.930000, 0.014025,
            17.835600, 9.137590, 6.611880, -36.456000, 0.000000, 0.000000, 0.000000,
            0.000000, 0.000000, 0.000000};
    for (size_t i = 0, n = reset_pose_angles_deg.size(); i < n; ++i) {
        reset_pose_angles_deg[i] = deg2rad(reset_pose_angles_deg[i]);
    }
    return std::move(reset_pose_angles_deg);
}
}

bool setIKLimbAsDefault();
void calcFK()
{
    ioBody->calcForwardKinematics();
    ioBody->calcCenterOfMass();
}


int main(int argc, char **argv)
{
    // Load Model File
    {
#ifdef CNOID_SHARE_DIR
        const std::string model_file = std::string(CNOID_SHARE_DIR) + "/" + std::string(CNOID_SHARE_SUBDIR) + "/model/SR1/SR1.body";
#else
        std::cerr << "CNOID_SHARE_DIR not found" << std::endl;
        return -1;
#endif
        cnoid::BodyLoader bodyLoader;
        std::cerr << "model: " << model_file << std::endl;
        ioBody = bodyLoader.load(model_file);
        if (!ioBody) {
            std::cerr << "failed to load model" << std::endl;
            return -1;
        }

        // check if the dof of ioBody match the number of joint in ioBody
        for (int i = 0, dof = ioBody->numJoints(); i < dof; i++) {
            if (i != ioBody->joint(i)->jointId()) {
                std::cerr << "jointId is not equal to the index number" << std::endl;
                return -1;
            }
        }
    }

    // Set Reset Pose
    {
        const std::vector<double> reset_pose = reset_pose_angles();
        for (int i = 0, dof = ioBody->numJoints(); i < dof; i++) {
            ioBody->joint(i)->q() = reset_pose[i];
        }
    }

    target_ee.resize(4);
    ee_trans_vec.reserve(4);
    ik_params.reserve(6); // EE + COM Pos + COM Ang Vel
    std::vector<std::string> ee_list{"RARM_WRIST_R", "LARM_WRIST_R",  "RLEG_ANKLE_R", "LLEG_ANKLE_R"};
    for (const auto& ee_target : ee_list) {
        EETrans ee_local_trans;

        ee_local_trans.target_name = ee_target;
        ee_local_trans.ee_path = std::make_shared<cnoid::JointPath>(ioBody->rootLink(), ioBody->link(ee_target));
        ee_trans_vec.emplace_back(std::move(ee_local_trans));
    }

    setIKLimbAsDefault();
    // ik_params[0].IK_weight.setZero(); // debug
    // ik_params[1].IK_weight.setZero(); // debug
    ik_params[2].IK_weight.setZero();
    ik_params[3].IK_weight.setZero();

    while (loop < 100) {
        calcFK();
        target_com_pos = ioBody->centerOfMass();
        target_com_pos[2] += 0.01;
        target_com_ang_vel = cnoid::Vector3::Zero();
        std::cerr << "target_com: " << target_com_pos.transpose() << std::endl;
        for (size_t i = 0; i < target_ee.size(); ++i) {
            target_ee[i].translation() = ee_trans_vec[i].ee_path->endLink()->p();
            // target_ee[i].translation() += cnoid::Vector3(0, 0, 0.05);
            target_ee[i].linear() = ee_trans_vec[i].ee_path->endLink()->R();
        }


        // std::cerr << "q bef: ";
        // for (auto joint : ioBody->joints()) {
        //     std::cerr << joint->q() << ", ";
        // }
        // std::cerr << std::endl;
        if (solveWeightedWholebodyIK(&(ioBody->rootLink()->T()), ioBody->joints(),
                                     ik_params, calcFK, 1e-4, 1e-3, 200, 1e-1)) {
            std::cerr << "IK true" << std::endl;
        } else std::cerr << "IK False" << std::endl;
        // std::cerr << "q aft: ";
        // for (auto joint : ioBody->joints()) {
        //     std::cerr << joint->q() << ", ";
        // }
        // std::cerr << std::endl;

        // std::cerr << "root_t1: \n" << ioBody->rootLink()->T().matrix() << std::endl;
        std::cerr << "current_com: " << ioBody->centerOfMass().transpose() << std::endl;
        // std::cerr << "Error:\n";
        // for (auto& ik_param : ik_params) {
        //     std::cerr << ik_param.calcError().transpose() << std::endl;
        // }
        // std::cerr << "root_t2: \n" << ioBody->rootLink()->T().matrix() << std::endl;

        ++loop;
    }

    return 0;
}

bool setIKLimbAsDefault()
{
    {
        const size_t cap = ik_params.capacity();
        ik_params.clear();
        ik_params.reserve(cap);
    }

    // COM Position
    {
        IKParam com_ik(ik_trans);
        // com_ik.calcJacobian = [ioBodyWeak = BodyPtrWeak(ioBody)]()
        com_ik.calcJacobian = [&]()
            {
                Eigen::MatrixXd J;
                cnoid::calcCMJacobian(ioBody, nullptr, J); // RMC Eq.1 upper [M/m E -r]
                return std::move(J);
            };
        com_ik.calcError = [&]()
            {
                return target_com_pos - ioBody->centerOfMass();
            };
        ik_params.emplace_back(std::move(com_ik));
    }

    // COM Angular Velocity
    {
        IKParam com_ang(ik_axisrot); // Angular Vel
        com_ang.calcJacobian = [&]()
            {
                Eigen::MatrixXd J;
                cnoid::calcAngularMomentumJacobian(ioBody, nullptr, J); // RMC Eq.1 lower [H 0 I]
                return std::move(J);
            };
        com_ang.calcError = [&]()
            {
                cnoid::Vector3 tmp_P, L;
                ioBody->calcTotalMomentum(tmp_P, L);
                return target_com_ang_vel - L;
            };
        ik_params.emplace_back(std::move(com_ang));
    }

    // End-effector IK parameters
    for (size_t i = 0; i < target_ee.size(); ++i) {
        IKParam ee_ikparam(ik_3daffine);
        ee_ikparam.calcJacobian = [&, i]()
            {
                auto ee_path = ee_trans_vec[i].ee_path;
                size_t path_numjoints = ee_path->numJoints();

                Eigen::MatrixXd J_path(6, path_numjoints);
                cnoid::setJacobian<0x3f, 0, 0>(*ee_path, ee_path->endLink(), J_path);

                Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 6 + ioBody->numJoints());
                for (size_t idx = 0; idx < path_numjoints; ++idx) {
                    J.col(ee_path->joint(idx)->jointId()) = std::move(J_path.col(idx));
                }

                Eigen::MatrixXd J_bodypart(6, 6);
                J_bodypart << Eigen::Matrix3d::Identity(), cnoid::hat(ioBody->rootLink()->p() - ee_path->endLink()->p()),
                              Eigen::Matrix3d::Zero(),     Eigen::Matrix3d::Identity();

                J.block<6, 6>(0, ioBody->numJoints()) = std::move(J_bodypart);
                return std::move(J);
            };
        ee_ikparam.calcError = [&, i]()
            {
                cnoid::Vector6 error;
                error.head<3>() = target_ee[i].translation() - ee_trans_vec[i].ee_path->endLink()->p();
                error.segment<3>(3) = target_ee[i].linear() * cnoid::omegaFromRot(target_ee[i].linear() * ee_trans_vec[i].ee_path->endLink()->R().transpose());
                return std::move(error);
            };
        ik_params.emplace_back(std::move(ee_ikparam));
    }

    return true;
}
