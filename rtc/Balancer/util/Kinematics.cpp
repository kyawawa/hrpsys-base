// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  Kinematics.cpp
 * @brief
 * @author Tatsuya Ishikawa
 */

#include "Kinematics.h"
#include <iostream>

bool solveWeightedWholebodyIK(cnoid::Position* position,
                              // std::vector<cnoid::LinkPtr>& joints,
                              cnoid::Body::ContainerWrapper<std::vector<cnoid::LinkPtr>> joints,
                              const std::vector<IKParam>& ik_params,
                              const std::function<void()> calcFK,
                              const double err_threshold,
                              const double dq_threshold,
                              const size_t max_iteration,
                              const double bias)
{
    // TODO: Ordered IK
    // cnoid::VectorX lambda;

    // Set the rank of configuration space
    size_t rank_c_space = joints.end() - joints.begin();
    if (position) rank_c_space += 6; // Whole body IK

    size_t task_dof = 0;
    for (IKParam ik_param : ik_params) {
        if (ik_param.target_type == ik_3daffine) task_dof += 6;
        else task_dof += 3;
    }

    // Eigen::VectorXd IK_weight(task_dof);
    Eigen::VectorXd IK_weight = Eigen::VectorXd::Ones(task_dof);
    Eigen::VectorXd error = Eigen::VectorXd::Zero(task_dof);
    Eigen::MatrixXd jacobian(task_dof, rank_c_space);
    const Eigen::VectorXd bias_vec = bias * Eigen::VectorXd::Ones(rank_c_space);

    // Dirty code
    {
        size_t dof_accum = 0;
        for (IKParam ik_param : ik_params) {
            size_t dof = 3;
            if (ik_param.target_type == ik_3daffine) dof = 6;
            IK_weight.segment(dof_accum, dof) = ik_param.IK_weight;
            dof_accum += dof;
        }
    }

    for (size_t _ = 0; _ < max_iteration; ++_) {
        size_t dof_accum = 0;
        for (IKParam ik_param : ik_params) {
            size_t dof = 3;
            if (ik_param.target_type == ik_3daffine) dof = 6;
            jacobian.block(dof_accum, 0, dof, rank_c_space) = ik_param.calcJacobian();
            error.segment(dof_accum, dof) = ik_param.calcError();
            dof_accum += dof;
        }

        if (error.norm() < err_threshold) {
            // std::cerr << "error: " << error.transpose() << std::endl;
            return true; // Completed
        }

        const Eigen::MatrixXd damping_mat = (0.5 * error.transpose() * IK_weight.asDiagonal() * error + bias_vec).asDiagonal();

        // std::cerr << "jacobian:\n" << jacobian << std::endl;

        // std::cerr << "J: " << jacobian.rows() << "x" << jacobian.cols() << ", IK_weight: " << IK_weight.rows() << std::endl;
        // std::cerr << "weight: \n" << IK_weight.asDiagonal() << std::endl;
        // std::cerr << (jacobian.transpose() * IK_weight.asDiagonal() * jacobian + bias_mat).inverse() << std::endl;
        // std::cout << (jacobian.transpose() * IK_weight.asDiagonal() * jacobian + Eigen::MatrixXd(bias_mat.asDiagonal())) << std::endl;
        // std::cerr << "jacobian:\n" << jacobian << std::endl;
        // std::cerr << "IK_weight: " << IK_weight.transpose() << std::endl;
        // std::cerr << "inv: \n"  << (jacobian.transpose() * IK_weight.asDiagonal() * jacobian + bias_mat) << std::endl;

        // const Eigen::VectorXd q_delta = (jacobian.transpose() * IK_weight.asDiagonal() * jacobian + bias_mat).inverse() *
        //     (jacobian.transpose() * IK_weight.asDiagonal() * error);
        const Eigen::VectorXd q_delta = (jacobian.transpose() * IK_weight.asDiagonal() * jacobian + damping_mat)
            .ldlt().solve((jacobian.transpose() * IK_weight.asDiagonal() * error));

        // const Eigen::VectorXd q_delta = jacobian.transpose() * (jacobian * jacobian.transpose() + bias_mat).inverse() * error;
        // std::cerr << "error norm: " << error.norm() << std::endl;
        // std::cerr << "error: " << error.transpose() << std::endl;
        // std::cerr << "q_delta: " << q_delta.transpose() << std::endl;
        // std::cerr << "q_delta joint0: " << q_delta[2] << std::endl;

        size_t idx_c_space = 0;
        for (auto joint : joints) joint->q() += q_delta[idx_c_space++];

        if (position) {
            // position->translate(q_delta.segment<3>(idx_c_space));
            position->translation() += q_delta.segment<3>(idx_c_space);
            const Eigen::Vector3d omega_delta(q_delta.segment<3>(idx_c_space + 3));
            position->rotate(Eigen::AngleAxisd(omega_delta.norm(), omega_delta.normalized()));
        }

        calcFK();
        // std::cerr << "q_delta: " << q_delta.transpose() << std::endl;
        if (q_delta.norm() < dq_threshold) {
            return true; // Completed
        }
    }

    return false; // Not completed
}
