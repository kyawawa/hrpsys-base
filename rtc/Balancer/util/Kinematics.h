// -*- mode: C++; coding: utf-8-unix; -*-

#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

// #include <cnoid/EigenTypes>
#include <cnoid/Body>
#include <cnoid/Jacobian>
#include <memory>
#include <functional>
#include <iostream>
enum IKTargetType {
    ik_3daffine, // AffineCompact3d
    ik_trans,    // 3d vector
    ik_axisrot,  // 3d vector
};

struct IKParam {
    IKTargetType target_type;
    // I wanna use union but...
    // cnoid::Position target_pos;
    // Eigen::Vector3d target_trans;
    // Eigen::Vector3d target_axisrot;
    std::function<Eigen::MatrixXd()> calcJacobian;
    std::function<Eigen::VectorXd()> calcError; // IKTargetTypeを使えばcalcTargetみたいなのだけでOK?
    // Limitation of choreonoid...
    // std::function<void(Eigen::Ref<Eigen::MatrixXd>)> calcJacobian;
    // std::function<void(Eigen::Ref<Eigen::VectorXd>)> calcError;

    // Eigen::DiagonalMatrix<double, 3> IK_weight;
    Eigen::VectorXd IK_weight;

    IKParam (IKTargetType type) : target_type(type)
    {
        switch(type) {
          case ik_3daffine:
              IK_weight = Eigen::VectorXd::Ones(6);
              break;
          case ik_trans:
          case ik_axisrot:
          default:
            IK_weight = Eigen::VectorXd::Ones(3);
            break;
        }
    }


    IKParam(const IKParam& ikp)            = default;
    IKParam(IKParam&& ikp)                 = default;
    IKParam& operator=(const IKParam& ikp) = default;
    IKParam& operator=(IKParam&& ikp)      = default;
};

using IKParamPtr = std::shared_ptr<IKParam>;

bool solveWeightedWholebodyIK(cnoid::Position* position, // Floating base, set nullptr if not wholebody
                              cnoid::Body::ContainerWrapper<std::vector<cnoid::LinkPtr>> joints, // Should use reference_wrapper and double?
                              // cnoid::BodyPtr ioBody,
                              const std::vector<IKParam>& ik_params,
                              const std::function<void()> calcFK, // Calculate forward kinematics and COM
                              const double err_threshold = 1e-4,
                              const double dq_threshold = 1e-4,
                              const size_t max_iteration = 100,
                              const double damping = 1e-1);

#endif // __KINEMATICS_H__
