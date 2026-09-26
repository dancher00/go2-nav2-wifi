#pragma once
#include <Eigen/Dense>
#include <array>
#include <cmath>

// Body-frame stance-foot velocity observation. No factory pose/velocity input.
namespace go2_leg {
using Vec = Eigen::Vector3d;
using Mat = Eigen::Matrix3d;
inline Mat skew(const Vec& v) {
    Mat s; s << 0,-v.z(),v.y(), v.z(),0,-v.x(), -v.y(),v.x(),0; return s;
}
inline void foot(int leg, const Vec& q, const Vec& dq, Vec& p, Vec& v) {
    const double side = leg%2 ? 1. : -1., front = leg<2 ? 1. : -1.;
    const double s=sin(q.x()), c=cos(q.x()), s23=sin(q.y()+q.z()), c23=cos(q.y()+q.z());
    const double reach=.213*(cos(q.y())+c23), bend=.213*(sin(q.y())+s23), hip=side*.0955;
    p << -bend+front*.1934, hip*c+reach*s+side*.0465, hip*s-reach*c;
    v << -reach*dq.y()-.213*c23*dq.z(),
        (-hip*s+reach*c)*dq.x()-s*bend*dq.y()-.213*s*s23*dq.z(),
        (hip*c+reach*s)*dq.x()+c*bend*dq.y()+.213*c*s23*dq.z();
}
inline Vec residual(const Mat& rotation, const Vec& velocity, const Vec& bias,
                    const Vec& gyro, const Vec& p, const Vec& foot_velocity) {
    return -(rotation.transpose()*velocity + (gyro-bias).cross(p) + foot_velocity);
}
inline Eigen::Matrix<double,3,24> jacobian(const Mat& rotation, const Vec& velocity, const Vec& p) {
    Eigen::Matrix<double,3,24> h=Eigen::Matrix<double,3,24>::Zero();
    h.block<3,3>(0,3)=skew(rotation.transpose()*velocity);
    h.block<3,3>(0,12)=rotation.transpose();
    h.block<3,3>(0,15)=skew(p);
    return h;
}
inline Eigen::Matrix<double,3,30> output_jacobian(const Mat& rotation, const Vec& velocity, const Vec& p) {
    Eigen::Matrix<double,3,30> h=Eigen::Matrix<double,3,30>::Zero();
    h.leftCols<24>()=jacobian(rotation,velocity,p);
    h.block<3,3>(0,15)=-skew(p); // True angular velocity, rather than gyro bias.
    return h;
}
inline Mat right_jacobian(const Vec& v) {
    double a=v.norm(); Mat s=skew(v);
    if(a<1e-6) return Mat::Identity()-.5*s+s*s/6.;
    return Mat::Identity()-(1-cos(a))/(a*a)*s+(a-sin(a))/(a*a*a)*s*s;
}
}
