#include "leg_observation.hpp"
#include <iostream>
#include <stdexcept>
using namespace go2_leg;
void require(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
int main() {
    const double eps=1e-7;
    // FK derivative checked against independently perturbed joint positions.
    for(int leg=0;leg<4;++leg) {
        Vec q(.15,.8,-1.5), dq(.2,-.3,.5), p,v,pp,vp;
        foot(leg,q,dq,p,v); foot(leg,q+eps*dq,Vec::Zero(),pp,vp);
        require(((pp-p)/eps-v).norm()<1e-7,"FK velocity derivative");
    }
    Mat rot=Eigen::AngleAxisd(.6,Vec(1,2,3).normalized()).toRotationMatrix();
    Vec vel(.4,-.2,.1), bias(.01,-.02,.03), gyro(.2,.3,-.1), p(.2,-.1,-.3), vf(.1,.2,.3);
    const auto h=jacobian(rot,vel,p);
    const auto ho=output_jacobian(rot,vel,p);
    const Vec base=residual(rot,vel,bias,gyro,p,vf);
    for(int axis=0;axis<3;++axis) {
        Vec d=eps*Vec::Unit(axis);
        Mat perturbed=rot*Eigen::AngleAxisd(eps,Vec::Unit(axis)).toRotationMatrix();
        require(((base-residual(perturbed,vel,bias,gyro,p,vf))/eps-h.col(3+axis)).norm()<1e-7,"rotation Jacobian sign/frame");
        require(((base-residual(rot,vel+d,bias,gyro,p,vf))/eps-h.col(12+axis)).norm()<1e-7,"velocity Jacobian");
        require(((base-residual(rot,vel,bias+d,gyro,p,vf))/eps-h.col(15+axis)).norm()<1e-7,"bias Jacobian sign");
        require(((base-residual(rot,vel,bias,gyro+d,p,vf))/eps-ho.col(15+axis)).norm()<1e-7,"output angular velocity Jacobian sign");
    }
    // A stance foot stays fixed in world under simultaneous rotation/translation.
    vf=-rot.transpose()*vel-(gyro-bias).cross(p);
    require(residual(rot,vel,bias,gyro,p,vf).norm()<1e-12,"fixed foot residual");
    // Preserve a fixed stance foot after moving the estimator origin to the L1
    // IMU. The origin velocity must include the rotational lever arm.
    Mat body_from_imu=Eigen::AngleAxisd(.4,Vec::UnitY()).toRotationMatrix();
    Vec imu_origin_body(.28,.01,-.03), omega=gyro-bias;
    Vec imu_velocity_world=vel+rot*omega.cross(imu_origin_body);
    require(residual(rot*body_from_imu,imu_velocity_world,Vec::Zero(),
                     body_from_imu.transpose()*omega,
                     body_from_imu.transpose()*(p-imu_origin_body),
                     body_from_imu.transpose()*vf).norm()<1e-12,"L1/body lever arm and velocity transform");
    require((right_jacobian(Vec::Zero())-Mat::Identity()).norm()<1e-12,"zero reset Jacobian");
    std::cout << "Point-LIO leg observation: FK and all active Jacobians passed\n";
}
