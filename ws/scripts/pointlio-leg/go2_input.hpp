#pragma once
#include "leg_observation.hpp"
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include <map>
#include <deque>

namespace go2_leg {
using Sport = unitree_go::msg::SportModeState;
using Low = unitree_go::msg::LowState;
using Key = std::array<float,6>;
struct Observation {
    double time;
    Vec gyro;
    std::array<Vec,4> position, velocity;
    std::array<bool,4> contact;
};
class Input {
    std::map<Key,Low::ConstSharedPtr> lows;
    std::deque<Key> order;
    std::deque<Sport::ConstSharedPtr> pending;
    std::deque<Observation> observations;
    std::array<bool,4> contacts{{false,false,false,false}};
    double last_imu=-1, last_update=-1;
    bool body_imu=false;
    Mat imu_from_body=Mat::Identity();
    Vec body_origin_in_imu=Vec::Zero();
    std::ofstream audit;
    template<class M> static Key key(const M& m) {
        Key k;
        for(int i=0;i<3;++i) { k[i]=m.imu_state.accelerometer[i]; k[3+i]=m.imu_state.gyroscope[i]; }
        return k;
    }
    static bool finite(const Key& k) { for(auto v:k) if(!std::isfinite(v)) return false; return true; }
    void emit(const Sport::ConstSharedPtr& sport, const Low::ConstSharedPtr& low) {
        double time=sport->stamp.sec+sport->stamp.nanosec*1e-9;
        if(time<=last_imu) return;
        last_imu=time;
        auto imu=std::make_shared<sensor_msgs::msg::Imu>();
        imu->header.stamp.sec=sport->stamp.sec; imu->header.stamp.nanosec=sport->stamp.nanosec;
        imu->header.frame_id="base_link";
        imu->linear_acceleration.x=sport->imu_state.accelerometer[0];
        imu->linear_acceleration.y=sport->imu_state.accelerometer[1];
        imu->linear_acceleration.z=sport->imu_state.accelerometer[2];
        imu->angular_velocity.x=sport->imu_state.gyroscope[0];
        imu->angular_velocity.y=sport->imu_state.gyroscope[1];
        imu->angular_velocity.z=sport->imu_state.gyroscope[2];
        if(body_imu) imu_cbk(imu);
        ++imu_count;
        if(!low) { ++unpaired_count; return; }
        ++paired_count;
        Observation o; o.time=time-time_lag_imu_to_lidar;
        o.gyro=Vec(imu->angular_velocity.x,imu->angular_velocity.y,imu->angular_velocity.z);
        for(int leg=0;leg<4;++leg) {
            contacts[leg]=contacts[leg] ? low->foot_force[leg]>=force_down : low->foot_force[leg]>=force_up;
            Vec q,dq;
            for(int j=0;j<3;++j) { q[j]=low->motor_state[3*leg+j].q; dq[j]=low->motor_state[3*leg+j].dq; }
            foot(leg,q,dq,o.position[leg],o.velocity[leg]);
            o.contact[leg]=contacts[leg] && q.allFinite() && dq.allFinite();
            if(!body_imu) {
                o.position[leg]=imu_from_body*o.position[leg]+body_origin_in_imu;
                o.velocity[leg]=imu_from_body*o.velocity[leg];
            }
        }
        observations.push_back(o);
        if(observations.size()>4096) observations.pop_front();
    }
    void drain() {
        while(!pending.empty()) {
            auto ready=std::find_if(pending.begin(),pending.end(),[this](const auto& s){return lows.count(key(*s));});
            if(ready==pending.end()) return;
            while(pending.begin()!=ready) {
                emit(pending.front(),nullptr); pending.pop_front();
                ready=std::find_if(pending.begin(),pending.end(),[this](const auto& s){return lows.count(key(*s));});
            }
            auto s=pending.front(); pending.pop_front(); emit(s,lows.at(key(*s)));
        }
    }
public:
    bool enabled=false;
    double variance=.5, interval=.02, max_age=.01, gate=11.345;
    int force_up=35,force_down=25;
    size_t imu_count=0,paired_count=0,unpaired_count=0,updates=0,rejected=0;
    rclcpp::Subscription<Sport>::SharedPtr sport_sub;
    rclcpp::Subscription<Low>::SharedPtr low_sub;
    void start(const rclcpp::Node::SharedPtr& node) {
        enabled=node->declare_parameter("leg.enabled",false);
        body_imu=node->declare_parameter("leg.body_imu",false);
        if(!body_imu && !enabled) return;
        if(body_imu && (!use_imu_as_input || !use_acceleration || !imu_en))
            throw std::runtime_error("Body IMU requires acceleration and the Point-LIO input model");
        if(!body_imu && use_imu_as_input)
            throw std::runtime_error("L1 leg corrections currently require the Point-LIO output model");
        if(!body_imu) {
            auto r=node->declare_parameter<std::vector<double>>("leg.body_from_lidar_R",std::vector<double>{});
            auto t=node->declare_parameter<std::vector<double>>("leg.body_from_lidar_T",std::vector<double>{});
            if(r.size()!=9 || t.size()!=3) throw std::runtime_error("Body/L1 extrinsics are required for leg observations");
            Mat body_from_lidar;
            for(int i=0;i<9;++i) body_from_lidar(i/3,i%3)=r[i];
            Vec body_lidar(t[0],t[1],t[2]);
            if(!body_from_lidar.allFinite() || !body_lidar.allFinite() ||
               (body_from_lidar.transpose()*body_from_lidar-Mat::Identity()).norm()>1e-5 ||
               std::abs(body_from_lidar.determinant()-1)>1e-5)
                throw std::runtime_error("Invalid body/L1 rigid transform");
            imu_from_body=Lidar_R_wrt_IMU*body_from_lidar.transpose();
            body_origin_in_imu=Lidar_T_wrt_IMU-imu_from_body*body_lidar;
        }
        variance=node->declare_parameter("leg.velocity_variance",.5);
        interval=node->declare_parameter("leg.update_interval",.02);
        max_age=node->declare_parameter("leg.max_age",.01);
        gate=node->declare_parameter("leg.innovation_gate",11.345);
        force_up=node->declare_parameter("leg.force_up",35);
        force_down=node->declare_parameter("leg.force_down",25);
        if(!(variance>0 && interval>0 && max_age>0 && gate>0 && force_up>force_down))
            throw std::runtime_error("Invalid leg observation parameters");
        audit.open(root_dir+"leg-updates.csv");
        audit << "time,contacts,accepted,nis,residual_mps,imu,paired,unpaired\n";
        sport_sub=node->create_subscription<Sport>("/lidar3d/kinematic_sync",rclcpp::SensorDataQoS().keep_last(2048),
            [this](Sport::ConstSharedPtr s){
                if(!finite(key(*s))) return;
                pending.push_back(s);
                if(pending.size()>512) { emit(pending.front(),nullptr); pending.pop_front(); }
                drain();
            });
        low_sub=node->create_subscription<Low>("/lowstate",rclcpp::SensorDataQoS().keep_last(2048),
            [this](Low::ConstSharedPtr l){
                Key k=key(*l); if(!finite(k)) return;
                if(lows.emplace(k,l).second) {
                    order.push_back(k);
                    if(order.size()>512) { lows.erase(order.front()); order.pop_front(); }
                }
                drain();
            });
        RCLCPP_INFO(node->get_logger(),"Point-LIO IMU=%s; stance velocity corrections=%s",body_imu?"body":"L1",enabled?"enabled":"disabled");
    }
    bool replaces_imu() const { return body_imu; }
    ~Input() {
        if(!audit.is_open()) return;
        std::ofstream summary(root_dir+"leg-input-summary.json");
        summary << "{\"imu\":" << imu_count << ",\"paired\":" << paired_count << ",\"unpaired\":" << unpaired_count
                << ",\"updates\":" << updates << ",\"rejected\":" << rejected << "}\n";
    }
    void update(double time) {
        if(observations.empty() || observations.front().time>time) return;
        Observation o=observations.front(); observations.pop_front();
        while(!observations.empty() && observations.front().time<=time) { o=observations.front(); observations.pop_front(); }
        if(!enabled || time-o.time>max_age || time-last_update<interval) return;
        last_update=time;
        Vec p=Vec::Zero(), v=Vec::Zero(); int count=0;
        for(int i=0;i<4;++i) if(o.contact[i]) { p+=o.position[i]; v+=o.velocity[i]; ++count; }
        if(!count) return;
        p/=count; v/=count;
        const Mat rot=body_imu ? kf_input.get_x().rot.toRotationMatrix() : kf_output.get_x().rot.toRotationMatrix();
        const Vec velocity=body_imu ? Vec(kf_input.get_x().vel) : Vec(kf_output.get_x().vel);
        const Vec omega=body_imu ? o.gyro-Vec(kf_input.get_x().bg) : Vec(kf_output.get_x().omg);
        Mat noise=variance*Mat::Identity();
        // Disagreement among stance feet inflates uncertainty; contacts are not
        // independent measurements, so the common covariance is NOT divided by N.
        for(int i=0;i<4;++i) if(o.contact[i]) {
            Vec d=o.velocity[i]+omega.cross(o.position[i])-v-omega.cross(p);
            noise+=d*d.transpose()/count;
        }
        const int n=body_imu?24:30;
        Eigen::MatrixXd h;
        // Input-model state[15:18] is gyro bias; output-model state[15:18]
        // is angular velocity. Their derivatives have opposite signs.
        if(body_imu) h=jacobian(rot,velocity,p); else h=output_jacobian(rot,velocity,p);
        const Vec r=-(rot.transpose()*velocity+omega.cross(p)+v);
        Eigen::MatrixXd covariance;
        if(body_imu) covariance=kf_input.get_P(); else covariance=kf_output.get_P();
        const Eigen::MatrixXd pht=covariance*h.transpose();
        Mat innovation=h*pht+noise;
        Eigen::LDLT<Mat> solver(innovation);
        if(solver.info()!=Eigen::Success || !solver.isPositive()) return;
        double nis=r.dot(solver.solve(r));
        bool accept=std::isfinite(nis) && nis<=gate;
        audit << std::setprecision(17) << time << ',' << count << ',' << accept << ',' << nis << ',' << r.norm()
              << ',' << imu_count << ',' << paired_count << ',' << unpaired_count << '\n';
        if(!accept) { ++rejected; return; }
        const Eigen::MatrixXd gain=solver.solve(pht.transpose()).transpose();
        const Eigen::VectorXd dx=gain*r;
        const Eigen::MatrixXd a=Eigen::MatrixXd::Identity(n,n)-gain*h;
        covariance=a*covariance*a.transpose()+gain*noise*gain.transpose();
        Eigen::MatrixXd reset=Eigen::MatrixXd::Identity(n,n);
        reset.block<3,3>(3,3)=right_jacobian(dx.segment<3>(3));
        reset.block<3,3>(6,6)=right_jacobian(dx.segment<3>(6));
        covariance=reset*covariance*reset.transpose();
        covariance=(.5*(covariance+covariance.transpose())).eval();
        if(body_imu) {
            auto state=kf_input.get_x(); Eigen::Matrix<double,24,1> delta=dx; state.boxplus(delta);
            Eigen::Matrix<double,24,24> cov=covariance; kf_input.change_x(state); kf_input.change_P(cov);
        } else {
            auto state=kf_output.get_x(); Eigen::Matrix<double,30,1> delta=dx; state.boxplus(delta);
            Eigen::Matrix<double,30,30> cov=covariance; kf_output.change_x(state); kf_output.change_P(cov);
        }
        ++updates;
    }
};
Input input;
}
