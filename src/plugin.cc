#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <functional>
#include <ignition/math/Vector3.hh>
#include <Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>

#define PI      3.141592
#define D2R     3.141592/180
#define R2D     180/PI

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

namespace gazebo {

    class DISTURBANCE_GENERATOR : public ModelPlugin {
        physics::LinkPtr BASE_LINK;
        physics::LinkPtr ROD_LINK;
        physics::LinkPtr MASS_LINK;

        physics::JointPtr ROD_JOINT;
        physics::JointPtr MASS_JOINT;

        physics::ModelPtr model;

        VectorXd init_joint = VectorXd::Zero(2);
        VectorXd target_joint = VectorXd::Zero(2);
        VectorXd goal_joint = VectorXd::Zero(2);
        VectorXd target_joint_dot = VectorXd::Zero(2);

        VectorXd actual_joint = VectorXd::Zero(2);
        VectorXd actual_joint_dot = VectorXd::Zero(2);

        VectorXd torque = VectorXd::Zero(2);
        VectorXd kp = VectorXd::Zero(2);
        VectorXd kd = VectorXd::Zero(2);

        //setting for getting <dt>(=derivative time)
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt = 0.001;
        unsigned int cnt = 0;
        double step_cnt = 3000;

        ros::NodeHandle n;
        ros::Subscriber S_target1;
        bool Lock_flag = false;
        math::Vector3 measured_force;
        math::Vector3 measured_force2;
        
        ros::Publisher ros_pub1;
        double TmpData[25];
        std_msgs::Float64MultiArray ros_msg1;
        
        //For model load
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
        void UpdateAlgorithm();
        void Get_Actual_Data();
        void Locking();
        void Unlocking();
        void Callback1(const std_msgs::Int32Ptr &msg);
        void ROSMsgPublish1();
    };
    GZ_REGISTER_MODEL_PLUGIN(DISTURBANCE_GENERATOR);
}

void gazebo::DISTURBANCE_GENERATOR::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) {
    // model = link + joint +sensor
    this->model = _model;

    //LINK DEFINITION
    this->BASE_LINK = this->model->GetLink("base_link");
    this->ROD_LINK = this->model->GetLink("rod_link");
    this->MASS_LINK = this->model->GetLink("mass_link");

    //JOINT DEFINITION
    this->ROD_JOINT = this->model->GetJoint("rod_joint");
    this->MASS_JOINT = this->model->GetJoint("mass_joint");

    this->last_update_time = this->model->GetWorld()->GetSimTime();
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DISTURBANCE_GENERATOR::UpdateAlgorithm, this));
    
    ros_pub1 = n.advertise<std_msgs::Float64MultiArray>("/tmp_data2/", 1000);
    ros_msg1.data.resize(15);
    //ROS Communication setting
    S_target1 = n.subscribe("Lock_ON", 1, &gazebo::DISTURBANCE_GENERATOR::Callback1, this);
}

void gazebo::DISTURBANCE_GENERATOR::UpdateAlgorithm() {
    //QP_TEST();
    //* Calculate time
    common::Time current_time = this->model->GetWorld()->GetSimTime();

    Get_Actual_Data();
    
    kp << 4000, 0;
    kd << 10, 0;

    if (Lock_flag == true) {
        Locking();
    } else {
        Unlocking();
    }

    //* Apply torque to joint
    this->ROD_JOINT->SetForce(1, torque[0]);
    this->MASS_JOINT->SetForce(1, torque[1]);

    measured_force=this->MASS_LINK->GetWorldForce();
    measured_force2=this->MASS_LINK->GetRelativeForce();
    
    ROSMsgPublish1();
    
    //*setting for getting dt
    this->last_update_time = current_time;
}

void gazebo::DISTURBANCE_GENERATOR::Callback1(const std_msgs::Int32Ptr &msg) {
    Lock_flag = msg->data;
}

void gazebo::DISTURBANCE_GENERATOR::ROSMsgPublish1() {
    //********************* DH : Data plot ***************************//

    TmpData[0] = measured_force.y;
    TmpData[1] = measured_force2.y;
 
    for (unsigned int i = 0; i < 10; ++i) {
        ros_msg1.data[i] = TmpData[i];
    }

    ros_pub1.publish(ros_msg1);
}

void gazebo::DISTURBANCE_GENERATOR::Locking(void) {
    if (cnt == 0) {
        init_joint = actual_joint;
        goal_joint << -90 * D2R, 0.0;
        target_joint = init_joint;
        target_joint_dot << 0.0, 0.0;
        cnt++;
    } else if (cnt < step_cnt) {
        target_joint = init_joint + (goal_joint - init_joint) / 2.0 * (1 - cos(PI / step_cnt * cnt));
        target_joint_dot = PI / step_cnt * (goal_joint - init_joint) / 2.0 * sin(PI / step_cnt * cnt);
        cnt++;
    } else {
        target_joint = goal_joint;
        target_joint_dot << 0.0, 0.0;
    }
    //Calculate Torque
    for (int i = 0; i < 2; i++) {
        torque[i] = kp[i]*(target_joint[i] - actual_joint[i]) + kd[i]*(target_joint_dot[i] - actual_joint_dot[i]);
    }
}

void gazebo::DISTURBANCE_GENERATOR::Unlocking(void) {
    for (int i = 0; i < 2; i++) {
    //torque [i] = kd[i]*(target_joint_dot[i] - actual_joint_dot[i]);
    torque [i] = 0;
    }
    cnt = 0;
}


void gazebo::DISTURBANCE_GENERATOR::Get_Actual_Data(void) {
    actual_joint[0] = this->ROD_JOINT->GetAngle(1).Radian();
    actual_joint[1] = this->MASS_JOINT->GetAngle(1).Radian();

    actual_joint_dot[0] = this->ROD_JOINT->GetVelocity(1);
    actual_joint_dot[1] = this->MASS_JOINT->GetVelocity(1);
}


