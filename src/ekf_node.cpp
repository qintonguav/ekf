#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <queue>
#include <vector>

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
bool flag_g_init = false;
bool flag_odom_init = false;
VectorXd x(16);
MatrixXd P = 100 * MatrixXd::Identity(15, 15);
MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt = MatrixXd::Identity(6,6);
Vector3d g_init = Vector3d::Zero();
Vector3d G;
queue<Matrix<double, 16, 1>> x_history;
queue<Matrix<double, 15, 15>> P_history;
int cnt_g_init = 0;
double t;
queue<sensor_msgs::Imu::ConstPtr> imu_buf;
queue<nav_msgs::Odometry::ConstPtr> odom_buf;

//Rotation from the camera frame to the IMU frame
//Matrix3d imu_R_cam = Quaterniond(0, 0, -1, 0).toRotationMatrix();
//VectorXd imu_T_cam = Vector3d(0, -0.05, 0.02);
Matrix3d imu_R_cam = Quaterniond(0, 0, 1, 0).toRotationMatrix();
VectorXd imu_T_cam = Vector3d(0, -0.04, -0.02);
Eigen::Matrix3d Rcam;

void pub_odom(std_msgs::Header header)
{
    //pub odom
    Matrix3d vicon_R_tag;
    vicon_R_tag << 0, 1, 0,
                    1, 0 ,0,
                    0, 0 ,-1;
    Quaterniond Quat(x(0), x(1), x(2), x(3));
    Quat = vicon_R_tag * Quat;
    Vector3d vicon_p, vicon_v;
    vicon_p = vicon_R_tag * Vector3d(x(4), x(5), x(6));
    vicon_v = vicon_R_tag * Vector3d(x(7), x(8), x(9));
    nav_msgs::Odometry odom;
    odom.header.stamp = header.stamp;
    odom.header.frame_id = "world";
    odom.pose.pose.position.x = vicon_p(0);
    odom.pose.pose.position.y = vicon_p(1);
    odom.pose.pose.position.z = vicon_p(2);
    odom.pose.pose.orientation.w = Quat.w();
    odom.pose.pose.orientation.x = Quat.x();
    odom.pose.pose.orientation.y = Quat.y();
    odom.pose.pose.orientation.z = Quat.z();
    odom.twist.twist.linear.x = vicon_v(0);
    odom.twist.twist.linear.y = vicon_v(1);
    odom.twist.twist.linear.z = vicon_v(2);

    odom_pub.publish(odom);
}

void propagate(const sensor_msgs::ImuConstPtr &imu_msg)
{
    //ROS_INFO("propagation");
    double cur_t = imu_msg->header.stamp.toSec();
    VectorXd w(3);
    VectorXd a(3);
    a(0) = imu_msg->linear_acceleration.x;
    a(1) = imu_msg->linear_acceleration.y;
    a(2) = imu_msg->linear_acceleration.z;
    w(0) = imu_msg->angular_velocity.x;
    w(1) = imu_msg->angular_velocity.y;
    w(2) = imu_msg->angular_velocity.z;

    double dt = cur_t - t;
    Quaterniond R(x(0), x(1), x(2), x(3));
    x.segment<3>(4) += x.segment<3>(7) * dt + 0.5 * (R * (a - x.segment<3>(10)) - G) * dt * dt;
    x.segment<3>(7) += (R * (a - x.segment<3>(10)) - G) * dt;
    Vector3d omg = w - x.segment<3>(13);
    omg = omg * dt / 2;
    Quaterniond dR(sqrt(1 - omg.squaredNorm()), omg(0), omg(1), omg(2));
    Quaterniond R_now;
    R_now = (R * dR).normalized();            
    x.segment<4>(0) << R_now.w(), R_now.x(), R_now.y(), R_now.z();

    Vector3d w_x = w - x.segment<3>(13);
    Vector3d a_x = a - x.segment<3>(10);
    Matrix3d R_w_x, R_a_x;

    R_w_x<<0, -w_x(2), w_x(1),
        w_x(2), 0, -w_x(0),
        -w_x(1), w_x(0), 0;
    R_a_x<<0, -a_x(2), a_x(1),
        a_x(2), 0, -a_x(0),
        -a_x(1), a_x(0), 0;

    MatrixXd A = MatrixXd::Zero(15, 15);
    A.block<3,3>(0,0) = -R_w_x;
    A.block<3,3>(0,12) = -1 * MatrixXd::Identity(3,3);
    A.block<3,3>(3,6) = MatrixXd::Identity(3,3);
    A.block<3,3>(6,0) = (-1 * R.toRotationMatrix()) * R_a_x;
    A.block<3,3>(6,9) = (-1 * R.toRotationMatrix());
    //cout<<"A"<<endl<<A<<endl;

    MatrixXd U = MatrixXd::Zero(15,12);
    U.block<3,3>(0,0) = -1 * MatrixXd::Identity(3,3);
    U.block<3,3>(6,3) = -1 * R.toRotationMatrix();
    U.block<3,3>(9,6) = MatrixXd::Identity(3,3);
    U.block<3,3>(12,9) = MatrixXd::Identity(3,3);

    MatrixXd F, V;
    F = (MatrixXd::Identity(15,15) + dt * A);
    V = dt * U;
    P = F * P * F.transpose() + V * Q * V.transpose();

    t = cur_t;

}

void update(const nav_msgs::Odometry::ConstPtr &msg)
{
    //ROS_INFO("update");
    Quaterniond Quat_r;
    Quat_r.w() = msg->pose.pose.orientation.w;
    Quat_r.x() = msg->pose.pose.orientation.x;
    Quat_r.y() = msg->pose.pose.orientation.y;
    Quat_r.z() = msg->pose.pose.orientation.z;
    Matrix3d cam_R_w = Quat_r.toRotationMatrix();

    Vector3d cam_T_w;
    cam_T_w(0) = msg->pose.pose.position.x;
    cam_T_w(1) = msg->pose.pose.position.y;
    cam_T_w(2) = msg->pose.pose.position.z;

    Matrix3d R = cam_R_w.transpose() * imu_R_cam.transpose();
    Vector3d T = -cam_R_w.transpose() * (imu_R_cam.transpose() * imu_T_cam + cam_T_w);

    MatrixXd C = MatrixXd::Zero(6,15);
    C.block<3,3>(0,0) = Matrix3d::Identity();
    C.block<3,3>(3,3) = Matrix3d::Identity();
    //cout<<"C"<<endl<<C<<endl;

    MatrixXd K(15,6);
    K = P * C.transpose() * (C * P *C.transpose() + Rt).inverse();

    VectorXd r(6);
    Quaterniond qm(R);
    Quaterniond q = Quaterniond(x(0),x(1),x(2),x(3));
    Quaterniond dq = q.conjugate() * qm;
    r.head<3>() = 2 * dq.vec();
    r.tail<3>() = T - x.segment<3>(4);
    VectorXd _r = K * r;
    Vector3d dw (_r(0) / 2,_r(1) / 2,_r(2) / 2);
    dq = Quaterniond(1,dw(0),dw(1),dw(2)).normalized();
    q = q * dq;

    x(0) = q.w();
    x(1) = q.x();
    x(2) = q.y();
    x(3) = q.z();

    x.segment<12>(4) += _r.tail(12);
    P = P - K * C * P;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    ROS_INFO("IMU CALLBACK , TIME :%f", msg->header.stamp.toSec());
    //your code for propagation
    if (!flag_g_init && cnt_g_init < 30)
    {
        Vector3d a;
        a(0) = msg->linear_acceleration.x;
        a(1) = msg->linear_acceleration.y;
        a(2) = msg->linear_acceleration.z;
        cnt_g_init++;
        g_init += a;
    }
    if (!flag_g_init && cnt_g_init == 30)
    {
        g_init /= cnt_g_init;
        flag_g_init = true;
    }
    

    if(flag_g_init && flag_odom_init)
    {
        imu_buf.push(msg);
        propagate(msg);
        x_history.push(x);
        P_history.push(P);
        pub_odom(msg->header);
        cout << " quat " << x(0) << x(1) <<x (2) << x(3) << endl;
        cout << "    p " << x(4) << x(5) << x(6) << endl;
        cout << "    v " << x(7) << x(8) << x(9) << endl;
    }
    
}


void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    ROS_INFO("odom CALLBACK , TIME :%f", msg->header.stamp.toSec());
    //your code for update
    //camera position in the IMU frame = (0, -0.05, +0.02)
    //camera orientaion in the IMU frame = Quaternion(0, 0, -1, 0); w x y z, respectively
    if (!flag_odom_init)
    {
        double cur_t = msg->header.stamp.toSec();
        Quaterniond Quat_r;
        Quat_r.w() = msg->pose.pose.orientation.w;
        Quat_r.x() = msg->pose.pose.orientation.x;
        Quat_r.y() = msg->pose.pose.orientation.y;
        Quat_r.z() = msg->pose.pose.orientation.z;
        Matrix3d cam_R_w = Quat_r.toRotationMatrix();

        Vector3d cam_T_w;
        cam_T_w(0) = msg->pose.pose.position.x;
        cam_T_w(1) = msg->pose.pose.position.y;
        cam_T_w(2) = msg->pose.pose.position.z;

        Matrix3d w_R_imu = cam_R_w.transpose() * imu_R_cam.transpose();
        Vector3d w_T_imu = -cam_R_w.transpose() * (imu_R_cam.transpose() * imu_T_cam + cam_T_w);

        x.setZero();
        Quaterniond w_Q_imu(w_R_imu);
        x.head<4>() << w_Q_imu.w(),w_Q_imu.vec();
        x.segment<3>(4) = w_T_imu;
        t = cur_t;
        if(flag_g_init)
        {
            ROS_WARN_STREAM("average g vector:  " << g_init.transpose());
            G = w_R_imu * g_init;
            ROS_WARN_STREAM("gravity vector in world frame:  " << G.transpose());
            //G = G / G.norm() * 9.805;
            flag_odom_init = true;
            /*
            while (imu_buf.front()->header.stamp < msg->header.stamp)
            {
                imu_buf.pop();
            }
            */
        }
    }
    else
    {
        //odom_buf.push(msg);
        if(flag_g_init && flag_odom_init)
        {
            while(!imu_buf.empty() && imu_buf.front()->header.stamp < msg->header.stamp)
            {
                t = msg->header.stamp.toSec();
                imu_buf.pop();
                x_history.pop();
                P_history.pop();
            }
            if(!x_history.empty())
            {
                x = x_history.front();
                P = P_history.front();
            }
            update(msg);
            while(!x_history.empty()) x_history.pop();
            while(!P_history.empty()) P_history.pop();
            queue<sensor_msgs::Imu::ConstPtr> new_imu_buf;
            while(!imu_buf.empty())
            {
                propagate(imu_buf.front());
                new_imu_buf.push(imu_buf.front());
                x_history.push(x);
                P_history.push(P);
                imu_buf.pop();
            }
            std::swap(imu_buf, new_imu_buf);
        }
    }

}


/*
void process()
{
    if (!flag_g_init || !flag_odom_init)
        return;
    if(imu_buf.empty() || odom_buf.empty())
        return;
    if (!(imu_buf.back()->header.stamp > odom_buf.front()->header.stamp))
    {
        ROS_WARN("wait for imu");
        return;
    }
    if (!(imu_buf.front()->header.stamp < odom_buf.front()->header.stamp))
    {
        ROS_WARN("throw odom");
        odom_buf.pop();
        return;
    }

    nav_msgs::OdometryConstPtr odom_msg = odom_buf.front();
    odom_buf.pop();
    //double t = odom_msg->header.stamp.toSec();
    while (imu_buf.front()->header.stamp <= odom_msg->header.stamp)
    {
        propagate(imu_buf.front());
        imu_buf.pop();
    }
    update(odom_msg);
    pub_odom(odom_msg->header);
}
*/


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 100, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 100, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    Rcam = Quaterniond(0, 0, -1, 0).toRotationMatrix();
    ros::Rate r(100);
    Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);  // IMU noise  w   a  
    Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6); // IMU  noise  bg   ba
    Rt.topLeftCorner(3, 3) = 0.5 * Rt.topLeftCorner(3, 3);  // Measure orientation
    Rt.bottomRightCorner(3, 3) = 0.5 * Rt.bottomRightCorner(3, 3); // Measure  position
    Rt.bottomRightCorner(1, 1) = 0.5 * Rt.bottomRightCorner(1, 1); // Measure  position
    Matrix3d R_tmp;
    R_tmp << -1, 0 ,0,
              0, 1, 0,
              0, 0, -1;
    Quaterniond Q_tmp(R_tmp);
    cout << "Q_tmp " << Q_tmp.w() << Q_tmp.vec().transpose() << endl;
    ROS_WARN("EKF time sychro version!!!!!!!!!");
    /*
    while (ros::ok())
    {
        ros::spinOnce();
        process();
        r.sleep();
    }
    */
    ros::spin();
}
