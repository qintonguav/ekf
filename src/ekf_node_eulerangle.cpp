#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
ros::Publisher odom_pub_update;


double t;
VectorXd x;
const static int STATE_CNT= 15;
const static int PROVESS_NOISE_CNT= 12;
bool isInit = false;
MatrixXd P = MatrixXd::Zero(15,15);
MatrixXd Q = MatrixXd::Identity(12,12);    // 
MatrixXd Rt = MatrixXd::Identity(6,6);    // 
Vector3d g=Vector3d::Zero(3);
int calLimit = 10;
int calCnt = 0;
struct State{
    double time;
    VectorXd x;
    MatrixXd P;
};
struct IMU{
    double time;
    VectorXd a;
    VectorXd w;
};
State x_now;


void state_init(double t)
{
    Matrix3d R_init;
    R_init<< 0.044408,     0.99875,   0.0229528,
            0.999003,  -0.0442893, -0.00565494,
            -0.00463131,    0.023181,   -0.999721;
    g = R_init.transpose() * g;
    g=-g;
    cout<<"g    "<<g.transpose()<<endl; //(0,0,10)
    puts("Init begin!");
    x = VectorXd::Zero(STATE_CNT);
    P = MatrixXd::Identity(STATE_CNT,STATE_CNT);
    x(0)=-0.001;
    x(1)=-2.33;
    x(2)=-1.02;
    x(3)=0.023;
    x(4)=3.13696;
    x(5)=-1.61511;
    //cout<<x<<endl;
    //cout<<P<<endl;
    isInit = true;  
    x_now.time = t;
    x_now.x = x;
    x_now.P = P;
}


State propagation(double cur_t,const State state_last,const IMU imu_now)
{

}

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{

    //your code for propagation
    double cur_t = msg->header.stamp.toSec();
    VectorXd w(3);
    VectorXd a(3);
    a(0) = msg->linear_acceleration.x;
    a(1) = msg->linear_acceleration.y;
    a(2) = msg->linear_acceleration.z;
    w(0) = msg->angular_velocity.x;
    w(1) = msg->angular_velocity.y;
    w(2) = msg->angular_velocity.z;
    IMU imu_now = {cur_t,a,w};
    cout<<"a   "<<a<<endl;
    cout<<"imu_a"<<imu_now.a<<endl;
    //cout<<"w"<<endl<<w<<endl;
    //cout<<"a"<<endl<<a<<endl;
    //cout<<"g"<<endl<<g<<endl;
    //Init x
    if (calCnt < calLimit)
    {
        calCnt++;
        g += a;
    }
    else if (calCnt == calLimit)
    {
        calCnt++;
        g /= calLimit;
        state_init(cur_t);
        t = cur_t;
        puts("Init end!");  
    }
    else
    {
        cout<<"propagation time:   "<<endl;
        printf("%f\n",cur_t);
        //state propagation
        double dt = cur_t - t;
        //cout<<"cur_t-t  :  "<<dt<<endl;
        double phi = x(3);
        double theta = x(4);
        double psi = x(5);
        /*
        phi = 0.2;
        theta = 0.3;
        psi = 0.4;
        a = Vector3d(0.1,0.2,0.3);
        w = Vector3d(1,2,3);
        */
        
        Matrix3d R,G;
        G<<cos(theta),0,-cos(phi)*sin(theta),
            0,1,sin(phi),
            sin(theta),0,cos(phi)*cos(theta);

        R<<cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi),
             cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi), sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi),
            -cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta);


        //cout<<"G"<<endl<<G<<endl;
        //cout<<"R"<<endl<<R<<endl;
        VectorXd f = VectorXd::Zero(STATE_CNT);
        f.segment<3>(0) = x.segment<3>(6); 
        f.segment<3>(3) = G.inverse()*(w - x.segment<3>(9));
        //cout<<"a---->world"<<endl<< (R * (a-x.segment<3>(12))).transpose()<<endl;
        f.segment<3>(6) = g + R * (a-x.segment<3>(12));
        x = x + dt * f;

        Quaterniond Quat;
        Quat = R;

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "my work";
        odom.pose.pose.position.x = x(0);
        odom.pose.pose.position.y = x(1);
        odom.pose.pose.position.z = x(2);
        odom.pose.pose.orientation.w = Quat.w();
        odom.pose.pose.orientation.x = Quat.x();
        odom.pose.pose.orientation.y = Quat.y();
        odom.pose.pose.orientation.z = Quat.z();
        odom.twist.twist.linear.x = x(6);
        odom.twist.twist.linear.y = x(7);
        odom.twist.twist.linear.z = x(8);

        odom_pub.publish(odom);

       //covarience propagation
        MatrixXd A = MatrixXd::Zero(STATE_CNT,STATE_CNT);
        
        Matrix3d R_dot,R_dot_phi,R_dot_theta,R_dot_psi;
        R_dot_phi<< -cos(phi)*sin(psi)*sin(theta),  sin(phi)*sin(psi),  cos(phi)*cos(theta)*sin(psi),
                     cos(phi)*cos(psi)*sin(theta), -cos(psi)*sin(phi), -cos(phi)*cos(psi)*cos(theta),
                     sin(phi)*sin(theta),           cos(phi),          -cos(theta)*sin(phi);
        R_dot_theta<< - cos(psi)*sin(theta) - cos(theta)*sin(phi)*sin(psi), 0, cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta),
                       cos(psi)*cos(theta)*sin(phi) - sin(psi)*sin(theta), 0, cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),
                                                      -cos(phi)*cos(theta), 0,                               -cos(phi)*sin(theta);
        R_dot_psi<< - cos(theta)*sin(psi) - cos(psi)*sin(phi)*sin(theta), -cos(phi)*cos(psi), cos(psi)*cos(theta)*sin(phi) - sin(psi)*sin(theta),
                   cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi),
                                                                    0,                  0,                                                  0;

        R_dot.col(0) = R_dot_phi * a;
        R_dot.col(1) = R_dot_theta * a;
        R_dot.col(2) = R_dot_psi * a;

        //cout<<"R_dot"<<endl<<R_dot<<endl;

        Matrix3d G_inv_dot,G_dot_phi,G_dot_theta,G_dot_psi;
        G_dot_phi<<                                                            0, 0,                                                 0,
                 sin(theta) + (sin(phi)*sin(phi)*sin(theta))/(cos(phi)*cos(phi)), 0, - cos(theta) - (cos(theta)*sin(phi)*sin(phi))/(cos(phi)*cos(phi)),
                                      -(sin(phi)*sin(theta))/(cos(phi)*cos(phi)), 0,                  (cos(theta)*sin(phi))/(cos(phi)*cos(phi));
        G_dot_theta<<                    -sin(theta), 0,                     cos(theta),
                      (cos(theta)*sin(phi))/cos(phi), 0, (sin(phi)*sin(theta))/cos(phi),
                                -cos(theta)/cos(phi), 0,           -sin(theta)/cos(phi);
        G_dot_psi<<0,0,0,
                   0,0,0,
                   0,0,0;
        G_inv_dot.col(0) = G_dot_phi * w;
        G_inv_dot.col(1) = G_dot_theta * w;
        G_inv_dot.col(2) = G_dot_psi * w;

        //cout<<"G_inv_dot"<<endl<<G_inv_dot<<endl;
        A.block<3,3>(0,6) = Matrix3d::Identity();
        A.block<3,3>(3,3) = G_inv_dot;
        A.block<3,3>(6,3) = R_dot;
        A.block<3,3>(3,9) = -G.inverse();
        A.block<3,3>(6,12) = -R;
        //cout<<"A"<<endl<<A<<endl;

        MatrixXd U = MatrixXd::Zero(15,12);
        U.block<3,3>(3,0) = -G.inverse();
        U.block<3,3>(6,3) = -R;
        U.block<3,3>(9,6) = MatrixXd::Identity(3,3);
        U.block<3,3>(12,9) = MatrixXd::Identity(3,3);

        //cout<<"U"<<endl<<U<<endl;

        MatrixXd F,V;

        F = (MatrixXd::Identity(15,15) + dt * A);
        V = dt * U;
        P = F * P * F.transpose() + V * Q * V.transpose();
        //cout<<"P"<<endl<<P<<endl;
        t = cur_t;
        //cout<<"x    "<<x.transpose()<<endl;
        //cout<<"P    "<<P.transpose()<<endl;
    }







}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //return;

    if(isInit==false)
        return;
    //your code for update
    //camera position in the IMU frame = (0, -0.05, +0.02)
    //camera orientaion in the IMU frame = Quaternion(0, 0, -1, 0); w x y z, respectively
    double cur_t = msg->header.stamp.toSec();
    cout<<"update time:   "<<endl;
    printf("%f\n",cur_t);
    Matrix3d imu_R_cam = Quaterniond(0,0,-1,0).toRotationMatrix();
    VectorXd imu_T_cam = Vector3d(0,-0.05,0.02);


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
    //cout<<"T"<<endl<<T<<endl;
    //ROS_BREAK();
    Quaterniond Quat;
    Quat = R;
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "my work";
    odom.pose.pose.position.x = T(0);
    odom.pose.pose.position.y = T(1);
    odom.pose.pose.position.z = T(2);
    odom.pose.pose.orientation.w = Quat.w();
    odom.pose.pose.orientation.x = Quat.x();
    odom.pose.pose.orientation.y = Quat.y();
    odom.pose.pose.orientation.z = Quat.z();


    odom_pub_update.publish(odom);


    //cout<<"R"<<endl<<R<<endl;



    double phi = asin(R(2,1));
    double psi = atan2(-R(0,1)/cos(phi),R(1,1)/cos(phi));
    //cout<<"cos(phi)"<<cos(phi)<<endl;
    //cout<<"R(2,0)"<<R(2,0)<<endl;
    //cout<<"R(2,2)"<<R(2,2)<<endl;

    double theta = atan2(-R(2,0)/cos(phi),R(2,2)/cos(phi));
  
    
    //cout<<"phi         theta        psi    "<<endl;
    //cout<<phi<<"    "<<theta<<"    "<<psi<<endl;

    MatrixXd C = MatrixXd::Zero(6,15);
    C.block<3,3>(0,0) = Matrix3d::Identity();
    C.block<3,3>(3,3) = Matrix3d::Identity();
    //cout<<"C"<<endl<<C<<endl;

    MatrixXd K(15,6);
    K = P * C.transpose() * (C * P *C.transpose() + Rt).inverse();
    VectorXd z(6);
    z.segment<3>(0) = T;
    z.segment<3>(3) = Vector3d(phi,theta,psi);
    for(int i=3;i<6;i++)
    {
        if(abs(x(i)-z(i))>3.14)
        {
            //cout<<"x     "<<x(i)<<endl;
            //cout<<"z     "<<z(i)<<endl;
            if(z(i)<0)
                z(i) += 2 * M_PI;
            else
                z(i) -= 2 * M_PI;
            //cout<<"hz    "<<z(i)<<endl;
        }
    }



    x = x + K * (z - C * x);
    P = P - K * C * P;

  
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 100, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 100, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    odom_pub_update = n.advertise<nav_msgs::Odometry>("ekf_odom_update", 100);
    Rcam = Quaterniond(0, 0, -1, 0).toRotationMatrix();
    //cout << "R_cam" << endl << Rcam << endl;
    Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);  // IMU w   a  
    Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6); // IMU   bg   ba
    Rt.topLeftCorner(3, 3) = 0.01 * Rt.topLeftCorner(3, 3);  // Measure position
    Rt.bottomRightCorner(3, 3) = 0.01 * Rt.bottomRightCorner(3, 3); // Measure  orientation
    //cout<<"Q"<<endl<<Q<<endl;
    //cout<<"Rt"<<endl<<Rt<<endl;
    ros::spin();
}



//0 p_x    //world frame
//1 p_y
//2 p_z
//3 q_roll   phi//body frame ----->world frame    WRB
//4 q_pitch  theta
//5 q_yaw    psi
//6 v_x    //world frame
//7 v_y
//8 v_z
//9 bw_x  //body frame   w_bias                   
//10 bw_y
//11 bw_z
//12 ba_x  //body frame  acc_bias
//13 ba_y
//14 ba_z