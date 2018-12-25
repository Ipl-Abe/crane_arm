#include <ros/ros.h>
#include <ros/wall_timer.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "crane_dh_inverse/dh_inverse.h"

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#define L_B2 0.07137	// base_link〜Shoulder(id2)のLink長さ
#define L_23 0.083	// Shoulder(id2)～Elbow(id3)のLink長さ
#define L_34 0.093	// Elbow(id3)〜Wrist(id4)のLink長さ
#define L_4G 0.08083	// Wrist(id4)〜Gripper_link先のLink長さ
#define PI 3.14159  // 円周率

float initial_x = 0.07;
float initial_y = 0.0;
float initial_z = 0.13;

float pos_x = 0.0;
float pos_y = 0.0;
float pos_z = 0.0;
float wrist_arg = -0.3;

// timer
ros::Timer timer_1_;
ros::Timer timer_2_;

bool flag = false;


// radian
std_msgs::Float64 tilt1_angle;
std_msgs::Float64 tilt2_angle;
std_msgs::Float64 tilt3_angle;
std_msgs::Float64 tilt4_angle;
std_msgs::Float64 tilt5_angle;

std::vector<float> a = {-0.0163, 0.0, 0.083, 0.093, 0.08083};
std::vector<float> alpha = {0.0, -PI/2, 0.0, 0.0, 0.0};
std::vector<float> d = {0.07137, 0.0, 0.0, 0.0, 0.0};
std::vector<float> theta;

// my function
MatrixXf solve_DH(float alpha, float a, float theta, float d);
Matrix4f TransX(float a);
Matrix4f TransZ(float a);
Matrix4f RotX(float theta);
Matrix4f RotZ(float theta);
Vector3f rpyFromRot(Matrix3f R);


MatrixXf getCraneJocobiMat(std::vector<float> theta);
Vector3f solve_craneFK(std::vector<float> alpha, std::vector<float> a, std::vector<float> theta, std::vector<float> d);

Vector3f goal_position = Vector3f(0.08, 0.0, 0.15);
void ShowMatrix(MatrixXf mat);


double deg2rad(double degree)
{
    return degree * M_PI / 180.0f;
};

double rad2deg(double radian)
{
    return radian * 180.0f/ M_PI;
};


bool calc_ik()
{
    MatrixXf J;
    MatrixXf J2;
    MatrixXf JTrans;
    int MAX_TIME = 10000;

    for(int i = 0; i<MAX_TIME; i++){
        Vector3f pos = solve_craneFK(alpha, a, theta, d);
        J = getCraneJocobiMat(theta);
        JTrans = J.transpose();
        Vector3f pref = goal_position;

        //VectorXd Dtheta = 0.05 * J * J*J.transpose() * (pref - pos);
        J2.conservativeResize(J.rows()+1, J.cols() +1);
        J2 = J*J.transpose();
        J2 = J2.inverse();
        JTrans = JTrans * J2;
        VectorXf Dtheta;

        Dtheta = 0.02 * JTrans * (pref - pos);

        for(int i=0; i< LINK_NUM; i++ ){
            theta[i] += Dtheta[i];
            //std::cout << "q [" << i << "] : " << theta[i] << std::endl;
        }

        if(theta[0] < -PI){
            theta[0] = -PI;
        }
        if(PI < theta[0])
        {
            theta[0] = PI;
        }
        if(theta[1] < -PI){
            theta[1] = -PI;
        }
        if(PI < theta[1])
        {
            theta[1] = PI;
        }


        float R_24 = sqrt(pow(pos(0),2)+pow(pos(1),2))-L_4G*cos(wrist_arg);
	    float Z_24 = pos(2) + L_4G*sin(wrist_arg) - L_B2;
        float theta_2 = acos((pow(L_23,2)-pow(L_34,2)+pow(R_24,2)+pow(Z_24,2))/(2*L_23*sqrt(pow(R_24,2)+pow(Z_24,2))));
	    float theta_3 = acos((pow(L_34,2)-pow(L_23,2)+pow(R_24,2)+pow(Z_24,2))/(2*L_34*sqrt(pow(R_24,2)+pow(Z_24,2))));
        float theta_4 = atan2(Z_24, R_24);
        //float theta_3 = acos((pow(L_34,2)-pow(L_23,2)+pow(R_24,2)+pow(Z_24,2))/(2*L_34*sqrt(pow(R_24,2)+pow(Z_24,2))));

        // tilt1_angle.data = 0;
        // tilt2_angle.data = -theta[1] - (PI/2);
        // tilt3_angle.data = theta[2];
        // tilt4_angle.data = theta_4 - theta_3 + wrist_arg;
        theta[3] = theta_4 - theta_3 + wrist_arg;

        Vector3f error;
        error = goal_position - pos;
        error = error.cwiseAbs();
        std::cout << error(0) << " " << error(1) << " "<< error(2) << " " << std::endl;
        if(error(0) < 0.01 &&  error(1) < 0.01 && error(2) < 0.001 ){
            tilt1_angle.data = 0;
            tilt2_angle.data = -theta[1] - (PI/2);
            tilt3_angle.data = theta[2];
            tilt4_angle.data = theta_4 - theta_3 + wrist_arg;
            //theta[3] = tilt4_angle.data;
            return true;
        }
    }
    std::cout << "angle " << std::endl;
    std::cout << "q [" << 0 << "] : " << tilt1_angle.data << std::endl;
    std::cout << "q [" << 1 << "] : " << tilt2_angle.data << std::endl;
    std::cout << "q [" << 2 << "] : " << tilt3_angle.data << std::endl;
    std::cout << "q [" << 3 << "] : " << tilt4_angle.data << std::endl;
    std::cout << "q [" << 4 << "] : " << tilt5_angle.data << std::endl;
    return false;
}


// wrist_arg
void joy_callback(const sensor_msgs::Joy& joy_msg){
    goal_position(0) += 0.001 * joy_msg.axes[1];
    goal_position(2) += 0.001 * joy_msg.axes[5];
    wrist_arg += 0.3 * joy_msg.buttons[1];
    wrist_arg -= 0.3 * joy_msg.buttons[3];
    tilt5_angle.data += 0.01 * joy_msg.buttons[6];
    tilt5_angle.data -= 0.01 * joy_msg.buttons[7];
    tilt1_angle.data = 0.0;
    flag = false;
    //calc_ik();
}

void initialize()
{
    for(int i = 0; i <LINK_NUM_GRIPPER; i++ ) theta.push_back(0.0);
    theta[0] = 0.0;
    theta[1] = -(PI/2) -1.70;
    theta[2] = 1.55;
    theta[3] = 1.0;

    tilt1_angle.data = theta[0];
    tilt2_angle.data = -theta[1] + 1.7;
    tilt3_angle.data = theta[2];
    tilt4_angle.data = theta[3];
    tilt5_angle.data = 0.03;

    Vector3f pos;

    pos = solve_craneFK(alpha, a, theta, d);

    std::cout << "pos : " << pos(0) << " " << pos(1) << " " <<pos(2) << " " << std::endl;

};


int main(int argc, char **argv){
    int seq_num = 0;
    //bool flag = false;
    ros::init(argc, argv, "crane_dh_inverse_pub");
    ros::NodeHandle n;

    ros::Publisher tilt1_cmd = n.advertise<std_msgs::Float64>("tilt1_cmd", 20);
    ros::Publisher tilt2_cmd = n.advertise<std_msgs::Float64>("tilt2_cmd", 20);
    ros::Publisher tilt3_cmd = n.advertise<std_msgs::Float64>("tilt3_cmd", 20);
    ros::Publisher tilt4_cmd = n.advertise<std_msgs::Float64>("tilt4_cmd", 20);
    ros::Publisher tilt5_cmd = n.advertise<std_msgs::Float64>("tilt5_cmd", 20);

    std::cout << "before"<< std::endl;
    initialize();

    ros::Subscriber joy_sub = n.subscribe("joy",20,joy_callback);
    ros::Rate loop_rate(20);

    while(ros::ok()){
    static int count = 0;
    Vector3f pos;
    //Vector3f error;
    //pos = solve_craneFK(alpha, a, theta, d);
    //error = goal_position - pos;
    //error = error.cwiseAbs();

    //  std::cout << "error x : " << error(0) << std::endl;
    //  std::cout << "error y : " << error(1) << std::endl;
    //  std::cout << "error z : " << error(2) << std::endl;
    //std::cout << " in loop " << std::endl;
    //if(0.001 < error(0) || 0.001 < error(1) || 0.001 < error(2)){
    
    
    switch(seq_num){
        case 1:

            goal_position = Vector3f(0.08, 0.0, 0.15);
            seq_num = 2;
            flag = false;
            break;
        case 2:
            goal_position = Vector3f(0.08, 0.0, 0.20);
            seq_num =1;
            flag = false;
            break;
        default:
            break;
    }
    
    
    if(!flag){
        flag = true;
        if(calc_ik()){
            std::cout << "get parameter " << std::endl;
            if(tilt4_angle.data >= -M_PI/2 ||tilt4_angle.data <= M_PI/2){
                tilt4_cmd.publish(tilt4_angle);
            }
	        if(tilt3_angle.data >= -M_PI   ||tilt3_angle.data <= M_PI){
                tilt3_cmd.publish(tilt3_angle);
            }
	        if(tilt2_angle.data >= -M_PI/2 ||tilt2_angle.data <= M_PI/2){
                tilt2_cmd.publish(tilt2_angle);
            }
            tilt1_cmd.publish(tilt1_angle);
            tilt5_cmd.publish(tilt5_angle);
            flag = false;
            if(seq_num == 0) seq_num = 1;
        }
    }
        ros::Duration(1.5).sleep();
        //ROS_INFO("test : %f ", tilt1_angle);
        if (count == 0)
        {
            ros::Duration(2.0).sleep();
            //count++;
        }
        count++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

MatrixXf getCraneJocobiMat(std::vector<float> theta){


    MatrixXf J = MatrixXf::Zero(3,4);

    // 第４リンクまでのヤコビ行列
    // J(0,0) = -(sin(theta[0])*(741*sin(theta[1] + theta[2] + theta[3])
    //           + 930*cos(theta[1] + theta[2]) + 830*cos(theta[1])))/10000;

    // J(0,1) = -(cos(theta[0])*(930*sin(theta[1] + theta[2]) - 741*cos(theta[1]
    //          + theta[2] + theta[3]) + 830*sin(theta[1])))/10000;

    // J(0,2) = (3*cos(theta[0])*(247*cos(theta[1] + theta[2] + theta[3])
    //           - 310*sin(theta[1] + theta[2])))/10000;

    // J(0,3) = (741*cos(theta[1] + theta[2] + theta[3])*cos(theta[0]))/10000;

    // J(1,0) = (cos(theta[0])*(741*sin(theta[1] + theta[2] + theta[3])
    //           + 930*cos(theta[1] + theta[2]) + 830*cos(theta[1])))/10000;

    // J(1,1) = -(sin(theta[0])*(930*sin(theta[1] + theta[2])
    //          - 741*cos(theta[1] + theta[2] + theta[3]) + 830*sin(theta[1])))/10000;

    // J(1,2) = (3*sin(theta[0])*(247*cos(theta[1] + theta[2] + theta[3])
    //          - 310*sin(theta[1] + theta[2])))/10000;

    // J(1,3) = (741*cos(theta[1] + theta[2] + theta[3])*sin(theta[0]))/10000;

    // J(2,0) =  0;

    // J(2,1) = - (741*sin(theta[1] + theta[2] + theta[3]))/10000
    //          -(93*cos(theta[1] + theta[2]))/1000 - (83*cos(theta[1]))/1000;

    // J(2,2) = -(741*sin(theta[1] + theta[2] + theta[3]))/10000 - (93*cos(theta[1] + theta[2]))/1000;

    // J(2,3) = -(741*sin(theta[1] + theta[2] + theta[3]))/10000;




    J(0,0) = (163*sin(theta[0]))/10000 - (83*cos(theta[1])*sin(theta[0]))/1000 + (8083*cos(theta[3])*(sin(theta[0])*sin(theta[1])*sin(theta[2])
             - cos(theta[1])*cos(theta[2])*sin(theta[0])))/100000 + (8083*sin(theta[3])*(cos(theta[1])*sin(theta[0])*sin(theta[2])
             + cos(theta[2])*sin(theta[0])*sin(theta[1])))/100000 + (93*sin(theta[0])*sin(theta[1])*sin(theta[2]))/1000
             - (93*cos(theta[1])*cos(theta[2])*sin(theta[0]))/1000;

    J(0,1) = - (83*cos(theta[0])*sin(theta[1]))/1000 - (8083*cos(theta[3])*(cos(theta[0])*cos(theta[1])*sin(theta[2])
             + cos(theta[0])*cos(theta[2])*sin(theta[1])))/100000 - (8083*sin(theta[3])*(cos(theta[0])*cos(theta[1])*cos(theta[2])
             - cos(theta[0])*sin(theta[1])*sin(theta[2])))/100000 - (93*cos(theta[0])*cos(theta[1])*sin(theta[2]))/1000
             - (93*cos(theta[0])*cos(theta[2])*sin(theta[1]))/1000;

    J(0,2) = - (8083*cos(theta[3])*(cos(theta[0])*cos(theta[1])*sin(theta[2]) + cos(theta[0])*cos(theta[2])*sin(theta[1])))/100000
             - (8083*sin(theta[3])*(cos(theta[0])*cos(theta[1])*cos(theta[2]) - cos(theta[0])*sin(theta[1])*sin(theta[2])))/100000
             - (93*cos(theta[0])*cos(theta[1])*sin(theta[2]))/1000 - (93*cos(theta[0])*cos(theta[2])*sin(theta[1]))/1000;

    J(0,3) = - (8083*cos(theta[3])*(cos(theta[0])*cos(theta[1])*sin(theta[2]) + cos(theta[0])*cos(theta[2])*sin(theta[1])))/100000
             - (8083*sin(theta[3])*(cos(theta[0])*cos(theta[1])*cos(theta[2]) - cos(theta[0])*sin(theta[1])*sin(theta[2])))/100000;

    J(1,0) = (83*cos(theta[0])*cos(theta[1]))/1000 - (163*cos(theta[0]))/10000 + (8083*cos(theta[3])*(cos(theta[0])*cos(theta[1])*cos(theta[2])
             - cos(theta[0])*sin(theta[1])*sin(theta[2])))/100000 - (8083*sin(theta[3])*(cos(theta[0])*cos(theta[1])*sin(theta[2])
             + cos(theta[0])*cos(theta[2])*sin(theta[1])))/100000 + (93*cos(theta[0])*cos(theta[1])*cos(theta[2]))/1000
             - (93*cos(theta[0])*sin(theta[1])*sin(theta[2]))/1000;

    J(1,1) = (8083*sin(theta[3])*(sin(theta[0])*sin(theta[1])*sin(theta[2]) - cos(theta[1])*cos(theta[2])*sin(theta[0])))/100000
             - (8083*cos(theta[3])*(cos(theta[1])*sin(theta[0])*sin(theta[2]) + cos(theta[2])*sin(theta[0])*sin(theta[1])))/100000
             - (83*sin(theta[0])*sin(theta[1]))/1000 - (93*cos(theta[1])*sin(theta[0])*sin(theta[2]))/1000
             - (93*cos(theta[2])*sin(theta[0])*sin(theta[1]))/1000;

    J(1,2) = (8083*sin(theta[3])*(sin(theta[0])*sin(theta[1])*sin(theta[2]) - cos(theta[1])*cos(theta[2])*sin(theta[0])))/100000
             - (8083*cos(theta[3])*(cos(theta[1])*sin(theta[0])*sin(theta[2]) + cos(theta[2])*sin(theta[0])*sin(theta[1])))/100000
             - (93*cos(theta[1])*sin(theta[0])*sin(theta[2]))/1000 - (93*cos(theta[2])*sin(theta[0])*sin(theta[1]))/1000;

    J(1,3) = (8083*sin(theta[3])*(sin(theta[0])*sin(theta[1])*sin(theta[2]) - cos(theta[1])*cos(theta[2])*sin(theta[0])))/100000
             - (8083*cos(theta[3])*(cos(theta[1])*sin(theta[0])*sin(theta[2]) + cos(theta[2])*sin(theta[0])*sin(theta[1])))/100000;

    J(2,0) = 0;

    J(2,1) = (93*sin(theta[1])*sin(theta[2]))/1000 - (93*cos(theta[1])*cos(theta[2]))/1000 - (83*cos(theta[1]))/1000
             - (8083*cos(theta[3])*(cos(theta[1])*cos(theta[2]) - sin(theta[1])*sin(theta[2])))/100000
             + (8083*sin(theta[3])*(cos(theta[1])*sin(theta[2]) + cos(theta[2])*sin(theta[1])))/100000;

    J(2,2) = (93*sin(theta[1])*sin(theta[2]))/1000 - (93*cos(theta[1])*cos(theta[2]))/1000
             - (8083*cos(theta[3])*(cos(theta[1])*cos(theta[2]) - sin(theta[1])*sin(theta[2])))/100000
             + (8083*sin(theta[3])*(cos(theta[1])*sin(theta[2]) + cos(theta[2])*sin(theta[1])))/100000;

    J(2,3) = (8083*sin(theta[3])*(cos(theta[1])*sin(theta[2]) + cos(theta[2])*sin(theta[1])))/100000
             - (8083*cos(theta[3])*(cos(theta[1])*cos(theta[2]) - sin(theta[1])*sin(theta[2])))/100000;

    return J;
}

void ShowMatrix(MatrixXf mat){
    std::cout << "Show the Matrix" << std::endl;
    for(int i=0; i < mat.cols()+1; i++){
        for(int j=0; j < mat.rows()+1; j++){
            std::cout << mat(i, j) << " ";
        }
    std::cout << std::endl;
    }
}

Vector3f solve_craneFK(std::vector<float> alpha, std::vector<float> a, std::vector<float> theta, std::vector<float> d)
{
    MatrixXf mat;
    Matrix4f rotationTrans;
    Vector3f vec;
    mat = solve_DH(alpha[0], a[0], theta[0], d[0]) *
          solve_DH(alpha[1], a[1], theta[1], d[1]) *
          solve_DH(alpha[2], a[2], theta[2], d[2]) *
          solve_DH(alpha[3], a[3], theta[3], d[3]) *
          solve_DH(alpha[4], a[4],        0, d[4]) ;
    rotationTrans = mat;
    rotationTrans = RotX(PI/2)*RotZ(-PI/2)*rotationTrans;
    Matrix3f Rmat;
    Rmat(0,0) = mat(0,0);
    Rmat(0,1) = mat(0,1);
    Rmat(0,2) = mat(0,2);
    Rmat(1,0) = mat(1,0);
    Rmat(1,1) = mat(1,1);
    Rmat(1,2) = mat(1,2);
    Rmat(2,0) = mat(2,0);
    Rmat(2,1) = mat(2,1);
    Rmat(2,2) = mat(2,2);

    Matrix3f Rmat2;
    Rmat2(0,0) = rotationTrans(0,0);
    Rmat2(0,1) = rotationTrans(0,1);
    Rmat2(0,2) = rotationTrans(0,2);
    Rmat2(1,0) = rotationTrans(1,0);
    Rmat2(1,1) = rotationTrans(1,1);
    Rmat2(1,2) = rotationTrans(1,2);
    Rmat2(2,0) = rotationTrans(2,0);
    Rmat2(2,1) = rotationTrans(2,1);
    Rmat2(2,2) = rotationTrans(2,2);

    // std::cout << "Rot : " << Rmat(0,0) << " Pitch : " << Rmat(0,1) << " Yaw : " << Rmat(0,2) << std::endl;
    // std::cout << "Rot : " << Rmat(1,0) << " Pitch : " << Rmat(1,1) << " Yaw : " << Rmat(1,2) << std::endl;
    // std::cout << "Rot : " << Rmat(2,0) << " Pitch : " << Rmat(2,1) << " Yaw : " << Rmat(2,2) << std::endl;


    Vector3f RPYvector,RPYvector2;
    //RPYvector = RPYvector;
    RPYvector = rpyFromRot(Rmat);
    RPYvector2 = rpyFromRot(Rmat2);
    // std::cout << "Roll : " << RPYvector(0) << " Pitch : " << RPYvector(1) << " Yaw : " << RPYvector(2) << std::endl;
    // std::cout << "Roll2 : " << RPYvector2(0) << " Pitch2 : " << RPYvector2(1) << " Yaw2 : " << RPYvector2(2) << std::endl;
    vec(0) = mat(0,3);
    vec(1) = mat(1,3);
    vec(2) = mat(2,3);

    return vec;
}


MatrixXf solve_DH(float alpha, float a, float theta, float d)
{
    MatrixXf mat;
    mat = TransX(a) * RotX(alpha) * TransZ(d) * RotZ(theta);
    return mat;
}


Matrix4f TransX(float a)
{
    Matrix4f mat;
    mat <<
      1, 0, 0, a,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

    return mat;
}
Matrix4f TransZ(float a)
{
    Matrix4f mat;
    mat <<
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, a,
      0, 0, 0, 1;

    return mat;
}
Matrix4f RotX(float theta)
{
    Matrix4f mat;
    mat <<
      1,          0,           0, 0,
      0, cos(theta), -sin(theta), 0,
      0, sin(theta),  cos(theta), 0,
      0,          0,           0, 1;

    return mat;
}
Matrix4f RotZ(float theta)
{
    Matrix4f mat;
    mat <<
      cos(theta), -sin(theta), 0, 0,
      sin(theta),  cos(theta), 0, 0,
               0,           0, 1, 0,
               0,           0, 0, 1;

    return mat;
}

Vector3f rpyFromRot(Matrix3f R)
{
    double roll, pitch, yaw;

    if((fabs(R(0,0)) < fabs(R(2,0))) && (fabs(R(1,0)) < fabs(R(2,0)))) {
        // cos(p) is nearly = 0
        double sp = -R(2,0);
        if (sp < -1.0) {
            sp = -1.0;
        } else if (sp > 1.0) {
            sp = 1.0;
        }
        pitch = asin(sp); // -pi/2< p < pi/2

        roll = atan2(sp * R(0,1) + R(1,2),  // -cp*cp*sr*cy
                     sp * R(0,2) - R(1,1)); // -cp*cp*cr*cy

        if (R(0,0) > 0.0) { // cy > 0
            (roll < 0.0) ? (roll += PI) : (roll -= PI);
        }
        const double sr = sin(roll);
        const double cr = cos(roll);
        if(sp > 0.0){
            yaw = atan2(sr * R(1,1) + cr * R(1,2), //sy*sp
                        sr * R(0,1) + cr * R(0,2));//cy*sp
        } else {
            yaw = atan2(-sr * R(1,1) - cr * R(1,2),
                        -sr * R(0,1) - cr * R(0,2));
        }
    } else {
        yaw = atan2(R(1,0), R(0,0));
        const double sa = sin(yaw);
        const double ca = cos(yaw);
        pitch = atan2(-R(2,0), ca * R(0,0) + sa * R(1,0));
        roll = atan2(sa * R(0,2) - ca * R(1,2), -sa * R(0,1) + ca * R(1,1));
    }
    return Vector3f(roll, pitch, yaw);
}


