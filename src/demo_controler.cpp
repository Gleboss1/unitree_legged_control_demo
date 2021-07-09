#include "ros/ros.h"
#include "std_msgs/String.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"

#include <string>
#include <sstream>


double motorAngle[12];
ros::Publisher servo_pub[12];
unitree_legged_msgs::MotorCmd motorCmd[12];

//smoothly turns the motors at targetAngles
void takePosition(double *targetAngles)
{
    int duration = 1000;
    for (int i = 0; i < duration; i++)
    {
        double percent = (double) i / duration;
        for (int m = 0; m < 12; m++) {
            if(!ros::ok()) break;
            motorCmd[m].q = motorAngle[m] * (1 - percent) + targetAngles[m] * percent;

            servo_pub[m].publish(motorCmd[m]);
        }
        usleep(3000);
        ros::spinOnce();
    }
}

void FRhipCallback(const unitree_legged_msgs::MotorState & msg)
{
    motorAngle[0] = msg.q;
}

void FRthighCallback(const unitree_legged_msgs::MotorState & msg)
{
    motorAngle[1] = msg.q;
}

void FRcalfCallback(const unitree_legged_msgs::MotorState & msg)
{
    motorAngle[2] = msg.q;
}

void FLhipCallback(const unitree_legged_msgs::MotorState & msg)
{
    motorAngle[3] = msg.q;
}

void FLthighCallback(const unitree_legged_msgs::MotorState & msg)
{
    motorAngle[4] = msg.q;
}

void FLcalfCallback(const unitree_legged_msgs::MotorState & msg)
{
    motorAngle[5] = msg.q;
}

void RRhipCallback(const unitree_legged_msgs::MotorState & msg) {
    motorAngle[6] = msg.q;
}

void RRthighCallback(const unitree_legged_msgs::MotorState & msg)
{
    motorAngle[7] = msg.q;
}

void RRcalfCallback(const unitree_legged_msgs::MotorState & msg)
{
    motorAngle[8] = msg.q;
}

void RLhipCallback(const unitree_legged_msgs::MotorState & msg)
{
    motorAngle[9] = msg.q;
}

void RLthighCallback(const unitree_legged_msgs::MotorState & msg)
{
    motorAngle[10] = msg.q;
}

void RLcalfCallback(const unitree_legged_msgs::MotorState & msg)
{
    motorAngle[11] = msg.q;
}


int main(int argc, char ** argv) {

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    std::string robot_name = "a1";

    //create subscribers and publisher
    ros::Subscriber servo_sub[12];
    servo_sub[0] = n.subscribe("/" + robot_name + "_gazebo/FR_hip_controller/state", 1, FRhipCallback);
    servo_sub[1] = n.subscribe("/" + robot_name + "_gazebo/FR_thigh_controller/state", 1, FRthighCallback);
    servo_sub[2] = n.subscribe("/" + robot_name + "_gazebo/FR_calf_controller/state", 1, FRcalfCallback);
    servo_sub[3] = n.subscribe("/" + robot_name + "_gazebo/FL_hip_controller/state", 1, FLhipCallback);
    servo_sub[4] = n.subscribe("/" + robot_name + "_gazebo/FL_thigh_controller/state", 1, FLthighCallback);
    servo_sub[5] = n.subscribe("/" + robot_name + "_gazebo/FL_calf_controller/state", 1, FLcalfCallback);
    servo_sub[6] = n.subscribe("/" + robot_name + "_gazebo/RR_hip_controller/state", 1, RRhipCallback);
    servo_sub[7] = n.subscribe("/" + robot_name + "_gazebo/RR_thigh_controller/state", 1, RRthighCallback);
    servo_sub[8] = n.subscribe("/" + robot_name + "_gazebo/RR_calf_controller/state", 1, RRcalfCallback);
    servo_sub[9] = n.subscribe("/" + robot_name + "_gazebo/RL_hip_controller/state", 1, RLhipCallback);
    servo_sub[10] = n.subscribe("/" + robot_name + "_gazebo/RL_thigh_controller/state", 1, RLthighCallback);
    servo_sub[11] = n.subscribe("/" + robot_name + "_gazebo/RL_calf_controller/state", 1, RLcalfCallback);

    servo_pub[0] = n.advertise < unitree_legged_msgs::MotorCmd > ("a1_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = n.advertise < unitree_legged_msgs::MotorCmd > ("a1_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = n.advertise < unitree_legged_msgs::MotorCmd > ("a1_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = n.advertise < unitree_legged_msgs::MotorCmd > ("a1_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = n.advertise < unitree_legged_msgs::MotorCmd > ("a1_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = n.advertise < unitree_legged_msgs::MotorCmd > ("a1_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = n.advertise < unitree_legged_msgs::MotorCmd > ("a1_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = n.advertise < unitree_legged_msgs::MotorCmd > ("a1_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = n.advertise < unitree_legged_msgs::MotorCmd > ("a1_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = n.advertise < unitree_legged_msgs::MotorCmd > ("a1_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = n.advertise < unitree_legged_msgs::MotorCmd > ("a1_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = n.advertise < unitree_legged_msgs::MotorCmd > ("a1_gazebo/RL_calf_controller/command", 1);

    //some default parameters for motors
    for (int i = 0; i < 4; i++) {
        motorCmd[i * 3 + 0].mode = 0x0A;
        motorCmd[i * 3 + 0].Kp = 70;
        motorCmd[i * 3 + 0].dq = 0;
        motorCmd[i * 3 + 0].Kd = 3;
        motorCmd[i * 3 + 0].tau = 3;
        motorCmd[i * 3 + 1].mode = 0x0A;
        motorCmd[i * 3 + 1].Kp = 180;
        motorCmd[i * 3 + 1].dq = 0;
        motorCmd[i * 3 + 1].Kd = 8;
        motorCmd[i * 3 + 1].tau = 3;
        motorCmd[i * 3 + 2].mode = 0x0A;
        motorCmd[i * 3 + 2].Kp = 300;
        motorCmd[i * 3 + 2].dq = 0;
        motorCmd[i * 3 + 2].Kd = 15;
        motorCmd[i * 3 + 2].tau = 3;
    }

    //array of motors angles for different poses
    //sitting
    double pos1[12] = {0.0, 0.67, -1.3,
                       -0.0, 0.67, -1.3,
                       -0.2, 1.2, -2.2,
                       0.2, 1.2, -2.2};

    // front right leg up
    double pos2[12] = {0.0, 0.0, -3,
                      -0.0, 0.67, -1.3,
                      -0.2, 1.2, -2.2,
                      0.2, 1.2, -2.2};

    // front right leg up hip up
    double pos3[12] = {-0.3, 0.0, -3,
                       -0.0, 0.67, -1.3,
                       -0.2, 1.2, -2.2,
                       0.2, 1.2, -2.2};

    // front right leg up hip down
    double pos4[12] = {0.3, 0.0, -3,
                       -0.0, 0.67, -1.3,
                       -0.2, 1.2, -2.2,
                       0.2, 1.2, -2.2};

    // front right leg up hip straight
    double pos5[12] = {0.0, 0.0, -3,
                       -0.0, 0.67, -1.3,
                       -0.2, 1.2, -2.2,
                       0.2, 1.2, -2.2};

    // front right leg up forward
    double pos6[12] = {0.0, -0.5, -2,
                       -0.0, 0.67, -1.3,
                       -0.2, 1.2, -2.2,
                       0.2, 1.2, -2.2};

    //sitting
    double pos7[12] = {0.0, 0.67, -1.3,
                       -0.0, 0.67, -1.3,
                       -0.2, 1.2, -2.2,
                       0.2, 1.2, -2.2};

    //high sitting
    double pos8[12] = {0.0, 1.0, -0.5,
                       -0.0, 1.0, -0.5,
                       -0.2, 1.0, -1.8,
                       0.2, 1.0, -1.8};
    //stay
    double pos9[12] = {0.0, 0.67, -1.3,
                       -0.0, 0.67, -1.3,
                       -0.0, 0.67, -1.3,
                       0.0, 0.67, -1.3};

    takePosition(pos1);
    takePosition(pos2);
    takePosition(pos3);
    takePosition(pos4);
    takePosition(pos5);
    takePosition(pos6);
    takePosition(pos7);
    takePosition(pos8);
    takePosition(pos9);

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
