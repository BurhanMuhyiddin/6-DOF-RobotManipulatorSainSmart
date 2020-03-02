#include <mrm_description/robot_hardware_interface.h>

MyRobot::MyRobot(ros::NodeHandle& nh) : nh_(nh){
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=4;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);

    pub = nh_.advertise<rospy_tutorials::Floats>("/joints_to_aurdino",10);
    //client = nh_.serviceClient<mrm_description::Floats_array>("/read_joint_state");

    non_realtime_loop_ = nh_.createTimer(update_freq, &MyRobot::update, this);
}

void MyRobot::init(){
    num_joints_ = 5;
    joint_names_[0] = "base_waist";
    joint_names_[1] = "waist_forearm";
    joint_names_[2] = "forearm_forearmRotation";
    joint_names_[3] = "forearmRotation_wrist";
    joint_names_[4] = "wrist_wristRotation";

    for (int i = 0; i < num_joints_; i++)
    {
        hardware_interface::JointStateHandle joint_state_handle(joint_names_[i], &pos[i], &vel[i], &eff[i]);
        jnt_state_interface.registerHandle(joint_state_handle);

        hardware_interface::JointHandle joint_pos_handle(jnt_state_interface.getHandle(joint_names_[i]), &cmd[i]);
        jnt_pos_interface.registerHandle(joint_pos_handle);
    }

    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);
    
}

void MyRobot::read(const sensor_msgs::JointState::ConstPtr& msg) {
    double pst0 = msg->position[0];
    double pst1 = msg->position[1];
    double pst2 = msg->position[2];
    double pst3 = msg->position[3];
    double pst4 = msg->position[4];

    pos[0]=angles::from_degrees(pst0-90.0);
    pos[1]=angles::from_degrees(90.0-pst1);
    pos[2]=angles::from_degrees(pst2);
    pos[3]=angles::from_degrees(pst3-90.0);
    pos[4]=angles::from_degrees(90.0-pst4);

    joints_pub.data.clear();
	joints_pub.data.push_back(pst0);
	joints_pub.data.push_back(pst1);
	joints_pub.data.push_back(pst2);
    joints_pub.data.push_back(pst3);
	joints_pub.data.push_back(pst4);
    pub.publish(joints_pub);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_hardware_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    MyRobot my_robot(nh);
    ros::Subscriber sub = nh.subscribe("/joint_state_commands_", 1000, &MyRobot::read, &my_robot);
    ros::waitForShutdown();

    //while(1);
    /*while (true)
    {
        robot.read();
        cm.update(robot.get_time(), robot.get_period());
        robot.write();
        sleep();
    }*/
}
