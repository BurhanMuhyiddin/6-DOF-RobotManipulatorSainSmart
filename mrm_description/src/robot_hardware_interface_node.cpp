#include <mrm_description/robot_hardware_interface.h>

MyRobot::MyRobot(ros::NodeHandle& nh) : nh_(nh){
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=4;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);

    client = nh_.serviceClient<mrm_description::Floats_array>("/read_joint_state");

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
        hardware_interface::JointStateHandle joint_state_handle(joint_names_[i], &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(joint_state_handle);

        hardware_interface::JointHandle joint_pos_handle(jnt_state_interface.getHandle(joint_names_[i]), &cmd[0]);
        jnt_pos_interface.registerHandle(joint_pos_handle);
    }

    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);
    
}

void MyRobot::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    //write(elapsed_time_);
}

void MyRobot::read() {

	joint_read.request.req=1.0;
	
	if(client.call(joint_read))
	{
	    
		pos[0]=angles::from_degrees(90-joint_read.response.res[0]);
		pos[1]=angles::from_degrees(joint_read.response.res[1]-90);
		pos[2]=angles::from_degrees(joint_read.response.res[2]-90);
        pos[3]=angles::from_degrees(joint_read.response.res[3]-90);
        pos[4]=angles::from_degrees(joint_read.response.res[4]-90);
		
		ROS_INFO("Receiving  j1: %.2f, j2: %.2f, j3: %.2f",joint_read.response.res[0],joint_read.response.res[1], joint_read.response.res[2]);
		
	}	
    else
    {
    	pos[0]=0;
        pos[1]=0;
        pos[2]=0;
        pos[3]=0;
        pos[4]=0;
        ROS_INFO("Service not found ");
    }   

}

int main(int argc, char** argv)
{
    ROS_INFO("Hello ");
    ros::init(argc, argv, "robot_hardware_interface");
    ros::NodeHandle nh;
    //ros::AsyncSpinner spinner(2);
    //spinner.start();
    MyRobot my_robot(nh);
    ros::spin();

    //while(1);
    /*while (true)
    {
        robot.read();
        cm.update(robot.get_time(), robot.get_period());
        robot.write();
        sleep();
    }*/
}
