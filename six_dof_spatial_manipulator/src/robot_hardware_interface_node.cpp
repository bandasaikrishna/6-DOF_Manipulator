#include <six_dof_spatial_manipulator/robot_hardware_interface.h>


ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=20;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
	pub = nh_.advertise<rospy_tutorials::Floats>("/joints_to_aurdino",10);
	
    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

void ROBOTHardwareInterface::init() {
    
    num_joints_=6;
	joint_names_[0]="joint1";	
	joint_names_[1]="joint2";
	joint_names_[2]="joint3";
	joint_names_[3]="joint4";	
	joint_names_[4]="joint5";
	joint_names_[5]="joint6";
	

    for (int i = 0; i < num_joints_; ++i) {

         // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create position joint interface
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
        
        position_joint_interface_.registerHandle(jointPositionHandle);
   
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    joint_position_command_[0]=0;
    joint_position_command_[1]=0;
    joint_position_command_[2]=0;
    joint_position_command_[3]=0;
    joint_position_command_[4]=0;
    joint_position_command_[5]=0;
}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ROBOTHardwareInterface::read() {

	//joint_read.request.req=1.0;
	
	joint_position_[0]= joint_position_command_[0];
	joint_position_[1]= joint_position_command_[1];
	joint_position_[2]= (joint_position_command_[2]);
	joint_position_[3]= joint_position_command_[3];
	joint_position_[4]= joint_position_command_[4];
	joint_position_[5]= joint_position_command_[5];
	
        

}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
    
	joints_pub.data.clear();
	joints_pub.data.push_back(90+(angles::to_degrees(joint_position_command_[0])));
	joints_pub.data.push_back(90+(angles::to_degrees(joint_position_command_[1])));
	joints_pub.data.push_back(180+(angles::to_degrees(joint_position_command_[2])));
	joints_pub.data.push_back((angles::to_degrees(joint_position_command_[3])));
	joints_pub.data.push_back(90-(angles::to_degrees(joint_position_command_[4])));
	joints_pub.data.push_back(90+(angles::to_degrees(joint_position_command_[5])));
	//ROS_INFO("Publishing j1: %.2f, j2: %.2f, j3: %.2f",joints_pub.data[0],joints_pub.data[1],joints_pub.data[2]);
	pub.publish(joints_pub);
		
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "six_robot_hardware_interface");
    ros::NodeHandle nh;
    //ros::AsyncSpinner spinner(1); 
    ros::MultiThreadedSpinner spinner(4);// 2 threads for controller service and for the Service client used to get the feedback from ardiuno
    ROBOTHardwareInterface ROBOT(nh);
    //spinner.start();
    spinner.spin();
    //ros::spin();
    return 0;
}
