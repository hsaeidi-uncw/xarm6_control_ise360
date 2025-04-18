#include <ros/ros.h>
#include <xarm6_control_ise360/Plan.h>
#include <geometry_msgs/Twist.h>
#include <xarm_msgs/GripperMove.h>
#include <std_msgs/Bool.h>

// flag for switching states
bool traj_expired = false;

void get_traj_status(const std_msgs::Bool & _data){
	// update the status
	traj_expired = _data.data; 
}

// flag for creating the plan
bool ball_pos_received = false;

float ball_x, ball_y;

float ball_z = 0.18; // a safe offset above the table that includes the gripper length

void get_ball_pos(const geometry_msgs::Twist & _data){
	// update the coordinates
	ball_x = _data.linear.x; 
	ball_y = _data.linear.y;
	// change the flag
	if (!ball_pos_received){	
		std::cout << "Received ball pos! " << std::endl;
		ball_pos_received = true;
	}
}


int main(int argc, char * argv[]){
	ros::init(argc,argv,"simple_planner_with_gripper");
	ros::NodeHandle nh;
	

	//publisher for sending plans to the trajectory generator
    ros::Publisher plan_pub = nh.advertise<xarm6_control_ise360::Plan>("/plan",1);
    // define a service client to connect to the gripper emulator
  	ros::ServiceClient client = nh.serviceClient<xarm_msgs::GripperMove>("/xarm/gripper_move");
	// subscriber for reading the status of the trajectory
    ros::Subscriber traj_status_sub = nh.subscribe("/traj_expired",10,get_traj_status);
    // subscriber for reading the ball coordinates
    ros::Subscriber ball_pos_sub = nh.subscribe("/ball_pos",10,get_ball_pos);
     
	
	// set the loop frequency equal to the camera input
	int loop_freq = 10;
    ros::Rate loop_rate(loop_freq);
	// subplans for each state(step)
    xarm6_control_ise360::Plan approach_plan;
    xarm6_control_ise360::Plan retract_plan;
	geometry_msgs::Twist plan_point;
	
	// for sending commands to the gripper
	xarm_msgs::GripperMove srv;

	bool overall_plan_created = false;
	bool plan_submitted = false;
	
	// a variable for switching states
    char state; // a: for approach, c: for closing the gripper, r: for retract ...

	while(ros::ok()){
		
		if (!overall_plan_created){
			 std::cout << "Plan not ready!!! Waiting for the ball coordinates! " << std::endl; 
			 if(ball_pos_received){
			 	// add some points to the approach phase
				// some dummy points for quick tests
		    	plan_point.linear.x = 0.2;
				plan_point.linear.y = 0.0;
				plan_point.linear.z = 0.11;
				plan_point.angular.x = -3.1415;
				plan_point.angular.y = 0.0011;
				plan_point.angular.z = 0.0002;
				// add this point to the plan
				approach_plan.points.push_back(plan_point);
				
				// some dummy points for quick tests
		    	plan_point.linear.x = ball_x;
				plan_point.linear.y = ball_y;
				plan_point.linear.z = ball_z;
				plan_point.angular.x = -3.1415;
				plan_point.angular.y = 0.0011;
				plan_point.angular.z = 0.0002;
				// add this point to the plan
				approach_plan.points.push_back(plan_point);
				
				/// create the next piece of the plan after we close the gripper
				// this will be the first point after we close the gripper
				retract_plan.points.push_back(plan_point);				
				// some dummy points for quick tests
		    	plan_point.linear.x = ball_x;
				plan_point.linear.y = ball_y;
				plan_point.linear.z = ball_z+0.1;
				plan_point.angular.x = -3.1415;
				plan_point.angular.y = 0.0011;
				plan_point.angular.z = 0.0002;
				// add this point to the plan
				retract_plan.points.push_back(plan_point);	
				
	
				// change the flag
			 	overall_plan_created = true;
			 	std::cout << "Plan CREATED! " << std::endl; 
			 	
			 	
				state = 'a'; //approach phase
	   			std::cout << "Switching to state: APPROACH! " << std::endl;

			 }
		}else{
				if(state == 'a'){
					if(!plan_submitted){		
			   			plan_pub.publish(approach_plan); // publish the message
			   			std::cout << "Approach plan sent! " << std::endl;
			   			plan_submitted = true;
			   			traj_expired = false;
			   		}
			   		if(traj_expired){
						state = 'c';
						plan_submitted = false;
			   			std::cout << "Switching to state: Gripper CLOSE! " << std::endl;
			   		}
		   		}
		   		if(state == 'c'){
			   		srv.request.pulse_pos = 220;// use 850 for openning the gripper
			   		if (client.call(srv)){
    					std::cout << srv.response.message << std::endl; 
    					
    					// add a short pause to complete gripper action before switching the state
						ros::Duration(1.5).sleep();
						// switch to the next state
    					state = 'r';
    					std::cout << "Switching to state: RETRACT! " << std::endl;
  					}else{
					    std::cout << "Failed to call service!\n"; 
					    return 1;
					}
		   		
		   		}
		   		if(state == 'r'){
					if(!plan_submitted){		
			   			plan_pub.publish(retract_plan); // publish the message
			   			std::cout << "Retract plan sent! " << std::endl;
			   			plan_submitted = true;
			   			traj_expired = false;
			   		}
			   		if(traj_expired){
						plan_submitted = false;
						state = 'f';
			   			std::cout << "COMPLETED THIS SIMPLE TASK! " << std::endl;
			   		}
		   		}
		   		
		   		
		}
		
		 
        //
		ros::spinOnce();
        loop_rate.sleep();
	}	
	
    return 0;
}
