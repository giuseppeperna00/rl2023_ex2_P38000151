#include "ros/ros.h" 
#include "ex2_custom_msg/my_msg.h"
#include <math.h>
//#include <ex2_custom_msg/sinu>

using namespace std;


int main(int argc, char **argv) {

	double amplitude;
	double period;

	//Initialize the ROS node with name: ros_publisher
	ros::init(argc, argv,"ros_publisher");

	amplitude = 10.0;
	period = 2.0;
	
	//Declare the node handle: our interface with the ROS system
	ros::NodeHandle nh;

	//parameter definition
	if(!nh.getParam("amplitude", amplitude)) {
		ROS_INFO("Amplitude not defined. Setting default value 10.0");
		amplitude = 10.0;
	}

	if(!nh.getParam("period", period)) {
		ROS_INFO("Period not defined. Setting default value 2.0");
		period = 2.0;
	}

	ros::Publisher topic_pub = nh.advertise<ex2_custom_msg::my_msg>("/signal", 1);

	//Rate object: 10 Hz of rate
	double ros_rate = 10.0;
	ros::Rate rate(ros_rate); 

    //Define the custom datatype
    ex2_custom_msg::my_msg data;
    //Fill the constant information (amplitude and period)
    data.A = amplitude;
	data.T = period;

	double t = 0.0;
	
	while ( ros::ok() ) {

        //Fill the data part
		data.value = amplitude*sin(2*M_PI/period*t);
        //value = n.advertise <ex2_custom_msg::sinu>("sinu/",1);

		//ROS_INFO("%lf",data.value);	//debug 

		//Publish the message over the ROS network
		topic_pub.publish(data);
		
		t += 1/ros_rate;

		rate.sleep();
	}
	
	return 0;
}

