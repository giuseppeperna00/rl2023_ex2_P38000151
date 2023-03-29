#include "ros/ros.h"
#include "ex2_custom_msg/my_msg.h"
#include "std_msgs/Float64.h"
#include "boost/thread.hpp"

using namespace std;

//Use a class to store the topic data 
class ROS_SUB {
	public:
		ROS_SUB();
		ROS_SUB(const double &a);
		void topic_cb(const ex2_custom_msg::my_msg::ConstPtr data);
		void filter();
		ros::NodeHandle get_node() {return _nh;}
		bool set_a(const double &a);
	
	private:
		ros::NodeHandle _nh;
		//Subscriber object
		ros::Subscriber _topic_sub;
		ros::Publisher _filter_pub;
		double _unf_value;
		double _a;
};

ROS_SUB::ROS_SUB() {
	//link subscriber to topic containing the generated signal
	_topic_sub = _nh.subscribe("/signal", 1, &ROS_SUB::topic_cb, this);
	//link publisher to topic containing the filtered signal
	_filter_pub = _nh.advertise<std_msgs::Float64>("/filtered_signal",10);

	//create filter thread
	boost::thread filter_t (&ROS_SUB::filter, this);
	_a = 0.5;
}

ROS_SUB::ROS_SUB(const double &a) {
	_topic_sub = _nh.subscribe("/signal", 1, &ROS_SUB::topic_cb, this);
	_filter_pub = _nh.advertise<std_msgs::Float64>("/filtered_signal",10);

	boost::thread filter_t (&ROS_SUB::filter, this);
	if(a <1 && a >= 0) _a = a;
	else  {
		ROS_ERROR("a value for filter not allowed, setting default value");
		_a = 0.5;
	}
}

bool ROS_SUB::set_a(const double &a) {
	if(a <1 && a >= 0) {_a = a; return true;}
	else return false;
}

//Callback function
void ROS_SUB::topic_cb(const ex2_custom_msg::my_msg::ConstPtr data) {
	//ROS_INFO("Listener: %lf", data->value);	//debug
	_unf_value = data->value;
}

void ROS_SUB::filter() {
	ROS_INFO("The filter thread is running...");
	ros::Rate rate(10);
	std_msgs::Float64 msg;
	
	while(ros::ok()) {
		//leaky integrator : simple first order filter 
		//0 <= a < 1 the higher this parameter the "smoother" the signal
		msg.data = _a*msg.data + (1 - _a)*_unf_value;
		_filter_pub.publish(msg);
		
		rate.sleep();

	}
}


int main( int argc, char** argv ) {

	//Init the ros node with ros_subscriber name
	ros::init(argc, argv, "ros_subscriber");

	double a;

	//Create the ROS_SUB class object
	ROS_SUB rs;


	if(!rs.get_node().getParam("a", a)) {
		ROS_INFO("a not defined for the filter. Setting default value 0.5");
		a = 0.5;
	}


	if(!rs.set_a(a)) {
		ROS_ERROR("a parameter for the filter not valid. Setting default value 0.5");
		rs.set_a(0.5); //this is redundant since the default value is already set in the constructor but
		// it is done for consistency
	}
	
	ros::spin(); 

	//----This function will be never overcome

	return 0;
}