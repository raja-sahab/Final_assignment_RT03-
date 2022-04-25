// Run with: roslaunch final_assignment assignment.launch

/* LIBRARIES */
#include "ros/ros.h"
#include "algorithm"
#include "cmath"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "final_assignment/Command.h"

/* FUNCTION HEADERS */
void functionCallback ( const sensor_msgs::LaserScan::ConstPtr& msg );
void scanSectors( float * ranges, float * sectors );
int logic( float * sectors );
void integral_logic( float * ranges );
double integral( float * values, int start, int end );
void drive( float straight, float turn );
bool server_response( final_assignment::Command::Request &req, final_assignment::Command::Response &res );

/* GLOBAL VARIABLES */
bool is_active = false;                       // Variable to turn on and off the node.
float d_br;                                   // Alert distance for avoiding obstacles, distance break.
float speed;								  // Linear speed of the robot.
float turn;                                   // Angular speed of the robot.
int nsect = 9; 								  // Number of sectors.
int front = std::floor( nsect / 2 );          // Index of the frontal sector.
int sector_nelem = std::floor( 720/nsect );   // Number of laser surveys per sector.
ros::Publisher pub;                           // Publisher on cmd_vel.

/* FUNCTIONS */
void functionCallback ( const sensor_msgs::LaserScan::ConstPtr& msg ) 
{
	/*
	Function callback for the base_scan subscriber, it is executed each time something is published 
	in this specific topic.
	*/

	if ( is_active == true ) 
	{
		/* Preallocates variables. */
		float sectors[nsect];             // Closest distance from obstacle in each sector.
		float ranges[720];                // Copy of the laser_scan ranges.

		std::fill_n(sectors, nsect, 10);  // Initializes the values in the sectors float array to 10.

		/* Copies the ranges values in the base_scan message into an array. */
		for (int i = 0; i < 720; i++) 
		{
			ranges[i] = msg -> ranges[i];
		}

		/* Calls the function scanSector to fill the sectors array. */
		scanSectors( ranges, sectors );

		/* Call the function logic to decide what to do. If it can not take a decision, then call the function
	   	integral_logic. */
		if ( !logic( sectors ) ) 
		{

			integral_logic( ranges );
		}
	}
}

void scanSectors( float * ranges, float * sectors ) 
{
    /*
    Function to search for the closest obstacle in each sector.
    */
	int i=1;
	int j=0;
	while(i<=nsect)
	{           // For all sectors.
    	while(j<sector_nelem) 
		{  // For all elements in each sector.

    		/* If it finds a closest obstacle, than update the sectors array. */
    		if ( sectors[i]> ranges[i*sector_nelem+j]) 
			{
    		    sectors[i] = ranges[i*sector_nelem+j];
    		}
			j++;

    	}
		i++;
    }
}

int logic( float * sectors ) 
{
	/*
    Function that rapresents the robot's behaviour. It makes choices based on the values into the sectors array.

    Returns: 1 choice made
    		 0 choice not made
    */

	if ( d_br < sectors[front] )
	 { // The frontal sector is obstacle-free.

		/* Searchs in the front-right and in the front-left sectors if there are obstacles, to line up with
		   the track. */
		if ( (sectors[front+1] <= d_br*.8) && (d_br<= sectors[front-1] ) )
		 {
			ROS_INFO("dist: %.2f, speed: %.2f, free road, turn right", sectors[front], speed);
            drive( speed, turn - 0.4 );

		} else if ( (sectors[front-1] <= d_br * .8 ) && (sectors[front+1] >= d_br) ) 
		{
			ROS_INFO("dist: %.2f, speed: %.2f, free road, turn left", sectors[front], speed);
            drive( speed, turn + 0.4 );

		} else 
		{
			ROS_INFO("dist: %.2f, speed: %.2f, free road", sectors[front], speed);
		    drive( speed, turn );
		}

		return 1;

	} 
	else 
	{ // There is an obstacle in the frontal sector.

		/* Searchs if there is an obstacle-free sector. */
		int k=1;
		while (k <= (front-1 )) 
		{ // Looks in all sectors without the frontal one.

		    if ( (sectors[front+k] >= d_br ) && ( sectors[front+k] >= sectors[front-k] ) )
			{ // First looks left.
		    	ROS_INFO("dist: %.2f, speed: %.2f, obstacle , turn left", sectors[front], 0.2);
		        drive( 0.1, 0.5 );
		        return 1;

		    } 
			else if ( ( sectors[front-k] >= d_br ) && ( sectors[front-k] >= sectors[front+k] ) ) 
			{ // Then looks right.
		        drive( 0.1, -0.5 );
		        ROS_INFO("dist: %.2f, speed: %.2f, obstacle , turn right", sectors[front], 0.2);
		        return 1;
		    }
			k++;
		}
	}
	/* If there is not one obstacle-free sector, then it can not make any choice.*/
	return 0;
}

void integral_logic( float * ranges ) 
{
	/*
    Function to decide where to go when there are obstales all around the robot.
    The choice made is based on the free area swept by the laser scan.
    */

	double right_area = integral( ranges, 0, 360 );   // Right-side free area.
	double left_area = integral( ranges, 360, 720 );  // Left-side free area.

		if ( right_area > left_area )
		{
			ROS_INFO("area: %.2f, speed: %.2f, OBSTACLE , turn right", right_area, 0.2);
			drive( 0, -1 );

		} 
		else 
		{
			ROS_INFO("area: %.2f, speed: %.2f, OBSTACLE , turn left", left_area, 0.2);
			drive( 0, 1 );

		}
}

double integral( float * values, int start, int end ) 
{
	/*
    Function to perform a discrete integral with the trapezium method.
    */

	double result = 0;
	int l=0;
	while ( l < end) 
	{
		result = result + ( values[l] + values[l+1]) / 2;
		l++;
	}
	return result;
}

void drive( float straight, float turn ) 
{
	/*
    Function to drive the robot, filling the geometry message and publishing it on the topic cmd_vel.
    */

	geometry_msgs::Twist my_vel;
	my_vel.linear.x = straight;
	my_vel.angular.z = turn;
	pub.publish(my_vel);
}

bool server_response( final_assignment::Command::Request &req, final_assignment::Command::Response &res ) {
	/*
    Function callback to the command service, it sets the speed of the motor depending on the message 
    received and replies with the updated velocity.
    */

	if ( req.command == 's' && speed >= 0.1 ) 
	{
		speed = speed - 0.1;

	} 
	else if ( req.command == 'w' ) 
	{
		speed = speed + 0.1;

	} 
	else if ( req.command == 'x' ) 
	{
		speed = 0.0;

	} 
	else if ( req.command == 'z' ) 
	{
		turn = 0.0;

	}
	else if ( req.command == 'd' ) 
	{
		turn = turn - 0.1;

	} 
	else if ( req.command == 'a' ) 
	{
		turn = turn + 0.1;

	} 
	else if (req.command == '0') 
	{
		is_active = true;

	} 
	else if (req.command == '1') 
	{
		is_active = false;
	}

	/* The distance break depends on the velocity, when the robot has an higer speed it increases the d_br variable. */
	d_br = 1.2 + speed/18;

	res.linear = speed;
	res.angular = turn;

	return true;
}

/* MAIN */
int main ( int argc, char ** argv ) 
{
  	
  	/* Init the ros node. */
	ros::init(argc, argv, "driver_assistance");
	ros::NodeHandle nh;

	/* Initialises the values of the global variables speed and d_br. */
	speed = 0;
	d_br = 1.2;

	/* Subscribes to the topic base_scan to receive informations about the distance from obstacles from the 
	   laser scanner on the robot. */
	ros::Subscriber sub = nh.subscribe("/base_scan", 1, functionCallback);

	/* Creates a service for the keyboard_pilot_node to control the speed of the robot, to reset the position, 
	   and to exit from the simulation. */
	ros::ServiceServer service = nh.advertiseService("/command", server_response);

	/* Creates a publisher to the cmd_vel topic, to guide the robot in real-time controlling its velocity. */
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	ros::spin();

	return 0;
}