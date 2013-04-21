/*
 * ROSService.cpp
 *
 *  Created on: Feb 12, 2013
 *      Author: eranda lakshantha
 */

#include "ROSService.h"

ROSService::ROSService() {
	// TODO Auto-generated constructor stub
	x_Value = 0.0;
	y_Value = 0.0;
	status = -1;
	firstRun = true;
	file_name = "/home/eranda/eranda_debug.txt";
}

ROSService::~ROSService() {
	// TODO Auto-generated destructor stub
}


void ROSService::Initialise(double _x,double _y,Vector<3> _start,  PTAMM::MapViewer* mpViewer)
{
	x_Value = _x;
	y_Value = _y;
	startVector = _start;
	mapViewer = mpViewer;
}

/*
 * This is the key method for moving robot a specified distance in a specified direction
 * */
void ROSService::run()
{
	std::stringstream ss;
	std::stringstream ss2;
	bool _status = false;
	std::string command;

	{

		toArMarker = mapViewer->GetPathToARMarker();			//Get the distance to AR marker in PTAGM's coordinate frame

		double vertical_meters = toArMarker[0] * (0.1 / 0.1);  //Calculate the distance in metrics
		double horizontal_meters = toArMarker[1] * (0.1 / 0.1); //we are sliding the robot 0.1m forward

		double theta2 = atan(horizontal_meters / vertical_meters)
				+ mapViewer->GetYaw();
		vertical_meters = vertical_meters - 0.225;				//deduct some amount as we drive the robot forward at the initialization.
																//so the AR marker always has a (original distance - distance at initialization)
																//distance

		double distance = sqrt(
				(vertical_meters * vertical_meters)
						+ (horizontal_meters * horizontal_meters));

		double horizontal_meters2 = 0;							//For eddie robot
																//this is the second parameter and we set it to 0 since we supply
																//the full distance in first parameter.

		//Supply the aruguments

		ss2.str("");
		ss2 << distance;										//total distance
		ss2 << " ";
		ss2 << horizontal_meters2;								//0
		ss2 << " ";
		ss2 << theta2;											//angle with regard to the robot's initial looking direction

		command = "rosrun jimmy jimmy " + ss2.str();			//the target ros node is called jimmy

		ss << "Vertical Distance: ";
		ss << vertical_meters;
		ss << std::endl;
		ss << "Horizontal Distance: ";
		ss << horizontal_meters;
		ss << std::endl;
		ss << "Distance: ";
		ss << distance;
		ss << std::endl;
		ss << "Theta : ";
		ss << theta2;
		ss << std::endl;
		DebugWriteToFile(ss.str()); 							//For debugging purposes only
		ss.str("");

		_status = system(command.c_str()); 						//Drive the robot. This function must return. returns 1 if successful

	}

}
/*
 * Method for dumping debug info into a file. the file name is set at the constructor
 * */
void ROSService::DebugWriteToFile(std::string message)
{
	std::ofstream fs(file_name,std::ios_base::app);
	if(!fs)
	{
		std::cerr<<"Failed to dump file. Exiting...";
		exit(0);
	}
	fs<<message;
	fs.close();
}

