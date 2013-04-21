/*
 * ROSService.h
 *
 *  Created on: Feb 12, 2013
 *      Author: eranda
 */

#ifndef ROSSERVICE_H_
#define ROSSERVICE_H_
#include <iostream>
#include <TooN/se3.h>
#include <cvd/thread.h>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include "MapViewer.h"

using namespace TooN;

class ROSService  : public CVD::Thread
{
public:
	ROSService();
	virtual ~ROSService();
	//bool Drive();
	void Initialise(double x,double y,Vector<3> _start, PTAMM::MapViewer* mpViewer);
	void DebugWriteToFile(std::string message);
	virtual void run();
	inline int GetStatus() {return status;}

protected:
	double h_Value;
	double x_Value;
	double y_Value;
	int status;
	int num_x;
	Vector<3> startVector;
	Vector<3> end;
	Vector<2> toArMarker;
	double diff;
	PTAMM::MapViewer* mapViewer;
	bool firstRun;
	char* file_name;


private:

};

#endif /* ROSSERVICE_H_ */
