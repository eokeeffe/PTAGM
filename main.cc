// Copyright 2009 Isis Innovation Limited
// This is the main extry point for PTAMM
#include <stdlib.h>
#include <iostream>
#include <gvars3/instances.h>
#include "System.h"
#include <GL/glut.h>
/*#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/init.h>
#include <ros/rate.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/raw_subscriber.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <class_loader/multi_library_class_loader.h>*/


//namespace enc = sensor_msgs::image_encodings;


using namespace std;
using namespace GVars3;
using namespace PTAMM;

/*void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  //sensor_msgs::CvBridge bridge;
  try
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    //cvShowImage("view", bridge.imgMsgToCv(msg, "bgr8"));
    try
       {
         System s;
         s.Run(cv_ptr->image);
       }
     catch(CVD::Exceptions::All e)
       {
         cout << endl;
         cout << "!! Failed to run system; got exception. " << endl;
         cout << "   Exception was: " << endl;
         cout << e.what << endl;
       }
    //imshow("view",cv_ptr->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}*/
int main(int argc, char** argv)
{
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
  cout << "  Welcome to PTAMM " << endl;
  cout << "  ---------------- " << endl;
  cout << "  Parallel tracking and multiple mapping" << endl;
  cout << "  Copyright (C) Isis Innovation Limited 2009 " << endl;
  cout << endl;
  cout << "  Parsing settings.cfg ...." << endl;
  GUI.LoadFile("settings.cfg");
  
  GUI.StartParserThread(); // Start parsing of the console input
  atexit(GUI.StopParserThread); 
  
  try
    {
      System s;
      s.Run();
    }
  catch(CVD::Exceptions::All e)
    {
      cout << endl;
      cout << "!! Failed to run system; got exception. " << endl;
      cout << "   Exception was: " << endl;
      cout << e.what << endl;
    }

  return 0;





	   /*ros::init(argc, argv, "image_listener");
	    ros::NodeHandle nh;
	    cvNamedWindow("view");
	    cvStartWindowThread();
	    image_transport::ImageTransport it(nh);

	    image_transport::RawSubscriber sub = it.subscribe("IMAGE_TOPIC", 1, imageCallback);

	    ros::Rate r(100);
	    while(nh.ok())
	    {
	     ros::spinOnce();
	     r.sleep();
	    }

	    cvDestroyWindow("view");*/
}



