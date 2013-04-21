// -*- c++ -*-
// Copyright 2009 Isis Innovation Limited
//
// MapViewer.h
//
// Defines the MapViewer class
//
// This defines a simple map viewer widget, which can draw the 
// current map and the camera/keyframe poses within it.
//
#ifndef __MAP_VIEWER_H
#define __MAP_VIEWER_H

#include "Map.h"
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <sstream>
#include "GLWindow2.h"

namespace PTAMM {

using namespace TooN;

class Map;

class MapViewer
{
  public:
    MapViewer(std::vector<Map*> &maps, Map *map, GLWindow2 &glw);
    void DrawMap(SE3<> se3CamFromWorld, double offset_X, double offset_Y,double offset_Z);
    std::string GetMessageForUser();
    void SwitchMap( Map * map, bool bForce = false );

    void ViewNextMap();
    void ViewPrevMap();
    void ViewCurrentMap();
    Vector<3> GetCurrentXZPosition();
    inline Vector<2> GetPathToARMarker(){ return pathToARMarker; };
    inline double GetYaw() {return yaw;};
    
  protected:
    std::vector<Map*> & mvpMaps;     // Reference to all of the maps
    Map * mpMap, *mpViewingMap;      // the active tracking map, and the map being viewed
    GLWindow2 &mGLWindow;
    SE3<> currPose;
    Vector<2> pathToARMarker;
    double yaw;
    
    void DrawGrid(double offset_X, double offset_Y, double offset_Z);
    void DrawMapDots();
    void DrawCamera(SE3<> se3, bool bSmall=false, Map* sourceMap = NULL);
    void SetupFrustum();
    void SetupModelView(SE3<> se3WorldFromCurrent = SE3<>());
    
    Vector<3> mv3MassCenter;
    SE3<> mse3ViewerFromWorld;

    std::ostringstream mMessageForUser;
    bool mbBrowseMode;                            // Map browsing mode enabled?
    SE3<> newPose;
};

}

#endif
