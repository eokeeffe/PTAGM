// Copyright 2009 Isis Innovation Limited
#define GL_GLEXT_PROTOTYPES 1
#include "ARDriver.h"
#include "Map.h"
#include "Games.h"
#include <GL/glut.h>
#include <math.h>
#include <gvars3/instances.h>
#include <gvars3/GStringUtil.h>

#include <cvd/image_io.h>

namespace PTAMM {

using namespace GVars3;
using namespace CVD;
using namespace std;

static bool CheckFramebufferStatus();

/**
 * Constructor
 * @param cam Reference to the camera
 * @param irFrameSize the size of the frame
 * @param glw the glwindow
 * @param map the current map
 */
ARDriver::ARDriver(const ATANCamera &cam, ImageRef irFrameSize, GLWindow2 &glw, Map &map, MapViewer& mapviewer,ROSService& _driver)
  :mCamera(cam), mGLWindow(glw), mpMap( &map ), mapViewer(&mapviewer), driver(_driver)
{
  GUI.RegisterCommand("drive", GUICommandCallBack, this);
  GUI.RegisterCommand("plusx", GUICommandCallBack, this);
  GUI.RegisterCommand("minusx", GUICommandCallBack, this);
  GUI.RegisterCommand("plusz",GUICommandCallBack, this);
  GUI.RegisterCommand("minusz",GUICommandCallBack, this);
  GUI.RegisterCommand("plusy",GUICommandCallBack, this);
  GUI.RegisterCommand("minusy",GUICommandCallBack, this);
  mirFrameSize = irFrameSize;
  mCamera.SetImageSize(mirFrameSize);
  mbInitialised = false;
  offset_x = 0;
  offset_z = 0;
  offset_y = 0;
}


/**
 * Initialize the AR driver
 */
void ARDriver::Init()
{
  mbInitialised = true;
  mirFBSize = GV3::get<ImageRef>("ARDriver.FrameBufferSize", ImageRef(1200,900), SILENT);
  glGenTextures(1, &mnFrameTex);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB,mnFrameTex);
  glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0,
	       GL_RGBA, mirFrameSize.x, mirFrameSize.y, 0,
	       GL_RGBA, GL_UNSIGNED_BYTE, NULL);
  MakeFrameBuffer();

  try {
    CVD::img_load(mLostOverlay, "ARData/Overlays/searching.png");
  }
  catch(CVD::Exceptions::All err) {
    cerr << "Failed to load searching image " << "\"ARData/Overlays/searching.png\"" << ": " << err.what << endl;
  }  
  
}


/**
 * Reset the game and the frame counter
 */
void ARDriver::Reset()
{
  if(mpMap->pGame) {
    mpMap->pGame->Reset();
  }

  mnCounter = 0;
}


/**
 * Render the AR composite image
 * @param imFrame The camera frame
 * @param se3CfromW The camera position
 * @param bLost Is the camera lost
 */
/*void ARDriver::Render(Image<Rgb<CVD::byte> > &imFrame, SE3<> se3CfromW, bool bLost)
{
  if(!mbInitialised)
  {
    Init();
    Reset();
  };

  mse3CfromW = se3CfromW;
  mnCounter++;

  // Upload the image to our frame texture
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, mnFrameTex);
  glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB,
		  0, 0, 0,
		  mirFrameSize.x, mirFrameSize.y,
		  GL_RGB,
		  GL_UNSIGNED_BYTE,
		  imFrame.data());

  // Set up rendering to go the FBO, draw undistorted video frame into BG
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,mnFrameBuffer);
  CheckFramebufferStatus();
  glViewport(0,0,mirFBSize.x,mirFBSize.y);
  DrawFBBackGround();
  glClearDepth(1);
  glClear(GL_DEPTH_BUFFER_BIT);

  // Set up 3D projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  //only draw 3d stuff if not lost.
  if(!bLost)
  {
    glMultMatrix(mCamera.MakeUFBLinearFrustumMatrix(0.005, 100));
    glMultMatrix(se3CfromW);

    DrawFadingGrid();

    if(mpMap->pGame) {
      mpMap->pGame->Draw3D( mGLWindow, *mpMap, se3CfromW);
    }
  }

  glDisable(GL_DEPTH_TEST);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_BLEND);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // Set up for drawing 2D stuff:
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);

  DrawDistortedFB();
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  mGLWindow.SetupViewport();
  mGLWindow.SetupVideoOrtho();
  mGLWindow.SetupVideoRasterPosAndZoom();

  
  //2d drawing
  if(!bLost)
  {   
    if(mpMap->pGame) {
      mpMap->pGame->Draw2D(mGLWindow, *mpMap);
    }
  }
  else
  {
    //draw the lost ar overlays
    glEnable(GL_BLEND);
    glRasterPos2i( ( mGLWindow.size().x - mLostOverlay.size().x )/2,
                   ( mGLWindow.size().y - mLostOverlay.size().y )/2 );
    glDrawPixels(mLostOverlay);
    glDisable(GL_BLEND);
  }

}*/
void ARDriver::Render(Image<CVD::Rgb<CVD::byte> > imFrame, SE3<> se3CfromW,  bool bLost)
{

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	mnCounter++;

	 // Upload the image to our frame texture
	 glBindTexture(GL_TEXTURE_RECTANGLE_ARB, mnFrameTex);
	 glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB,
				  0, 0, 0,
				  mirFrameSize.x, mirFrameSize.y,
				  GL_RGB,
				  GL_UNSIGNED_BYTE,
				  imFrame.data());

	 //std::cout<<"AR Driver Render entareed"<<std::endl;
		  // Set up rendering to go the FBO, draw undistorted video frame into BG
	 glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,mnFrameBuffer);
	 CheckFramebufferStatus();

	 //glViewport(0,0,mirFBSize.x, mirFBSize.y);
	 DrawFBBackGround();

	 glClearDepth(1.0);
	 glClear(GL_DEPTH_BUFFER_BIT);

	 glMatrixMode(GL_PROJECTION);
	 glLoadIdentity();
	 glMultMatrix(mCamera.MakeUFBLinearFrustumMatrix(0.005, 100));
	 //glMultMatrix(se3CfromW);

	 glClearDepth( 1.0 );
	 glClear(GL_DEPTH_BUFFER_BIT);
	 glEnable(GL_DEPTH_TEST);
	 glDepthFunc(GL_LEQUAL);

	 //DrawFadingGrid(se3CfromW);
//---------------------------------------------------------------------
	 if(!bLost)
	 {
		 drawMyMarker(se3CfromW); // This should be called when the map quality is good
	 }
//----------------------------------------------------------------------

	 //glMatrixMode(GL_MODELVIEW);
	 //glTranslatef( 0.0, 0.0, -30.0 );
	 //glutSolidCube(30.0);


	 glDisable( GL_DEPTH_TEST );
	 glMatrixMode(GL_MODELVIEW);
	 glLoadIdentity();

	 // Set up for drawing 2D stuff:
	 glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);

	 DrawDistortedFB();

	 glMatrixMode(GL_MODELVIEW);
	 glLoadIdentity();

	 //mGLWindow.SetupViewport();
	 mGLWindow.SetupVideoOrtho();
	 mGLWindow.SetupVideoRasterPosAndZoom();
}

void ARDriver::drawMyMarker(SE3<> pose)
{
	glMatrixMode(GL_MODELVIEW);			//Set the matrix mode Model View
	glLoadIdentity();					//set the Identity matrix as the top most matrix on stack.
	//glMultMatrix(pose);

	TooN::Vector<3> trans = pose.get_translation();
	Matrix<3> _mat = pose.get_rotation().get_matrix();
	double paras[16];

	argConvGlpara(trans,_mat,paras);
	glLoadMatrixd(paras);

	//------------------------------------------------------------
			//The following code block rotates the marker to give a illusion that it rotates around z axis when the camera moves
			//I commented this to reduce jitter

		/*TooN::Vector<3> v;
		TooN::Vector<3> vx;

		TooN::Vector<3> unit_z;
		TooN::Vector<3> unit_x;

			v[0] = _mat[0][2];
			v[1] = _mat[1][2];
			v[2] = _mat[2][2];

			vx[0] = _mat[0][0];
			vx[1] = _mat[1][0];
			vx[2] = _mat[2][0];

			unit_z[0] = 0;
			unit_z[1] = 0;
			unit_z[2] = 1;

			unit_x[0] = 1;
			unit_x[1] = 0;
			unit_x[2] = 0;

			float alpha = acos((v*unit_z)) * 180.0f / (float)M_PI; // Convert to degrees..
			float beta = acos((vx*unit_x)) * 180.0f / (float)M_PI;


			glRotatef(alpha,0.0f,1.0f,0.0f); // Use this line if you want to rotate the marker around y-axis.
			glRotatef(beta,1.0f,0.0f,0.0f);*/
			//------------------------------------------------------------

	/*glBegin(GL_TRIANGLES);

	glColor3f(1,0,0);
	glVertex2f(-0.5 + offset_x,-1.0 + offset_y);
	glColor3f(0,1,0);
	glVertex2f(0.5+ offset_x,-1.0 + offset_y);
	glColor3f(0,0,1);
	glVertex2f(0.0+ offset_x,0.0 + offset_y);

	glEnd();*/
	GLfloat   mat_ambient[]     = {0.0, 0.0, 1.0, 1.0};
	    GLfloat   mat_flash[]       = {0.0, 0.0, 1.0, 1.0};
	    GLfloat   mat_flash_shiny[] = {50.0};
	    GLfloat   light_position[]  = {100.0,-200.0,200.0,0.0};
	    GLfloat   ambi[]            = {0.1, 0.1, 0.1, 0.1};
	    GLfloat   lightZeroColor[]  = {0.9, 0.9, 0.9, 0.1};
	glEnable(GL_LIGHTING);
	    glEnable(GL_LIGHT0);
	    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	    glLightfv(GL_LIGHT0, GL_AMBIENT, ambi);
	    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor);
	    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_flash);
	    glMaterialfv(GL_FRONT, GL_SHININESS, mat_flash_shiny);
	    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	    glTranslatef(offset_x,offset_y,offset_z);
	glutSolidSphere(0.025,30,30);
	glDisable( GL_LIGHTING );
	glDisable( GL_DEPTH_TEST );

	glBegin(GL_POINTS);
	glColor3f(1,0,0);
	glVertex2f(0.0,0.0);
	glEnd();

	/*glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glLineWidth(3);
	glBegin(GL_LINES);
	glVertex3f(0.0,0.0,0.0);		//Draw X-axis It's red colored
	glVertex3f(5.0,0.0,0.0);
	glColor3f(0,1,0);
	glVertex3f(0.0,0.0,0.0);		//Draw Y-axis It's green colored
	glVertex3f(0.0,5.0,0.0);
	glColor3f(0,0,1);
	glVertex3f(0.0,0.0,0.0);		//Draw Z-axis It's blue colored
	glVertex3f(0.0,0.0,5.0);
	glEnd();*/

	glLoadIdentity();


}

void ARDriver::argConvGlpara( TooN::Vector<3> _trans, Matrix<3> rota,double gl_para[16] )
{
    /*int     i, j;

    for( j = 0; j < 3; j++ ) {
        for( i = 0; i < 4; i++ ) {
            gl_para[i*4+j] = para[j][i];
        }
    }*/
    //double* trans_pointer = _trans.ptr<double>();

	gl_para[0*4+3] = gl_para[1*4+3] = gl_para[2*4+3] = 0.0;
    gl_para[3*4+3] = 1.0;

    gl_para[0*4+0] = rota[0][0];//1;//
    gl_para[0*4+1] = rota[1][0];//0;//
    gl_para[0*4+2] = rota[2][0];//0;//

    gl_para[1*4+0] = rota[0][1];//0;//
	gl_para[1*4+1]= rota[1][1];//1;//
	gl_para[1*4+2] =rota[2][1];//0;//

    gl_para[2*4+0] = rota[0][2];//0;//
	gl_para[2*4+1]= rota[1][2];//0;//
	gl_para[2*4+2] =rota[2][2];//1;//

    gl_para[3*4+0] = _trans[0];
    gl_para[3*4+1] = _trans[1];
    gl_para[3*4+2] = _trans[2];//_trans[2];


}

/**
 * Make the frame buffer
 */
void ARDriver::MakeFrameBuffer()
{
  // Needs nvidia drivers >= 97.46
  cout << "  ARDriver: Creating FBO... ";

  glGenTextures(1, &mnFrameBufferTex);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB,mnFrameBufferTex);
  glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0,
	       GL_RGBA, mirFBSize.x, mirFBSize.y, 0,
	       GL_RGBA, GL_UNSIGNED_BYTE, NULL);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  GLuint DepthBuffer;
  glGenRenderbuffersEXT(1, &DepthBuffer);
  glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, DepthBuffer);
  glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, mirFBSize.x, mirFBSize.y);

  glGenFramebuffersEXT(1, &mnFrameBuffer);
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, mnFrameBuffer);
  glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT,
			    GL_TEXTURE_RECTANGLE_ARB, mnFrameBufferTex, 0);
  glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
  			       GL_RENDERBUFFER_EXT, DepthBuffer);

  CheckFramebufferStatus();
  cout << " .. created FBO." << endl;
  glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}


/**
 * check the status of the frame buffer
 */
static bool CheckFramebufferStatus()
{
  GLenum n;
  n = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
  if(n == GL_FRAMEBUFFER_COMPLETE_EXT)
    return true; // All good
  
  cout << "glCheckFrameBufferStatusExt returned an error." << endl;
  return false;
}


/**
 * Draw the background (the image from the camera)
 */
void ARDriver::DrawFBBackGround()
{
  static bool bFirstRun = true;
  static GLuint nList;
  mGLWindow.SetupUnitOrtho();

  glEnable(GL_TEXTURE_RECTANGLE_ARB);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, mnFrameTex);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glDisable(GL_POLYGON_SMOOTH);
  glDisable(GL_BLEND);
  // Cache the cpu-intesive projections in a display list..
  if(bFirstRun)
    {
      bFirstRun = false;
      nList = glGenLists(1);
      glNewList(nList, GL_COMPILE_AND_EXECUTE);
      glColor3f(1,1,1);
      // How many grid divisions in the x and y directions to use?
      int nStepsX = 24; // Pretty arbitrary..
      int nStepsY = (int) (nStepsX * ((double) mirFrameSize.x / mirFrameSize.y)); // Scaled by aspect ratio
      if(nStepsY < 2)
	nStepsY = 2;
      for(int ystep = 0; ystep< nStepsY; ystep++)
	{
	  glBegin(GL_QUAD_STRIP);
	  for(int xstep = 0; xstep <= nStepsX; xstep++)
	    for(int yystep = ystep; yystep<=ystep+1; yystep++) // Two y-coords in one go - magic.
	      {
		Vector<2> v2Iter;
		v2Iter[0] = (double) xstep / nStepsX;
		v2Iter[1] = (double) yystep / nStepsY;
		// If this is a border quad, draw a little beyond the
		// outside of the frame, this avoids strange jaggies
		// at the edge of the reconstructed frame later:
		if(xstep == 0 || yystep == 0 || xstep == nStepsX || yystep == nStepsY)
		  for(int i=0; i<2; i++)
		    v2Iter[i] = v2Iter[i] * 1.02 - 0.01;
		Vector<2> v2UFBDistorted = v2Iter;
		Vector<2> v2UFBUnDistorted = mCamera.UFBLinearProject(mCamera.UFBUnProject(v2UFBDistorted));
		glTexCoord2d(v2UFBDistorted[0] * mirFrameSize.x, v2UFBDistorted[1] * mirFrameSize.y);
		glVertex(v2UFBUnDistorted);
	      }
	  glEnd();
	}
      glEndList();
    }
  else
    glCallList(nList);
  glDisable(GL_TEXTURE_RECTANGLE_ARB);
}


/**
 * Draw the distorted frame buffer
 */
void ARDriver::DrawDistortedFB()
{
  static bool bFirstRun = true;
  static GLuint nList;
  mGLWindow.SetupViewport2();
  mGLWindow.SetupUnitOrtho();
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glEnable(GL_TEXTURE_RECTANGLE_ARB);
  glBindTexture(GL_TEXTURE_RECTANGLE_ARB, mnFrameBufferTex);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glDisable(GL_POLYGON_SMOOTH);
  glDisable(GL_BLEND);
  if(bFirstRun)
    {
      bFirstRun = false;
      nList = glGenLists(1);
      glNewList(nList, GL_COMPILE_AND_EXECUTE);
      // How many grid divisions in the x and y directions to use?
      int nStepsX = 24; // Pretty arbitrary..
      int nStepsY = (int) (nStepsX * ((double) mirFrameSize.x / mirFrameSize.y)); // Scaled by aspect ratio
      if(nStepsY < 2)
	nStepsY = 2;
      glColor3f(1,1,1);
      for(int ystep = 0; ystep<nStepsY; ystep++)
	{
	  glBegin(GL_QUAD_STRIP);
	  for(int xstep = 0; xstep<=nStepsX; xstep++)
	    for(int yystep = ystep; yystep<=ystep + 1; yystep++) // Two y-coords in one go - magic.
	      {
		Vector<2> v2Iter;
		v2Iter[0] = (double) xstep / nStepsX;
		v2Iter[1] = (double) yystep / nStepsY;
		Vector<2> v2UFBDistorted = v2Iter;
		Vector<2> v2UFBUnDistorted = mCamera.UFBLinearProject(mCamera.UFBUnProject(v2UFBDistorted));
		glTexCoord2d(v2UFBUnDistorted[0] * mirFBSize.x, (1.0 - v2UFBUnDistorted[1]) * mirFBSize.y);
		glVertex(v2UFBDistorted);
	      }
	  glEnd();
	}
      glEndList();
    }
  else
    glCallList(nList);
  glDisable(GL_TEXTURE_RECTANGLE_ARB);
}

/**
 * Draw the fading grid
 */
void ARDriver::DrawFadingGrid()
{
  double dStrength;
  if(mnCounter >= 60)
    return;
  if(mnCounter < 30)
    dStrength = 1.0;
  dStrength = (60 - mnCounter) / 30.0;

  glColor4f(1,1,1,dStrength);
  int nHalfCells = 8;
  if(mnCounter < 8)
    nHalfCells = mnCounter + 1;
  int nTot = nHalfCells * 2 + 1;
  Vector<3>  aaVertex[17][17];
  for(int i=0; i<nTot; i++)
    for(int j=0; j<nTot; j++)
      {
	Vector<3> v3;
	v3[0] = (i - nHalfCells) * 0.1;
	v3[1] = (j - nHalfCells) * 0.1;
	v3[2] = 0.0;
	aaVertex[i][j] = v3;
      }

  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glLineWidth(2);
  for(int i=0; i<nTot; i++)
    {
      glBegin(GL_LINE_STRIP);
      for(int j=0; j<nTot; j++)
	glVertex(aaVertex[i][j]);
      glEnd();

      glBegin(GL_LINE_STRIP);
      for(int j=0; j<nTot; j++)
	glVertex(aaVertex[j][i]);
      glEnd();
    };
}


/**
 * What to do when the user clicks on the screen.
 * Calculates the 3d postion of the click on the plane
 * and passes info to a game, if there is one.
 * @param nButton the button pressed
 * @param irWin the window x, y location 
 */
void ARDriver::HandleClick(int nButton, ImageRef irWin )
{
  //The window may have been resized, so want to work out the coords based on the orignal image size
  Vector<2> v2VidCoords = mGLWindow.VidFromWinCoords( irWin );
  
  
  Vector<2> v2UFBCoords;
#ifdef WIN32
  Vector<2> v2PlaneCoords;   v2PlaneCoords[0] = numeric_limits<double>::quiet_NaN();   v2PlaneCoords[1] = numeric_limits<double>::quiet_NaN();
#else
  Vector<2> v2PlaneCoords;   v2PlaneCoords[0] = NAN;   v2PlaneCoords[1] = NAN;
#endif
  Vector<3> v3RayDirn_W;

  // Work out image coords 0..1:
  v2UFBCoords[0] = (v2VidCoords[0] + 0.5) / mCamera.GetImageSize()[0];
  v2UFBCoords[1] = (v2VidCoords[1] + 0.5) / mCamera.GetImageSize()[1];

  // Work out plane coords:
  Vector<2> v2ImPlane = mCamera.UnProject(v2VidCoords);
  Vector<3> v3C = unproject(v2ImPlane);
  Vector<4> v4C = unproject(v3C);
  SE3<> se3CamInv = mse3CfromW.inverse();
  Vector<4> v4W = se3CamInv * v4C;
  double t = se3CamInv.get_translation()[2];
  double dDistToPlane = -t / (v4W[2] - t);

  if(v4W[2] -t <= 0) // Clicked the wrong side of the horizon?
  {
    v4C.slice<0,3>() *= dDistToPlane;
    Vector<4> v4Result = se3CamInv * v4C;
    v2PlaneCoords = v4Result.slice<0,2>(); // <--- result
  }

  // Ray dirn:
  v3RayDirn_W = v4W.slice<0,3>() - se3CamInv.get_translation();
  normalize(v3RayDirn_W);

  if(mpMap->pGame) {
    mpMap->pGame->HandleClick(v2VidCoords, v2UFBCoords, v3RayDirn_W, v2PlaneCoords, nButton);
  }
}



/**
 * Handle the user pressing a key
 * @param sKey the key the user pressed.
 */
void ARDriver::HandleKeyPress( std::string sKey )
{
  if(mpMap && mpMap->pGame ) {
    mpMap->pGame->HandleKeyPress( sKey );
  }

}


/**
 * Load a game by name.
 * @param sName Name of the game
 */
void ARDriver::LoadGame(std::string sName)
{
  if(mpMap->pGame)
  {
    delete mpMap->pGame;
    mpMap->pGame = NULL;
  }

  mpMap->pGame = LoadAGame( sName, "");
  if( mpMap->pGame ) {
    mpMap->pGame->Init();
  }
 
}

/**
 * Advance the game logic
 */
void ARDriver::AdvanceLogic()
{
  if(mpMap->pGame) {
    mpMap->pGame->Advance();
  }
}

void ARDriver::DriveRobot(double x, double y, Vector<3> _start, MapViewer* mpViewer)
{
	driver.Initialise(x,y,_start,mpViewer);
	driver.start();
}

void ARDriver::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
 if(sCommand == "drive")
 {
	 Vector<3> start;
	 ARDriver* ad = static_cast<ARDriver*>(ptr);
	 //get the start position
	 start = ad->mapViewer->GetCurrentXZPosition();
	 ad->DriveRobot(0.5,0,start,ad->mapViewer); //drive robot to the place designated by AR
 }

 if(sCommand == "plusx")
 {
	 static_cast<ARDriver*>(ptr)->offset_x += 0.25;
 }
 if(sCommand == "minusx")
  {
	 static_cast<ARDriver*>(ptr)->offset_x -= 0.25;
  }
 if(sCommand == "plusz")
  {
	 static_cast<ARDriver*>(ptr)->offset_z += 0.25;
  }
 if(sCommand == "minusz")
  {
	 static_cast<ARDriver*>(ptr)->offset_z -= 0.25;
  }
 if(sCommand == "plusy")
   {
 	 static_cast<ARDriver*>(ptr)->offset_y += 0.25;
   }
  if(sCommand == "minusy")
   {
 	 static_cast<ARDriver*>(ptr)->offset_y -= 0.25;
   }
}


}
