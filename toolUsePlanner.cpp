/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#include "toolUsePlanner.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <Tabs/GRIPTab.h>

#include <iostream>

#include <kinematics/Joint.h>
#include <kinematics/Dof.h>

#include <Tabs/AllTabs.h>
#include <GRIPApp.h>

#include "JTFollower/JTFollower.h"


/* Quick intro to adding tabs:
 * 1- Copy template cpp and header files and replace with new class name
 * 2- include classname.h in AllTabs.h, and use the ADD_TAB macro to create it
 */

// Control IDs (used for event handling - be sure to start with a non-conflicted id)
enum planTabEvents {
  button_SetStart = 50,
  button_goToObject,
  button_pickUpObject,
  button_dropOffObject,
  button_resetPlanner,
  button_empty1,
  button_empty2,
  button_incrementObject,
  button_Stop,
  button_UpdateTime,
  button_ExportSequence,
  button_ShowPath,
  checkbox_beGreedy,
  checkbox_useConnect,
  checkbox_useSmooth,
  slider_Time
};

// sizer for whole tab
wxBoxSizer* sizerFullTool;

//Add a handler for any events that can be generated by the widgets you add here (sliders, radio, checkbox, etc)
BEGIN_EVENT_TABLE(ToolUsePlannerTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, ToolUsePlannerTab::OnSlider)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_RADIOBOX_SELECTED, ToolUsePlannerTab::OnRadio)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, ToolUsePlannerTab::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_CHECKBOX_CLICKED, ToolUsePlannerTab::OnCheckBox)
END_EVENT_TABLE()

// Class constructor for the tab: Each tab will be a subclass of RSTTab
IMPLEMENT_DYNAMIC_CLASS(ToolUsePlannerTab, GRIPTab)

/**
 * @function RipTabPlanner
 * @brief Constructor
 */
ToolUsePlannerTab::ToolUsePlannerTab( wxWindow *parent, const wxWindowID id,
				      const wxPoint& pos, const wxSize& size, long style) :
GRIPTab(parent, id, pos, size, style) {

  mStartConf.resize(0);
  mGoalConf.resize(0);

  mRobotId = 0;
  mObjectId = 3;
  mLinks.resize(0);

  mRrtStyle = 0;
  mGreedyMode = false;
  mConnectMode = false;
  mSmooth = false;
  mPlanner = NULL;

  sizerFullTool = new wxBoxSizer( wxHORIZONTAL );

  // ** Create left static box for configuring the planner **

  // Create StaticBox container for all items
  wxStaticBox* configureBox = new wxStaticBox(this, -1, wxT("Configure"));

  // Create sizer for this box with horizontal layout
  wxStaticBoxSizer* configureBoxSizer = new wxStaticBoxSizer(configureBox, wxHORIZONTAL);

  // Create a sizer for radio buttons in 1st column
  wxBoxSizer *col1Sizer = new wxBoxSizer(wxVERTICAL);
  wxBoxSizer *miniSizer = new wxBoxSizer(wxVERTICAL); // annoying hack to get checkboxes close together
  miniSizer->Add( new wxCheckBox(this, checkbox_beGreedy, _T("&goal bias (be greedy)")),
		  1, // vertical stretch evenly
		  wxALIGN_NOT,
		  0);
  miniSizer->Add( new wxCheckBox(this, checkbox_useConnect, _T("use &connect algorithm (be really greedy)")),
		  1, // vertical stretch evenly
		  wxALIGN_NOT,
		  0 );
  miniSizer->Add( new wxCheckBox(this, checkbox_useSmooth, _T("use &smoother (make it less ugly)")),
		  1, // vertical stretch evenly
		  wxALIGN_NOT,
		  0 );
  col1Sizer->Add(miniSizer,1,wxALIGN_NOT,0);

  // Create radio button for rrt_style
  static const wxString RRTStyles[] =
    {
      wxT("Single"),
      wxT("Bi-directional")
    };
  col1Sizer->Add( new wxRadioBox(this, wxID_ANY, wxT("RRT &style:"),
				 wxDefaultPosition, wxDefaultSize, WXSIZEOF(RRTStyles), RRTStyles, 1,
				 wxRA_SPECIFY_ROWS),
		  1, // stretch evenly with buttons and checkboxes
		  wxALIGN_NOT,
		  0 );
  // Add col1 to configureBoxSizer
  configureBoxSizer->Add( col1Sizer,
			  3, // 3/5 of configure box
			  wxALIGN_NOT,
			  0 ); //

  // Create sizer for start buttons in 2nd column
  wxBoxSizer *col2Sizer = new wxBoxSizer(wxVERTICAL);
  col2Sizer->Add( new wxButton(this, button_SetStart, wxT("Set &Start")),
		  0, // make horizontally unstretchable
		  wxALL, // make border all around (implicit top alignment)
		  1 ); // set border width to 1, so start buttons are close together
  col2Sizer->Add( new wxButton(this, button_pickUpObject, wxT("Pick Up Object")),
		  0, // make horizontally unstretchable
		  wxALL, // make border all around (implicit top alignment)
		  1 ); // set border width to 1, so start buttons are close together
  col2Sizer->Add( new wxButton(this, button_empty1, wxT("Check collision")),
		  0, // make horizontally unstretchable
		  wxALL, // make border all around (implicit top alignment)
		  1 ); // set border width to 1, so start buttons are close together


  // Add col2Sizer to the configuration box
  configureBoxSizer->Add( col2Sizer,
			  1, // takes half the space of the configure box
			  wxALIGN_NOT ); // no border and center horizontally

  // Create sizer for goal buttons in 3rd column
  wxBoxSizer *col3Sizer = new wxBoxSizer(wxVERTICAL);
  col3Sizer->Add( new wxButton(this, button_goToObject, wxT("Go To Object")),
		  0, // make horizontally unstretchable
		  wxALL, // make border all around (implicit top alignment)
		  1 ); // set border width to 1, so start buttons are close together
  col3Sizer->Add( new wxButton(this, button_dropOffObject, wxT("Drop Off Object")),
		  0, // make horizontally unstretchable
		  wxALL, // make border all around (implicit top alignment)
		  1 ); // set border width to 1, so start buttons are close together
  col3Sizer->Add( new wxButton(this, button_empty2, wxT("Empty 2")),
		  0, // make horizontally unstretchable
		  wxALL, // make border all around (implicit top alignment)
		  1 ); // set border width to 1, so start buttons are close together
  configureBoxSizer->Add( col3Sizer,
			  1, // size evenly with radio box and checkboxes
			  wxALIGN_NOT ); // no border and center horizontally

  // Add this box to parent sizer
  sizerFullTool->Add( configureBoxSizer,
		      4, // 4-to-1 ratio with execute sizer, since it just has 3 buttons
		      wxEXPAND | wxALL,
		      6 );


  // ** Create right static box for running the planner **
  wxStaticBox* executeBox = new wxStaticBox(this, -1, wxT("Execute Planner"));

  // Create sizer for this box
  wxStaticBoxSizer* executeBoxSizer = new wxStaticBoxSizer(executeBox, wxVERTICAL);

  // Add buttons for "plan", "save movie", and "show path"
  executeBoxSizer->Add( new wxButton(this, button_incrementObject, wxT("Increment Object")),
			1, // stretch to fit horizontally
			wxGROW ); // let it hog all the space in it's column

  executeBoxSizer->Add( new wxButton(this, button_Stop, wxT("&Stop")),
			1, // stretch to fit horizontally
			wxGROW );


  wxBoxSizer *timeSizer = new wxBoxSizer(wxHORIZONTAL);
  mTimeText = new wxTextCtrl(this,1008,wxT("5.0"),wxDefaultPosition,wxSize(40,20),wxTE_RIGHT);//,wxTE_PROCESS_ENTER | wxTE_RIGHT);
  timeSizer->Add( mTimeText,2,wxALL,1 );
  timeSizer->Add(new wxButton(this, button_UpdateTime, wxT("Set T(s)")),2,wxALL,1);
  executeBoxSizer->Add(timeSizer,1,wxALL,2);

  executeBoxSizer->Add( new wxButton(this, button_ShowPath, wxT("&Print")),
			1, // stretch to fit horizontally
			wxGROW );

  sizerFullTool->Add(executeBoxSizer, 1, wxEXPAND | wxALL, 6);

  SetSizer(sizerFullTool);

}

/**
 * @function OnRadio
 * @brief Handle Radio toggle
 */
void ToolUsePlannerTab::OnRadio(wxCommandEvent &evt) {

  mRrtStyle = evt.GetSelection();
  std::cout << "rrtStyle = " << mRrtStyle << std::endl;
}

/**
 * @function OnButton
 * @brief Handle Button Events
 */
void ToolUsePlannerTab::OnButton(wxCommandEvent &evt) {

  int button_num = evt.GetId();
  getLinks();

  switch (button_num) {

    /** Set Start */
  case button_SetStart:
    if ( mWorld != NULL ) {
      if( mWorld->getNumRobots() < 1) {
	std::cout << "(!) Must have a world with a robot to set a Start state" << std::endl;
	break;
      }
      std::cout << "(i) Setting Start state for " << mWorld->getRobot(mRobotId)->getName() << ":" << std::endl;
      mStartConf.resize(0);
      mStartConf = mWorld->getRobot(mRobotId)->getDofs( mLinks );

      for( unsigned int i = 0; i < mStartConf.size(); i++ )
	{  std::cout << mStartConf(i) << " ";  }
      std::cout << std::endl;
    } else {
      std::cout << "(!) Must have a world loaded to set a Start state." << std::endl;
    }
    break;

    /** Go To Object */
  case button_goToObject:
    {
      if ( mWorld != NULL ) {
	if( mWorld->getNumRobots() < 1){
	  std::cout << "(!) Must have a world with a robot to set a Goal state.(!)" << std::endl;
	  break;
	}
      }
      mStartConf.resize(0);
      mStartConf = mWorld->getRobot(mRobotId)->getDofs( mLinks );
      std::cout << "(i) Start state :" << mStartConf << std::endl;
      std::cout << "(i) Setting Goal state for " << mWorld->getObject(mObjectId)->getName() << std::endl;


      // Get the transform of the current object
      Eigen::MatrixXd qTransform = mWorld->getObject(mObjectId)->getRoot()->getWorldTransform();

      double r,p,y;
      r = atan2( qTransform(2,1), qTransform(2,2) );
      p = -asin( qTransform(2,0) );
      y = atan2( qTransform(1,0), qTransform(0,0) );
      std::string mObjName = mWorld->getObject(mObjectId)->getName();

      Eigen::VectorXd qRPY(3);
      Eigen::VectorXd qXYZ(3);
      if (mObjName == "bolt"){
	qRPY << r,p-PI/2.0,y;
	qXYZ << qTransform(0,3), qTransform(1,3), qTransform(2,3)+.1;
      }
      else if (mObjName == "driver"){
	qRPY << r,p-PI/2.0,y;
	qXYZ << qTransform(0,3), qTransform(1,3), qTransform(2,3)+.15;
      }
      else {
	qRPY << r,p,y+PI/2;
	qXYZ << qTransform(0,3), qTransform(1,3)+.15, qTransform(2,3);
      }


      JTFollower *jt = new JTFollower(*mWorld);
      jt->init( mRobotId, mLinks, mEEName, mEEId, 0.02 );

      // Move the arm to that configuration
      std::vector<Eigen::VectorXd> wsPath;
      Eigen::VectorXd start = mStartConf;

      if( jt->GoToXYZR( start, qXYZ, qRPY, wsPath ) == true){
	printf("Found solution JT! \n");
	SetTimeline( wsPath ,true);
      }
      else{
	printf("NO Found solution JT! Plotting anyway \n");
	SetTimeline( wsPath ,true);
      }
    }
    break;

    /** Pick Up Object */
  case button_pickUpObject:
    {
      pickedUp = true;
      pickedUpObjectId = mObjectId;
      // Get the transform of the current object
      Eigen::MatrixXd qTransform = mWorld->getObject(mObjectId)->getRoot()->getWorldTransform();

      // Get the transform of the end effector
      Eigen::MatrixXd eTransform = mWorld->getRobot(mRobotId)->getNode(mEEId)->getWorldTransform();

      // Set mRelationship equal to the affine transform matrix to produce given offset
      mRelationship = eTransform.inverse()*qTransform;

    }
    break;

    /** Drop Off Object */
  case button_dropOffObject:
    pickedUp = 0;
    break;

    /** Reset Planner */
  case button_resetPlanner:
    if ( mWorld != NULL) {
      if ( mPlanner != NULL)
	delete mPlanner;

      std::cout << "Creating a new planner" << std::endl;
      double stepSize = 0.1; // default
      mPlanner = new PathPlanner( *mWorld, false, stepSize );
    } else {
      std::cout << "(!) Must have a world loaded to make a planner" << std::endl;
    }
    break;

    /** Empty button 1 */
  case button_empty1:
    {
      std::cout << "(0) Checking Collisions" << std::endl;
      bool st;
      st = mWorld->checkCollision();
      if( st == true )
	{ printf("Collisions \n");}
      else
	{ printf("No Collisions \n");}

    }
    break;

    /** Empty button 2 */
  case button_empty2:
    std::cout << "-- (0) Empty Button to use for whatever you want (0)--" << std::endl;
    break;

    /** Increment Object ID */
  case button_incrementObject:
    {
      int numObjects = mWorld->getNumObjects();
      mObjectId += 1;
      if (mObjectId == numObjects)
	{
	  mObjectId = 0;
	}
      std::cout << "(i) Selected Object:" << mWorld->getObject(mObjectId)->getName() << std::endl;
    }
    break;

    /** Update Time */
  case button_UpdateTime:
    {
      /// Update the time span of the movie timeline
      //SetTimeline();
    }
    break;

    /** Show Path */
  case button_ShowPath:
    if( mWorld == NULL || mPlanner == NULL || mPlanner->path.size() == 0 ) {
      std::cout << "(!) Must create a valid plan before printing."<<std::endl;
      return;
    } else {
      std::cout<<"(i) Printing...Implement me :)"<<std::endl;
    }
    break;
  }
}

/**
 * @function setTimeLine
 * @brief
 */
void ToolUsePlannerTab::SetTimeline(std::vector< Eigen::VectorXd > _path, bool resetPath) {

  if( mWorld == NULL  ) {
    printf("--(!) Must create a valid plan before updating its duration (!)--");
    return;
  }

  double T = 10;
  int numsteps = _path.size();
  double increment = T/(double)numsteps;

  printf( "** Ready to see Plan: Updated Timeline - Increment: %f, Total T: %f  Steps: %d \n", increment, T, numsteps);

  if(resetPath)
    frame->InitTimer( string("Plan"),increment );
  Eigen::VectorXd vals( mLinks.size() );

  for( int i = 0; i < _path.size(); i++ ) {
    mWorld->getRobot( mRobotId )->setDofs( _path[i], mLinks );
    mWorld->getRobot(mRobotId)->update();


    if (pickedUp){
      // Get the transform of the end effector
      Eigen::MatrixXd eTransform = mWorld->getRobot(mRobotId)->getNode(mEEId)->getWorldTransform();

      // Produce transform of the tool based on offset from end effector via affine transformation
      Eigen::MatrixXd qPos = eTransform * mRelationship;

      double r,p,y;
      r = atan2( qPos(2,1), qPos(2,2) );
      p = -asin( qPos(2,0) );
      y = atan2( qPos(1,0), qPos(0,0) );

      // Set object XYZ, RPY from values calculated from affine transformation
      mWorld->getObject(pickedUpObjectId)->setPositionXYZ(qPos(0,3), qPos(1,3), qPos(2,3));
      mWorld->getObject(pickedUpObjectId)->setRotationRPY(r,p,y);
      mWorld->getObject(pickedUpObjectId)->update();

    }
    frame->AddWorld( mWorld );
  }

}

/**
 * @function OnCheckBox
 * @brief Handle CheckBox Events
 */
void ToolUsePlannerTab::OnCheckBox( wxCommandEvent &evt ) {
  int checkbox_num = evt.GetId();

  switch (checkbox_num) {

  case checkbox_beGreedy:
    mGreedyMode = (bool)evt.GetSelection();
    std::cout << "(i) greedy = " << mGreedyMode << std::endl;
    break;

  case checkbox_useConnect:
    mConnectMode = (bool)evt.GetSelection();
    std::cout << "(i) useConnect = " << mConnectMode << std::endl;
    break;
  case checkbox_useSmooth:
    mSmooth = (bool)evt.GetSelection();
    std::cout << "(i) Smooth option = " << mSmooth << std::endl;
    break;
  }
}

/**
 * @function OnSlider
 * @brief Handle slider changes
 */
void ToolUsePlannerTab::OnSlider(wxCommandEvent &evt) {
  if (selectedTreeNode == NULL) {
    return;
  }

  int slnum = evt.GetId();
  double pos = *(double*) evt.GetClientData();
  char numBuf[1000];

  switch (slnum) {
  case slider_Time:
    sprintf(numBuf, "X Change: %7.4f", pos);
    std::cout << "(i) Timeline slider output: " << numBuf << std::endl;
    //handleTimeSlider(); // uses slider position to query plan state
    break;

  default:
    return;
  }
  //world->updateCollision(o);
  //viewer->UpdateCamera();

  if (frame != NULL)
    frame->SetStatusText(wxString(numBuf, wxConvUTF8));
}

/**
 * @function GRIPStateChange
 * @brief This function is called when an object is selected in the Tree View or other
 *        global changes to the RST world. Use this to capture events from outside the tab.
 */
void ToolUsePlannerTab::GRIPStateChange() {
  if ( selectedTreeNode == NULL ) {
    return;
  }

  std::string statusBuf;
  std::string buf, buf2;

  switch (selectedTreeNode->dType) {

  case Return_Type_Object:
    mSelectedObject = (robotics::Object*) ( selectedTreeNode->data );
    statusBuf = " Selected Object: " + mSelectedObject->getName();
    buf = "You clicked on object: " + mSelectedObject->getName();

    // Enter action for object select events here:

    break;
  case Return_Type_Robot:
    mSelectedRobot = (robotics::Robot*) ( selectedTreeNode->data );
    statusBuf = " Selected Robot: " + mSelectedRobot->getName();
    buf = " You clicked on robot: " + mSelectedRobot->getName();

    // Enter action for Robot select events here:

    break;
  case Return_Type_Node:
    mSelectedNode = (dynamics::BodyNodeDynamics*) ( selectedTreeNode->data );
    statusBuf = " Selected Body Node: " + string(mSelectedNode->getName()) + " of Robot: "
      + ( (robotics::Robot*) mSelectedNode->getSkel() )->getName();
    buf = " Node: " + std::string(mSelectedNode->getName()) + " of Robot: " + ( (robotics::Robot*) mSelectedNode->getSkel() )->getName();

    // Enter action for link select events here:

    break;
  default:
    fprintf(stderr, "--( :D ) Someone else's problem!\n");
    assert(0);
    exit(1);
  }

  //cout << buf << endl;
  frame->SetStatusText(wxString(statusBuf.c_str(), wxConvUTF8));
  sizerFullTool->Layout();
}

/**
 * @function getLinks
 */
void ToolUsePlannerTab::getLinks() {

  mNumLinks = mWorld->getRobot(mRobotId)->getNumQuickDofs();

  mLinks.resize( mNumLinks );
  mLinks =  mWorld->getRobot(mRobotId)->getQuickDofsIndices();

  mEEName = "FT";
  mEEId = -1;

  for( int i = 0; i < mNumLinks; i++){
    int EEDofId = mLinks( i );
    int id = mWorld->getRobot(mRobotId)->getDof( EEDofId )->getJoint()->getChildNode()->getSkelIndex();
    if ( mEEName.compare(mWorld->getRobot(mRobotId)->getNode(id)->getName()) == 0){
      mEEId = id;
    }
  }

  std::cout << "Link IDs: " << mLinks.transpose() << std::endl;
  std::cout << " EE Name: "<<mWorld->getRobot(mRobotId)->getNode(mEEId)->getName() << std::endl;

  // Only for Schunk (no hand) -- Comment otherwise
  //mStartHardcode.resize( mNumLinks );
  //mStartHardcode << 0.528,  -1.089,  0.176,  -1.156,  -0.276,  -0.873,  0.000; // -> Pointing down
  // mStartHardcode <<  0.075,  -1.022,  -0.478,  -1.022,  0.000,  -0.854,  0.000 ;
}
