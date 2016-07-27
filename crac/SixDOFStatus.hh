/*****************************************************************************
------------------------------------------------------------------------------
--  Copyright 2012-2013
--  Georgia Tech Research Institute
--  505 10th Street
--  Atlanta, Georgia 30332
--
--  This material may be reproduced by or for the U.S. Government
--  pursuant to the copyright license under the clause at DFARS
--  252.227-7013 (October 1988).
------------------------------------------------------------------------------
 *****************************************************************************/
/*!
  \file   SixDOFStatus.hh
  \brief  Provide a generic class to contain the status of a 6-DOF robotic arm

  \author Stephen Balakirsky
  \date   01/08/2015
*/
#ifndef __SixDOFStatus
#define __SixDOFStatus

#include "crac/poseInfo.hh"
#include "CRCLStatusClasses.hh"
#include <string>
#include <fstream>
#include <iostream>

/////////////////////////////////////////////////////////////////////////////
/*! Enum for command state
 */
enum CommandStateEnum
  {
    /*
      Done now begins at 1 because of xmlpositiveint
     */
    Done = 1,
    Error = 2,
    Ready = 3,
    Working = 4
  };
    
/////////////////////////////////////////////////////////////////////////////
/*! CommandStatus structure
  The CommandStatus relates the execution status of the
  currently executing command (or the most recently executed
  command, if there is no current command).
  Note that the combination of commandID and statusID should be unique for
  each transmitted message
 */
typedef struct
{
  unsigned int commandID; /* echoes the command id from the received 
			     command to which the status message applies */
  unsigned int statusID; /* ID associated with this particular status
			    message */
  CommandStateEnum commandState; /* the status of the command */
}CommandStatus;

/////////////////////////////////////////////////////////////////////////////
/* Parallel gripper structure
 */
typedef struct
{
  float separation;
}GripParallel;

/////////////////////////////////////////////////////////////////////////////
/*! Enum for gripper types
 */
enum GripType
  {
    NoGripper = 0,
    ParallelGripper,
    VacuumGripper,
    ThreeFingerGripper
  };

/////////////////////////////////////////////////////////////////////////////
/* Vacuum gripper structure
 */
typedef struct
{
  bool vacuumOn;
}GripVacuum;

/////////////////////////////////////////////////////////////////////////////
/* Three finger gripper structure
 */
typedef struct
{
  // finger positions and force
  float finger1pos, finger2pos, finger3pos;
  float finger1force, finger2force, finger3force;
}GripThreeFinger;

/////////////////////////////////////////////////////////////////////////////
/* gripper union
 */
typedef union
{
  GripParallel gripperParallel;
  GripVacuum   gripperVacuum;
  GripThreeFinger gripperThreeFinger;
}GripsAll;


/////////////////////////////////////////////////////////////////////////////
/*! GripStatus structure
  Reports the status for the various types of grippers
*/
typedef struct
{
  GripType gripperType;
  GripsAll grippersAll;
}GripStatus;

/////////////////////////////////////////////////////////////////////////////
/*! ValidFloat structure
  Contains a float with a valid/not valid flag.
*/
typedef struct
{
  float value;
  bool valid; /* a value of true indicates that value is valid */
}ValidFloat;

/////////////////////////////////////////////////////////////////////////////
/*! JointStatus structure
  Reports the status of a single joint
*/
typedef struct
{
  int jointNumber;
  ValidFloat jointPosition;
  ValidFloat jointTorque;
  ValidFloat jointVelocity;
}JointStatus;

/////////////////////////////////////////////////////////////////////////////
/* global variables and functions for use in cpp file 
*/
extern CRCLStatusFile *CRCLStatusTree;
extern int yysparse();
extern char *yysStringInputPointer;
extern char *yysStringInputEnd;

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
class SixDOFStatus{
public:
  SixDOFStatus();
  ~SixDOFStatus();
  
  CommandStatus getCommandStatus();
  JointStatus *getJointStatus();
  GripStatus getGripperStatus();
  PoseInfo getPoseInfo();
  
  void parse(char *xml_buf, int len);
  void setCommandStatus(CommandStatus commandStatusIn);
  void setGripperStatus(GripStatus gripperStatusIn);
  void setJointStatus(JointStatus *jointsIn, int numJoints);
  void setPose(PoseInfo poseIn);
  
  void printCommandStatus();
  void printPoseInfo();
  void printJointStatus();
  void printStatus(int verbosity, std::string prefix);
  int checkDone();
protected:
  CommandStatus commandStatus;
  GripStatus *gripperStatus; /* status of gripper */
  JointStatus jointStatus[6]; /* one status for each DOF of robot */
  PoseInfo *pose; /* the pose of the end-of-arm-tooling */
};
#endif
