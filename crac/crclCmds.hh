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
  \file   crclCmds.hh
  \brief  All of the various commands found in CRCLCommands.xsd

  \author Stephen Balakirsky
  \date   01/14/2015
*/
#ifndef __crclCmds
#define __crclCmds
#include <crac/poseInfo.hh>
#include <crac/crclCmdHeader.hh>
#include <vector>
// need to include crcl commands classes somehow
//#include "~/gtri/crac/xml/crcl/src/CRCLCommandsClasses.hh"

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

class ActuateJoint;
class JointControlMode; // need to add
class JointReport;
class PoseTolerance;

class CrclActuateJoints; // need to add
class CrclCloseToolChanger;
class CrclConfigureJointReports; // need to do
class CrclDwell;
class CrclEndCanon;
class CrclGetStatus;
class CrclInitCanon;
class CrclMessage;
class CrclMoveScrew; // not currently supported
class CrclMoveThroughTo;
class CrclMoveTo;
class CrclOpenToolChanger;
class CrclRunProgram;
class CrclSetAngleUnits;
class CrclSetEndEffectorParameters;
class CrclSetEndEffector;
class CrclSetEndPoseTolerance;
class CrclSetForceUnits;
class CrclSetIntermediatePoseTolerance;
class CrclSetLengthUnits;
class CrclSetMotionCoordination;
class CrclSetRobotParameters;
class CrclSetTorqueUnits;
class CrclStopMotion;
//class CrclConfigureJointReport; 
//class ParameterSettingType;
//class CrclPoseAndSet;
//class CrclPoseTolerance;
//class CrclJointDetails;
//class JointSpeedAccelType;
/*
Can add these as needed
class CrclSetRotAccelAbsolute;
class CrclSetRotAccelRelative;
class CrclSetRotAccel;
class CrclSetRotSpeedAbsolute;
class CrclSetRotSpeedRelative;
class CrclSetRotSpeed;
class CrclSetTransAccelAbsolute;
class CrclSetTransAccelRelative;
class CrclSetTransAccel;
class CrclSetTransSpeedAbsolute;
class CrclSetTransSpeedRelative;
class CrclSetTransSpeed;
*/


/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  Joint details
*/

typedef struct
{
  float setting;
  float changeRate;
} JointForceTorque;

typedef struct
{
  float jointSpeed;
  float jointAccel;
} JointSpeedAccel;

typedef union 
{ 
  JointForceTorque jointForceTorque;
  JointSpeedAccel jointSpeedAccel;
} JointDetails;

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  ActuateJoint class for adding to CrclActuateJoints
  May add support for CrclActuateJoint
*/
class ActuateJoint {
public:
  ActuateJoint();
  ActuateJoint(int jointIn, float posIn);
  ~ActuateJoint();
  void SetActuateJoint(int jointIn, float posIn);
  void SetJointForceTorque(float settingIn, float changeRateIn);
  void SetJointSpeedAccel(float speedIn, float accelIn);
  int GetJoint();
  float GetPosition();
  int GetDetailType();
  JointDetails GetDetails();
private:
  int joint;
  float position;
  JointDetails jdetails;
  // bit 0 specifies if joint & position are declared
  // bit 1 specifies jointForceTorque. bit 2 specifies jointSpeedAccel
  bool valid[3];
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
class JointReport{
public:
  JointReport();
  JointReport(int jointNumIn, bool positionIn, bool torqueIn, bool velocityIn);
  ~JointReport();
  void setJPTV(int jointNumIn, bool positionIn, bool torqueIn, bool velocityIn);
  //void setJointNum(int jointNumIn);
  //void setPosition(bool positionIn);
  //void setTorque(bool torqueIn);
  //void setVelocity(bool velocityIn);
  int getJointnum();
  bool getPos();
  bool getTor();
  bool getVel();
  bool getValid();
private:
  int jointNum;
  bool reportPos;
  bool reportTor;
  bool reportVel;
  bool valid;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  The PoseTolerance class for use in SetIntermediatePoseToleranceType class and
  SetEndPoseTolerance class. Indicates to the robot the precision with which 
  it must reach each intermediate waypoint.
*/
class PoseTolerance {
public:
  PoseTolerance();
  PoseTolerance(float xPTolIn,
		float yPTolIn,
		float zPTolIn,
		float xATolIn,
		float zATolIn);
  ~PoseTolerance();
  void setTolerance(float xPTolIn,
		    float yPTolIn,
		    float zPTolIn,
		    float xATolIn,
		    float zATolIn);
  float getXPTol();
  float getYPTol();
  float getZPTol();
  float getXATol();
  float getZATol();
private:
  float xPointTolerance;
  float yPointTolerance;
  float zPointTolerance;
  float xAxisTolerance;
  float zAxisTolerance;
  bool valid;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  Actuate Joints moves multiple joints in the same manner as CrclActuateJoint
  Each joint may appear in at most one ActuateJoint element. If
  a joint appears in no ActuateJoint element, its actuation should
  be as previously set.

  May add get function
*/
class CrclActuateJoints : public CrclCmdHeader{
public:
  CrclActuateJoints();
  ~CrclActuateJoints();
  virtual bool checkValid();
  void AddActuate(ActuateJoint actuateJointIn);
protected:
  ActuateJointsType *actuateJoints;
  std::vector<ActuateJoint> actuateList;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  If the tool changer was in a position to aquire an end effector, the end
  effector will be mounted on the robot
*/
class CrclCloseToolChanger: public CrclCmdHeader{
public:
  CrclCloseToolChanger();
  ~CrclCloseToolChanger();
  virtual bool checkValid();
protected:
  CloseToolChangerType *closeToolChanger;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  ConfigureJointReportType is used to specify whether and how status
  reporting should be done for the joint identified by its joint
  number. For each ReportXXX element, true means XXX data should be
  reported and false means XXX data should not be reported.

  ReportPosition
  ReportTorqueOrForce
  ReportVelocity

  By default all will be set to true
*/
/*
class CrclConfigureJointReport: public CrclCmdHeader{
public:
  CrclConfigureJointReport();
  ~CrclConfigureJointReport();
  void setJointReport(JointReport jointReportIn);
protected:
  ConfigureJointReportType *jointReport;
  JointReport report;
};
*/
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  Configure multiple joints. Each joint can only appear once and joint
  numbers must be given in increasing order
  
  Has reset all option where (if true) all joint reporting will be reset and
  the status of any joints not specified will not be reported

  reset all will be set to false by default
*/
class CrclConfigureJointReports: public CrclCmdHeader{
public:
  CrclConfigureJointReports();
  ~CrclConfigureJointReports();
  virtual bool checkValid();
  void addJointReport(JointReport jointReportIn);
  void setResetAll(bool resetAllIn);
protected:
  ConfigureJointReportsType *jointReports;
  std::vector<JointReport> jointReportsList;
  bool resetAll;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  The DwellTime is an amount of time, in seconds, that the robot
  should wait before executing the next command.
*/
class CrclDwell : public CrclCmdHeader{
public:
  CrclDwell();
  ~CrclDwell();
  virtual bool checkValid();
  float getTime();
  void setTime(float timeIn);
protected:
  DwellType *dwell;
  float time;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  An instance of EndCanonType is used to indicate that the robot
  should not execute any further CRCL commands other than an
  instance of InitCanonType until an InitCanonType command is
  received. Other robot-specific actions may be taken in
  preparation for shutting down.
*/
class CrclEndCanon: public CrclCmdHeader{
public:
  CrclEndCanon();
  ~CrclEndCanon();
  virtual bool checkValid();
protected:
  EndCanonType *endCanon;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  An instance of GetStatusType is used to indicate that the robot
  should report status immediately. The joint status portion of
  the status report must be as set by the most recent
  ConfigureJointReports command.
*/
class CrclGetStatus: public CrclCmdHeader{
public:
  CrclGetStatus();
  ~CrclGetStatus();
  virtual bool checkValid();
protected:
  GetStatusType *getStatus;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  An instance of InitCanonType is used to indicate that the
  robot should be prepared to execute further canonical robot
  commands. When a robot is ready to execute commands, the first
  CRCL command it should be sent is an instance of
  InitCanonType. Any CRCL commands received before an instance
  of InitCanonType must not be executed. Other robot-specific
  actions may be taken in preparation for executing CRCL
  commands.
*/
class CrclInitCanon: public CrclCmdHeader{
public:
  CrclInitCanon();
  ~CrclInitCanon();
  virtual bool checkValid();
protected:
  InitCanonType *initCanon;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  Sends a message to the robot that should be displayed by the robot
  controller.
*/
class CrclMessage: public CrclCmdHeader{
public:
  CrclMessage();
  CrclMessage(std::string messageIn);
  ~CrclMessage();
  virtual bool checkValid();
  void CrclMessageConstructor();
  void setMessage(std::string messageIn);
protected:
  MessageType *messageType;
  std::string message;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  ScrewMove Class
  Needs to be implemented
*/
class CrclMoveScrew: public CrclCmdHeader{
public:
  CrclMoveScrew();
  ~CrclMoveScrew();
  virtual bool checkValid();
protected:
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  Move Command which creates waypoints which the robot should
  move through
*/
class CrclMoveThroughTo: public CrclCmdHeader{
public:
  CrclMoveThroughTo();
  ~CrclMoveThroughTo();
  virtual bool checkValid();
  bool getMoveStraight();
  void addWayPoint(PoseInfo wayPointIn);
  void setMoveStraight(bool moveStraightIn);
protected:
  MoveThroughToType *moveThrough;
  bool moveStraight; // set to false as default
  int NumPositions;
  std::vector<PoseInfo> waypointList;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  EndPosition is a Pose to which the robot will move. If the value of
  MoveStraight is true, the controlled point must be moved in a
  straight line. If the value of MoveStraight is false, the
  controlled point may be moved along any convenient trajectory.
  
  The robot must reach the EndPosition within the tolerance
  established (1) by the tolerance given for the pose in the
  EndPosition, if there is a tolerance in the EndPosition, or if not
  (2) by the most recently executed instance of
  SetEndPoseToleranceType. The speed and acceleration to use are set
  either in the EndPosition or by previously executed CRCL commands.
 */
class CrclMoveTo: public CrclCmdHeader{
public:
  CrclMoveTo();
  ~CrclMoveTo();
  virtual bool checkValid();
  void setEndPosition(PoseInfo endPositionIn);
  void setMoveStraight(bool moveStraightIn);
  PoseInfo getEndPosition();
  bool getMoveStraight();
protected:
  bool moveStraight;
  PoseInfo endPosition;
  MoveToType *moveTo;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  Open tool changer
*/
class CrclOpenToolChanger: public CrclCmdHeader{
public:
  CrclOpenToolChanger();
  ~CrclOpenToolChanger();
  virtual bool checkValid();
protected:
  OpenToolChangerType *openToolChanger;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  The RunProgramType is used to instruct the low level controller
  to run a program written in a non-CRCL language that controller
  understands. The ProgramText element gives the text of the
  program.
*/
class CrclRunProgram: public CrclCmdHeader{
public:
  CrclRunProgram();
  ~CrclRunProgram();
  CrclRunProgram(std::string ProgramIn);
  virtual bool checkValid();
  void CrclRunProgramConstructor();
  void setProgram(std::string ProgramIn);
protected:
  RunProgramType *runProg;
  std::string programName;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  enum for angle units. Can be either degree or radian
*/
typedef enum angleUnitsEnum{ radian, degree } angleUnits;

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  Set Angle units to either 'radian' or 'degree'
 */
class CrclSetAngleUnits: public CrclCmdHeader{
public:
  CrclSetAngleUnits();
  ~CrclSetAngleUnits();
  virtual bool checkValid();
  void setRadian();
  void setDegree();
protected:
  SetAngleUnitsType *setAngleUnits;
  angleUnits unit;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  Not in use
  Need to implement if want to use
*/
class CrclSetEndEffectorParameters: public CrclCmdHeader{
public:
  CrclSetEndEffectorParameters();
  ~CrclSetEndEffectorParameters();
  virtual bool checkValid();
  void setParameters(std::string nameIn, std::string valueIn);
protected:
  SetEndEffectorParametersType *endEffectorParameters;
  std::string parameterName;
  std::string parameterValue;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  SetEndEffector is for setting the effectivity of end
  effectors.  If an end effector has multiple control modes, the
  control mode must be set using a SetEndEffectorParameters
  command, so that the meaning of SetEndEffector commands is
  unambiguous.

  For end effectors that have a continuously variable setting,
  the Setting means a fraction of maximum openness, force, torque,
  power, etc.

  For end effectors that have only two choices (powered or
  unpowered, open or closed, on or off), a positive Setting
  value means powered, open, or on, while a zero Setting value
  means unpowered, closed, or off.
 */
class CrclSetEndEffector: public CrclCmdHeader{
public:
  CrclSetEndEffector();
  CrclSetEndEffector(float set);
  ~CrclSetEndEffector();
  virtual bool checkValid();
  void CrclSetEndEffectorConstructor();
  float getSetting();
  void setSetting(float settingIn); /* this method should
				       verify that the date is
				       within the range of 0 - 100
				    */
protected:
  float setting; /* for our vacuum effector, this will be 
		    0 for closed and positive for open.
		    For our electric effector, this should
		    be a precentage of the maximum opening
		    with a range of 0 - 100
		 */
  SetEndEffectorType *endEffector;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  The Tolerance element indicates to the robot the precision with
  which it must reach its end location.
*/
class CrclSetEndPoseTolerance: public CrclCmdHeader{
public:
  CrclSetEndPoseTolerance();
  ~CrclSetEndPoseTolerance();
  virtual bool checkValid();
  void setEndTol(PoseTolerance poseTolClassIn);
  void setEndTol(float xPTolIn,
		 float yPTolIn,
		 float zPTolIn,
		 float xATolIn,
		 float zATolIn);
  PoseTolerance getPoseTolerance();
protected:
  PoseTolerance myPoseTol;
  SetEndPoseToleranceType *endPoseTolerance;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  enum for force units. Can be either newton, pound, or ounce
*/
typedef enum ForceUnitsEnum{ newton, pound, ounce }  ForceUnits;

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  Set force units to 'newton', 'pound', or 'ounce'. This tells the robot
  that all further commands giving force values will implicitly use the
  named unit.
*/
class CrclSetForceUnits: public CrclCmdHeader{
public:
  CrclSetForceUnits();
  ~CrclSetForceUnits();
  virtual bool checkValid();
  void setNewton();
  void setPound();
  void setOunce();
protected:
  SetForceUnitsType *setForceUnits;
  ForceUnits forceUnits;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  The Tolerance element indicates to the robot the precision with
  which it must reach each intermediate waypoint.
*/
class CrclSetIntermediatePoseTolerance: public CrclCmdHeader{
public:
  CrclSetIntermediatePoseTolerance();
  ~CrclSetIntermediatePoseTolerance();
  virtual bool checkValid();
  void setIntermediateTol(PoseTolerance poseTolClassIn);
  void setIntermediateTol(float xPTolIn,
			  float yPTolIn,
			  float zPTolIn,
			  float xATolIn,
			  float zATolIn);
  PoseTolerance getPoseTolerance();
protected:
  SetIntermediatePoseToleranceType *poseTolerance;
  PoseTolerance myPoseTol;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  An instance of SetLengthUnitsType tells the robot that all further
  commands giving position or length values will implicitly use the
  named unit.
  It be set as 'meter', 'millimeter', or inch
*/
class CrclSetLengthUnits: public CrclCmdHeader{
public:
  CrclSetLengthUnits();
  ~CrclSetLengthUnits();
  virtual bool checkValid();
  void setMeter();
  void setMillimeter();
  void setInch();
protected:
  SetLengthUnitsType *lengthUnits;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  Coordinated is a boolean. If the value is true, rotational and
  translational motion must finish simultaneously in motion commands
  (including each segment in a multiple segment motion command),
  except as possibly temporarily overridden in the the motion
  command. If the value is false, there is no such requirement.
*/
class CrclSetMotionCoordination: public CrclCmdHeader{
public:
  CrclSetMotionCoordination();
  CrclSetMotionCoordination(bool coordinatedIn);
  ~CrclSetMotionCoordination();
  virtual bool checkValid();
  void CrclSetMotionCoordinationConstructor(bool coordinatedIn);
  void SetCoordinated(bool coordinatedIn);
protected:
  bool coord;
  SetMotionCoordinationType *motionCoordination;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  Set robot parameters which are not set by another crcl command
  not supported right now?
 */
class CrclSetRobotParameters: public CrclCmdHeader{
public:
  CrclSetRobotParameters();
  ~CrclSetRobotParameters();
  virtual bool checkValid();
  void setParameters(std::string nameIn, std::string valueIn);
protected:
  SetRobotParametersType *robotParameters;
  std::string parameterName;
  std::string parameterValue;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  SetTorqueUnits
*/
class CrclSetTorqueUnits: public CrclCmdHeader{
public:
  CrclSetTorqueUnits();
  ~CrclSetTorqueUnits();
  virtual bool checkValid();
  void setNewtonMeter();
  void setFootPound();
protected:
  SetTorqueUnitsType *torqueUnits;
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*
  CrclStopMotion class
  Stops if one of the following conditions occur. Conditions can be set
  to "Immediate", "Fast", or "Normal"

  Immediate means the robot's drives are deactivated immediately and
  the brakes are applied. This may result in the controlled point
  being off the commanded path when the robot stops.
  
  Fast means the robot and any external axes are brought to a fast,
  controlled stop. The drives are deactivated after one second, and
  the brakes are applied. The controlled point must be kept on the
  on the commanded path as the robot stops.
  
  Normal means the robot and any external drives are stopped using
  a normal braking ramp. The drives are not deactivated, and the
  brakes are not applied. The controlled point must be kept on the
  on the commanded path as the robot stops.
*/

typedef enum stopMotionCondition{ Immediate, Fast, Normal } stopCondition;

class CrclStopMotion: public CrclCmdHeader{
public:
  CrclStopMotion();
  ~CrclStopMotion();
  virtual bool checkValid();
  void setConditionImmediate();
  void setConditionFast();
  void setConditionNormal();
protected:
  StopMotionType *stopMotion;
  stopCondition cond;
};

#endif
