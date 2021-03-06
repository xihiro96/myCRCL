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

#ifndef __poseInfo
#define __poseInfo

#include <stdio.h>
#include <vector>
#include <string>
#include <math.h>
typedef enum
  {
    SENSOR,
    GNDTRUTH
  } OrientUpdateType;

class Orientation{
 public:
  Orientation();
  Orientation(double rollIn, double pitchIn, double yawIn, bool fromSensorIn);
  void clear();
  bool getRPY(double *rollOut, double *pitchOut, double *yawOut) const;
  bool isValid();
  bool updateRPY(double rollIn, double pitchIn, double yawIn, OrientUpdateType update);
  bool isSensed(); // did this orientation come from the sensor?
  void printMe(int verbosity, std::string prefix);
 private:
  double roll;
  double pitch;
  double yaw;
  int valid;
  bool sensed; // is this orientation from the sensor system?
};

/*! This class contains the information for a pose
  Note that when used as a coordinate frame, the
  xAxisName and zAxisName represent the axis of the 
  frame and not necessarily the orientation of an
  object in that frame
*/
class PoseInfo{
private:
  std::string pointName;
  std::string xAxisName;
  std::string zAxisName;
  std::vector<double>pointXYZ;
  std::vector<double>xAxis;
  std::vector<double>zAxis;
  int valid;

public:
  PoseInfo();
  void clear();
  std::vector<double> computeYAxis() const;
  std::string getPointName() const;
  int getValid() const;
  std::string getXAxisName() const;
  std::string getZAxisName() const;
  std::vector<double> getPoint() const;
  double getPointX() const;
  double getPointY() const;
  double getPointZ() const;
  PoseInfo operator+(const PoseInfo& poseInfo);
  PoseInfo operator-(const PoseInfo& poseInfo);
  void setPoint(double x, double y, double z);
  void setPointName(std::string pointNameIn);
  std::vector<double> getXAxis() const;
  double getXAxisI() const;
  double getXAxisJ() const;
  double getXAxisK() const;
  void setValid(int validIn);
  void setXAxis(double x, double y, double z);
  void setXAxisName(std::string xAxisNameIn);
  std::vector<double> getZAxis() const;
  double getZAxisI() const;
  double getZAxisJ() const;
  double getZAxisK() const;
  void setZAxis(double x, double y, double z);
  void setZAxisName(std::string zAxisNameIn);
  void getRollPitchYaw(double *roll, double *pitch, double *yaw) const;
  void setRollPitchYaw(double roll, double pitch, double yaw);
  void setRollPitchYaw(Orientation orient);
  PoseInfo invert();
  void printMe(int verbosity, std::string prefix="");
};

#endif
