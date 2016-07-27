/*********************************************************************/

#ifndef CVCLCOMMANDS_HH
#define CVCLCOMMANDS_HH
#include <stdio.h>
#include <list>
#include "crac/xmlSchemaInstance.hh"
#include "DataPrimitivesClasses.hh"

/*********************************************************************/

class CVCLCommandType;
class CalibrateExtrinsicType;
class CalibrateIntrinsicType;
class EndCanonType;
class InitCanonType;
class MiddleCommandType;
class SearchType;

/*********************************************************************/
/*********************************************************************/

class CVCLCommandType :
  public DataThingType
{
public:
  CVCLCommandType();
  CVCLCommandType(
    XmlID * NameIn,
    XmlPositiveInteger * CommandIDIn);
  ~CVCLCommandType();
  void PRINTSELFDECL;

  XmlPositiveInteger * CommandID;

  bool printTypp;
};

/*********************************************************************/

class EndCanonType :
  public CVCLCommandType
{
public:
  EndCanonType();
  EndCanonType(
    XmlID * NameIn,
    XmlPositiveInteger * CommandIDIn);
  ~EndCanonType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class InitCanonType :
  public CVCLCommandType
{
public:
  InitCanonType();
  InitCanonType(
    XmlID * NameIn,
    XmlPositiveInteger * CommandIDIn);
  ~InitCanonType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class MiddleCommandType :
  public CVCLCommandType
{
public:
  MiddleCommandType();
  MiddleCommandType(
    XmlID * NameIn,
    XmlPositiveInteger * CommandIDIn);
  ~MiddleCommandType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class SearchType :
  public MiddleCommandType
{
public:
  SearchType();
  SearchType(
    XmlID * NameIn,
    XmlPositiveInteger * CommandIDIn,
    XmlString * ModelNameIn,
    PositiveDecimalType * MaxReturnsIn,
    RegionOfInterestType * ROIIn);
  ~SearchType();
  void PRINTSELFDECL;

  XmlString * ModelName;
  PositiveDecimalType * MaxReturns;
  RegionOfInterestType * ROI;

  bool printTypp;
};

/*********************************************************************/

class CalibrateExtrinsicType :
  public MiddleCommandType
{
public:
  CalibrateExtrinsicType();
  CalibrateExtrinsicType(
    XmlID * NameIn,
    XmlPositiveInteger * CommandIDIn,
    PoseOnlyLocationType * CameraPositionIn);
  ~CalibrateExtrinsicType();
  void PRINTSELFDECL;

  PoseOnlyLocationType * CameraPosition;

  bool printTypp;
};

/*********************************************************************/

class CalibrateIntrinsicType :
  public MiddleCommandType
{
public:
  CalibrateIntrinsicType();
  CalibrateIntrinsicType(
    XmlID * NameIn,
    XmlPositiveInteger * CommandIDIn,
    PointType * AlphaIn,
    XmlDecimal * GammaIn,
    PointType * PrincipalPointIn);
  ~CalibrateIntrinsicType();
  void PRINTSELFDECL;

  PointType * Alpha;
  XmlDecimal * Gamma;
  PointType * PrincipalPoint;

  bool printTypp;
};

/*********************************************************************/

#endif // CVCLCOMMANDS_HH
