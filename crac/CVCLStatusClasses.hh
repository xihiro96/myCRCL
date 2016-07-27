/*********************************************************************/

#ifndef CVCLSTATUS_HH
#define CVCLSTATUS_HH
#include <stdio.h>
#include <list>
#include "crac/xmlSchemaInstance.hh"
#include "DataPrimitivesClasses.hh"

/*********************************************************************/

class CVCLCommandStatusType;
class CVCLSearchStatusType;
class CommandStateEnumType;
class ModelLocationType;

/*********************************************************************/
/*********************************************************************/

class CVCLCommandStatusType :
  public DataThingType
{
public:
  CVCLCommandStatusType();
  CVCLCommandStatusType(
    XmlID * NameIn,
    XmlNonNegativeInteger * CommandIDIn,
    XmlPositiveInteger * StatusIDIn,
    CommandStateEnumType * CommandStateIn);
  ~CVCLCommandStatusType();
  void PRINTSELFDECL;

  XmlNonNegativeInteger * CommandID;
  XmlPositiveInteger * StatusID;
  CommandStateEnumType * CommandState;

  bool printTypp;
};

/*********************************************************************/

class CVCLSearchStatusType :
  public CVCLCommandStatusType
{
public:
  CVCLSearchStatusType();
  CVCLSearchStatusType(
    XmlID * NameIn,
    XmlNonNegativeInteger * CommandIDIn,
    XmlPositiveInteger * StatusIDIn,
    CommandStateEnumType * CommandStateIn,
    ModelLocationType * FoundModelsIn);
  ~CVCLSearchStatusType();
  void PRINTSELFDECL;

  ModelLocationType * FoundModels;

  bool printTypp;
};

/*********************************************************************/

class CommandStateEnumType :
  public XmlString
{
public:
  CommandStateEnumType();
  CommandStateEnumType(
    const char * valIn);
  ~CommandStateEnumType();
  bool CommandStateEnumTypeIsBad();
  void PRINTSELFDECL;
};

/*********************************************************************/

class ModelLocationType :
  public DataThingType
{
public:
  ModelLocationType();
  ModelLocationType(
    XmlID * NameIn,
    XmlString * ModelNameIn,
    PoseLocationType * ModelLocationIn,
    XmlPositiveInteger * ModelTrackingNumberIn);
  ~ModelLocationType();
  void PRINTSELFDECL;

  XmlString * ModelName;
  PoseLocationType * ModelLocation;
  XmlPositiveInteger * ModelTrackingNumber;

  bool printTypp;
};

/*********************************************************************/

#endif // CVCLSTATUS_HH
