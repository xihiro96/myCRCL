/*********************************************************************/

#ifndef CVCLSTATUSINSTANCE_HH
#define CVCLSTATUSINSTANCE_HH
#include <stdio.h>
#include <list>
#include "crac/xmlSchemaInstance.hh"
#include "CVCLStatusClasses.hh"

/*********************************************************************/

class CVCLStatusInstanceFile;
class CVCLStatusInstanceType;
class XmlHeaderForCVCLStatusInstance;

/*********************************************************************/
/*********************************************************************/

class CVCLStatusInstanceFile :
  public XmlSchemaInstanceBase
{
public:
  CVCLStatusInstanceFile();
  CVCLStatusInstanceFile(
    XmlVersion * versionIn,
    XmlHeaderForCVCLStatusInstance * headerIn,
    CVCLStatusInstanceType * CVCLStatusInstanceIn);
  ~CVCLStatusInstanceFile();
  void PRINTSELFDECL;

  XmlVersion * version;
  XmlHeaderForCVCLStatusInstance * header;
  CVCLStatusInstanceType * CVCLStatusInstance;
};

/*********************************************************************/

class CVCLStatusInstanceType :
  public DataThingType
{
public:
  CVCLStatusInstanceType();
  CVCLStatusInstanceType(
    XmlID * NameIn,
    CVCLCommandStatusType * CVCLStatusIn);
  ~CVCLStatusInstanceType();
  void PRINTSELFDECL;

  CVCLCommandStatusType * CVCLStatus;

  bool printTypp;
};

/*********************************************************************/

class XmlHeaderForCVCLStatusInstance
{
public:
  XmlHeaderForCVCLStatusInstance();
  XmlHeaderForCVCLStatusInstance(
    SchemaLocation * locationIn);
  ~XmlHeaderForCVCLStatusInstance();
  void PRINTSELFDECL;

  SchemaLocation * location;
};

/*********************************************************************/

#endif // CVCLSTATUSINSTANCE_HH
