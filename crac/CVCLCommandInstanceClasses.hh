/*********************************************************************/

#ifndef CVCLCOMMANDINSTANCE_HH
#define CVCLCOMMANDINSTANCE_HH
#include <stdio.h>
#include <list>
#include "crac/xmlSchemaInstance.hh"
#include "CVCLCommandsClasses.hh"

/*********************************************************************/

class CVCLCommandInstanceFile;
class CVCLCommandInstanceType;
class XmlHeaderForCVCLCommandInstance;

/*********************************************************************/
/*********************************************************************/

class CVCLCommandInstanceFile :
  public XmlSchemaInstanceBase
{
public:
  CVCLCommandInstanceFile();
  CVCLCommandInstanceFile(
    XmlVersion * versionIn,
    XmlHeaderForCVCLCommandInstance * headerIn,
    CVCLCommandInstanceType * CVCLCommandInstanceIn);
  ~CVCLCommandInstanceFile();
  void PRINTSELFDECL;

  XmlVersion * version;
  XmlHeaderForCVCLCommandInstance * header;
  CVCLCommandInstanceType * CVCLCommandInstance;
};

/*********************************************************************/

class CVCLCommandInstanceType :
  public DataThingType
{
public:
  CVCLCommandInstanceType();
  CVCLCommandInstanceType(
    XmlID * NameIn,
    CVCLCommandType * CVCLCommandIn);
  ~CVCLCommandInstanceType();
  void PRINTSELFDECL;

  CVCLCommandType * CVCLCommand;

  bool printTypp;
};

/*********************************************************************/

class XmlHeaderForCVCLCommandInstance
{
public:
  XmlHeaderForCVCLCommandInstance();
  XmlHeaderForCVCLCommandInstance(
    SchemaLocation * locationIn);
  ~XmlHeaderForCVCLCommandInstance();
  void PRINTSELFDECL;

  SchemaLocation * location;
};

/*********************************************************************/

#endif // CVCLCOMMANDINSTANCE_HH
