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
  \file   crclCmdHeader.hh
  \brief  Provide the common parent for all commands. 
          Based on CRCLCommands.xsd

  \author Stephen Balakirsky
  \date   01/14/2015
*/
#ifndef __crclCmdHeader
#define __crclCmdHeader

#include <string.h>
// Need CRCLCommandINstatnceClasses
#include "CRCLCommandInstanceClasses.hh"

/////////////////////////////////////////////////////////////////////////////
// structures and typedefs
typedef struct {
  std::string xmlCmd;
  unsigned int cmdID;
  bool valid;
}CxclCmdReturn;

/////////////////////////////////////////////////////////////////////////////
class CrclCmdHeader{
public:
  CrclCmdHeader();
  ~CrclCmdHeader();
  void setCommandID(unsigned int commandIDIn);
  void setName(std::string nameIn);
  void printFile();
  virtual bool checkValid();
  std::string getName();
  unsigned int getCommandID();
  CxclCmdReturn getCmd();
  void printxml();
  static unsigned int commandID;
protected:
  std::string name; /* optional parameter */
  CRCLCommandInstanceFile *crclCmdInst;
  XmlHeaderForCRCLCommandInstance *headerIn;
  bool valid;
};
#endif

