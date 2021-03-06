/*********************************************************************/

#ifndef OWL_INSTANCE_PRINTER_HH
#define OWL_INSTANCE_PRINTER_HH

/*********************************************************************/

#include <stdio.h>             // for printf, etc.
#include <string.h>            // for strdup
#include <stdlib.h>            // for exit
#include <list>
#include "xmlSchemaInstance.hh"

class OwlInstancePrinter
{
public:
  OwlInstancePrinter();
  ~OwlInstancePrinter() {}
  void endIndi(
    const char * aType,
    FILE * outFile);
  void printIndividuals(
    const char * typeName,
    std::set<std::string> * individuals,
    FILE * outFile);
  void printObjProp(
    const char * property,
    XmlID * Name,
    XmlID * valName,
    FILE * outFile);
  void printObjRefProp(
    const char * property,
    XmlID * Name,
    XmlIDREF * valName,
    FILE * outFile);
  void printXmlBooleanProp(
    const char * property,
    XmlID * Name,
    XmlBoolean * val,
    FILE * outFile);
  void printXmlDateTimeProp(
    const char * property,
    XmlID * Name,
    XmlDateTime * val,
    FILE * outFile);
  void printXmlDecimalProp(
    const char * property,
    XmlID * Name,
    XmlDecimal * val,
    FILE * outFile);
  void printXmlDoubleProp(
    const char * property,
    XmlID * Name,
    XmlDouble * val,
    FILE * outFile);
  void printXmlIDProp(
    const char * property,
    XmlID * Name,
    XmlID * val,
    FILE * outFile);
  void printXmlIDREFProp(
    const char * property,
    XmlID * Name,
    XmlIDREF * val,
    FILE * outFile);
  void printXmlIntProp(
    const char * property,
    XmlID * Name,
    XmlInt * val,
    FILE * outFile);
  void printXmlIntegerProp(
    const char * property,
    XmlID * Name,
    XmlInteger * val,
    FILE * outFile);
  void printXmlLongProp(
    const char * property,
    XmlID * Name,
    XmlLong * val,
    FILE * outFile);
  void printXmlNMTOKENProp(
    const char * property,
    XmlID * Name,
    XmlNMTOKEN * val,
    FILE * outFile);
  void printXmlNonNegativeIntegerProp(
    const char * property,
    XmlID * Name,
    XmlNonNegativeInteger * val,
    FILE * outFile);
  void printXmlPositiveIntegerProp(
    const char * property,
    XmlID * Name,
    XmlPositiveInteger * val,
    FILE * outFile);
  void printXmlStringProp(
    const char * property,
    XmlID * Name,
    XmlString * val,
    FILE * outFile);
  void printXmlTokenProp(
    const char * property,
    XmlID * Name,
    XmlToken * val,
    FILE * outFile);
  void printXmlUnsignedIntProp(
    const char * property,
    XmlID * Name,
    XmlUnsignedInt * val,
    FILE * outFile);
  void startIndi(
    XmlID * Name,
    const char * aType,
    FILE * outFile);

  int depth;
};

/*********************************************************************/

#endif // OWL_INSTANCE_PRINTER_HH
