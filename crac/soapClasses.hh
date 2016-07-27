/*********************************************************************/

#ifndef SOAP_HH
#define SOAP_HH
#include <stdio.h>
#include <list>
#include "crac/xmlSchemaInstance.hh"
#include "soapWorkstationClasses.hh"

/*********************************************************************/

class SOAPFile;
class ActionBaseType;
class ActionParameterSetType;
class DecreaseType;
class DomainType;
class EffectType;
class FunctionConditionType;
class FunctionOperationType;
class FunctionParameterType;
class FunctionToFunctionConditionType;
class FunctionToFunctionEqualType;
class FunctionToFunctionGreaterOrEqualType;
class FunctionToFunctionGreaterType;
class FunctionToFunctionLessOrEqualType;
class FunctionToFunctionLessType;
class FunctionToNumberConditionType;
class FunctionToNumberEqualType;
class FunctionToNumberGreaterOrEqualType;
class FunctionToNumberGreaterType;
class FunctionToNumberLessOrEqualType;
class FunctionToNumberLessType;
class FunctionType;
class IncreaseType;
class IntermediateStateRelationType;
class NegativePredicateType;
class PositivePredicateType;
class PreconditionType;
class PredicateParameterType;
class PredicateStateRelationOrType;
class PredicateStateRelationType;
class RCC8StateRelationType;
class SOAPType;
class XmlHeaderForSOAP;

/*********************************************************************/
/*********************************************************************/

class SOAPFile :
  public XmlSchemaInstanceBase
{
public:
  SOAPFile();
  SOAPFile(
    XmlVersion * versionIn,
    XmlHeaderForSOAP * headerIn,
    SOAPType * SOAPIn);
  ~SOAPFile();
  void PRINTSELFDECL;

  XmlVersion * version;
  XmlHeaderForSOAP * header;
  SOAPType * SOAP;
};

/*********************************************************************/

class ActionBaseType :
  public DataThingType
{
public:
  ActionBaseType();
  ActionBaseType(
    XmlID * NameIn,
    XmlString * DescriptionIn,
    std::list<ActionParameterSetType *> * ActionParameterSetIn,
    PreconditionType * PreconditionIn,
    EffectType * EffectIn);
  ~ActionBaseType();
  void PRINTSELFDECL;

  XmlString * Description;
  std::list<ActionParameterSetType *> * ActionParameterSet;
  PreconditionType * Precondition;
  EffectType * Effect;

  bool printTypp;
};

/*********************************************************************/

class ActionParameterSetType :
  public DataThingType
{
public:
  ActionParameterSetType();
  ActionParameterSetType(
    XmlID * NameIn,
    XmlNMTOKEN * ActionParameterIn,
    XmlPositiveInteger * ActionParameterPositionIn);
  ~ActionParameterSetType();
  void PRINTSELFDECL;

  XmlNMTOKEN * ActionParameter;
  XmlPositiveInteger * ActionParameterPosition;

  bool printTypp;
};

/*********************************************************************/

class DomainType :
  public DataThingType
{
public:
  DomainType();
  DomainType(
    XmlID * NameIn,
    std::list<XmlNMTOKEN *> * RequirementIn,
    std::list<XmlNMTOKEN *> * VariableIn,
    std::list<PositivePredicateType *> * PositivePredicateIn,
    std::list<FunctionType *> * FunctionIn,
    std::list<ActionBaseType *> * ActionIn);
  ~DomainType();
  void PRINTSELFDECL;

  std::list<XmlNMTOKEN *> * Requirement;
  std::list<XmlNMTOKEN *> * Variable;
  std::list<PositivePredicateType *> * PositivePredicate;
  std::list<FunctionType *> * Function;
  std::list<ActionBaseType *> * Action;

  bool printTypp;
};

/*********************************************************************/

class EffectType :
  public DataThingType
{
public:
  EffectType();
  EffectType(
    XmlID * NameIn,
    std::list<XmlIDREF *> * PositivePredicateNameIn,
    std::list<NegativePredicateType *> * NegativePredicateIn,
    std::list<IncreaseType *> * IncreaseIn,
    std::list<DecreaseType *> * DecreaseIn);
  ~EffectType();
  void PRINTSELFDECL;

  std::list<XmlIDREF *> * PositivePredicateName;
  std::list<NegativePredicateType *> * NegativePredicate;
  std::list<IncreaseType *> * Increase;
  std::list<DecreaseType *> * Decrease;

  bool printTypp;
};

/*********************************************************************/

class FunctionConditionType :
  public DataThingType
{
public:
  FunctionConditionType();
  FunctionConditionType(
    XmlID * NameIn);
  ~FunctionConditionType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class FunctionOperationType :
  public DataThingType
{
public:
  FunctionOperationType();
  FunctionOperationType(
    XmlID * NameIn,
    XmlIDREF * FunctionNameIn,
    XmlInteger * ValueIn);
  ~FunctionOperationType();
  void PRINTSELFDECL;

  XmlIDREF * FunctionName;
  XmlInteger * Value;

  bool printTypp;
};

/*********************************************************************/

class FunctionParameterType :
  public DataThingType
{
public:
  FunctionParameterType();
  FunctionParameterType(
    XmlID * NameIn,
    XmlNMTOKEN * ParameterIn,
    XmlInteger * ParameterPositionIn);
  ~FunctionParameterType();
  void PRINTSELFDECL;

  XmlNMTOKEN * Parameter;
  XmlInteger * ParameterPosition;

  bool printTypp;
};

/*********************************************************************/

class FunctionToFunctionConditionType :
  public FunctionConditionType
{
public:
  FunctionToFunctionConditionType();
  FunctionToFunctionConditionType(
    XmlID * NameIn,
    XmlIDREF * F1NameIn,
    XmlIDREF * F2NameIn);
  ~FunctionToFunctionConditionType();
  void PRINTSELFDECL;

  XmlIDREF * F1Name;
  XmlIDREF * F2Name;

  bool printTypp;
};

/*********************************************************************/

class FunctionToFunctionEqualType :
  public FunctionToFunctionConditionType
{
public:
  FunctionToFunctionEqualType();
  FunctionToFunctionEqualType(
    XmlID * NameIn,
    XmlIDREF * F1NameIn,
    XmlIDREF * F2NameIn);
  ~FunctionToFunctionEqualType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class FunctionToFunctionGreaterOrEqualType :
  public FunctionToFunctionConditionType
{
public:
  FunctionToFunctionGreaterOrEqualType();
  FunctionToFunctionGreaterOrEqualType(
    XmlID * NameIn,
    XmlIDREF * F1NameIn,
    XmlIDREF * F2NameIn);
  ~FunctionToFunctionGreaterOrEqualType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class FunctionToFunctionGreaterType :
  public FunctionToFunctionConditionType
{
public:
  FunctionToFunctionGreaterType();
  FunctionToFunctionGreaterType(
    XmlID * NameIn,
    XmlIDREF * F1NameIn,
    XmlIDREF * F2NameIn);
  ~FunctionToFunctionGreaterType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class FunctionToFunctionLessOrEqualType :
  public FunctionToFunctionConditionType
{
public:
  FunctionToFunctionLessOrEqualType();
  FunctionToFunctionLessOrEqualType(
    XmlID * NameIn,
    XmlIDREF * F1NameIn,
    XmlIDREF * F2NameIn);
  ~FunctionToFunctionLessOrEqualType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class FunctionToFunctionLessType :
  public FunctionToFunctionConditionType
{
public:
  FunctionToFunctionLessType();
  FunctionToFunctionLessType(
    XmlID * NameIn,
    XmlIDREF * F1NameIn,
    XmlIDREF * F2NameIn);
  ~FunctionToFunctionLessType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class FunctionToNumberConditionType :
  public FunctionConditionType
{
public:
  FunctionToNumberConditionType();
  FunctionToNumberConditionType(
    XmlID * NameIn,
    XmlIDREF * FunctionNameIn,
    XmlInteger * NumberIn);
  ~FunctionToNumberConditionType();
  void PRINTSELFDECL;

  XmlIDREF * FunctionName;
  XmlInteger * Number;

  bool printTypp;
};

/*********************************************************************/

class FunctionToNumberEqualType :
  public FunctionToNumberConditionType
{
public:
  FunctionToNumberEqualType();
  FunctionToNumberEqualType(
    XmlID * NameIn,
    XmlIDREF * FunctionNameIn,
    XmlInteger * NumberIn);
  ~FunctionToNumberEqualType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class FunctionToNumberGreaterOrEqualType :
  public FunctionToNumberConditionType
{
public:
  FunctionToNumberGreaterOrEqualType();
  FunctionToNumberGreaterOrEqualType(
    XmlID * NameIn,
    XmlIDREF * FunctionNameIn,
    XmlInteger * NumberIn);
  ~FunctionToNumberGreaterOrEqualType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class FunctionToNumberGreaterType :
  public FunctionToNumberConditionType
{
public:
  FunctionToNumberGreaterType();
  FunctionToNumberGreaterType(
    XmlID * NameIn,
    XmlIDREF * FunctionNameIn,
    XmlInteger * NumberIn);
  ~FunctionToNumberGreaterType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class FunctionToNumberLessOrEqualType :
  public FunctionToNumberConditionType
{
public:
  FunctionToNumberLessOrEqualType();
  FunctionToNumberLessOrEqualType(
    XmlID * NameIn,
    XmlIDREF * FunctionNameIn,
    XmlInteger * NumberIn);
  ~FunctionToNumberLessOrEqualType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class FunctionToNumberLessType :
  public FunctionToNumberConditionType
{
public:
  FunctionToNumberLessType();
  FunctionToNumberLessType(
    XmlID * NameIn,
    XmlIDREF * FunctionNameIn,
    XmlInteger * NumberIn);
  ~FunctionToNumberLessType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class FunctionType :
  public DataThingType
{
public:
  FunctionType();
  FunctionType(
    XmlID * NameIn,
    XmlString * DescriptionIn,
    std::list<FunctionParameterType *> * FunctionParameterIn);
  ~FunctionType();
  void PRINTSELFDECL;

  XmlString * Description;
  std::list<FunctionParameterType *> * FunctionParameter;

  bool printTypp;
};

/*********************************************************************/

class IncreaseType :
  public FunctionOperationType
{
public:
  IncreaseType();
  IncreaseType(
    XmlID * NameIn,
    XmlIDREF * FunctionNameIn,
    XmlInteger * ValueIn);
  ~IncreaseType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class IntermediateStateRelationType :
  public DataThingType
{
public:
  IntermediateStateRelationType();
  IntermediateStateRelationType(
    XmlID * NameIn,
    RCC8StateRelationType * RCC8StateRelationIn);
  ~IntermediateStateRelationType();
  void PRINTSELFDECL;

  RCC8StateRelationType * RCC8StateRelation;

  bool printTypp;
};

/*********************************************************************/

class NegativePredicateType :
  public DataThingType
{
public:
  NegativePredicateType();
  NegativePredicateType(
    XmlID * NameIn,
    XmlIDREF * PositivePredicateNameIn);
  ~NegativePredicateType();
  void PRINTSELFDECL;

  XmlIDREF * PositivePredicateName;

  bool printTypp;
};

/*********************************************************************/

class PositivePredicateType :
  public DataThingType
{
public:
  PositivePredicateType();
  PositivePredicateType(
    XmlID * NameIn,
    XmlString * DescriptionIn,
    std::list<PredicateParameterType *> * PredicateParameterIn,
    PredicateStateRelationOrType * PredicateStateRelationORIn,
    std::list<PredicateStateRelationType *> * PredicateStateRelationIn);
  ~PositivePredicateType();
  void PRINTSELFDECL;

  XmlString * Description;
  std::list<PredicateParameterType *> * PredicateParameter;
  PredicateStateRelationOrType * PredicateStateRelationOR;
  std::list<PredicateStateRelationType *> * PredicateStateRelation;

  bool printTypp;
};

/*********************************************************************/

class PreconditionType :
  public DataThingType
{
public:
  PreconditionType();
  PreconditionType(
    XmlID * NameIn,
    std::list<XmlIDREF *> * PositivePredicateNameIn,
    std::list<FunctionToNumberEqualType *> * FunctionToNumberEqualIn,
    std::list<FunctionToNumberGreaterOrEqualType *> * FunctionToNumberGreaterOrEqualIn,
    std::list<FunctionToNumberGreaterType *> * FunctionToNumberGreaterIn,
    std::list<FunctionToNumberLessType *> * FunctionToNumberLessIn,
    std::list<FunctionToNumberLessOrEqualType *> * FunctionToNumberLessOrEqualIn,
    std::list<FunctionToFunctionEqualType *> * FunctionToFunctionEqualIn,
    std::list<FunctionToFunctionGreaterOrEqualType *> * FunctionToFunctionGreaterOrEqualIn,
    std::list<FunctionToFunctionGreaterType *> * FunctionToFunctionGreaterIn,
    std::list<FunctionToFunctionLessType *> * FunctionToFunctionLessIn,
    std::list<FunctionToFunctionLessOrEqualType *> * FunctionToFunctionLessOrEqualIn);
  ~PreconditionType();
  void PRINTSELFDECL;

  std::list<XmlIDREF *> * PositivePredicateName;
  std::list<FunctionToNumberEqualType *> * FunctionToNumberEqual;
  std::list<FunctionToNumberGreaterOrEqualType *> * FunctionToNumberGreaterOrEqual;
  std::list<FunctionToNumberGreaterType *> * FunctionToNumberGreater;
  std::list<FunctionToNumberLessType *> * FunctionToNumberLess;
  std::list<FunctionToNumberLessOrEqualType *> * FunctionToNumberLessOrEqual;
  std::list<FunctionToFunctionEqualType *> * FunctionToFunctionEqual;
  std::list<FunctionToFunctionGreaterOrEqualType *> * FunctionToFunctionGreaterOrEqual;
  std::list<FunctionToFunctionGreaterType *> * FunctionToFunctionGreater;
  std::list<FunctionToFunctionLessType *> * FunctionToFunctionLess;
  std::list<FunctionToFunctionLessOrEqualType *> * FunctionToFunctionLessOrEqual;

  bool printTypp;
};

/*********************************************************************/

class PredicateParameterType :
  public DataThingType
{
public:
  PredicateParameterType();
  PredicateParameterType(
    XmlID * NameIn,
    XmlNMTOKEN * ParameterIn,
    XmlInteger * ParameterPositionIn);
  ~PredicateParameterType();
  void PRINTSELFDECL;

  XmlNMTOKEN * Parameter;
  XmlInteger * ParameterPosition;

  bool printTypp;
};

/*********************************************************************/

class PredicateStateRelationOrType :
  public DataThingType
{
public:
  PredicateStateRelationOrType();
  PredicateStateRelationOrType(
    XmlID * NameIn,
    std::list<PredicateStateRelationType *> * PredicateStateRelationIn);
  ~PredicateStateRelationOrType();
  void PRINTSELFDECL;

  std::list<PredicateStateRelationType *> * PredicateStateRelation;

  bool printTypp;
};

/*********************************************************************/

class PredicateStateRelationType :
  public DataThingType
{
public:
  PredicateStateRelationType();
  PredicateStateRelationType(
    XmlID * NameIn,
    XmlNMTOKEN * ReferenceParameterIn,
    XmlNMTOKEN * TargetParameterIn,
    XmlIDREF * IntermediateStateRelationNameIn);
  ~PredicateStateRelationType();
  void PRINTSELFDECL;

  XmlNMTOKEN * ReferenceParameter;
  XmlNMTOKEN * TargetParameter;
  XmlIDREF * IntermediateStateRelationName;

  bool printTypp;
};

/*********************************************************************/

class RCC8StateRelationType :
  public DataThingType
{
public:
  RCC8StateRelationType();
  RCC8StateRelationType(
    XmlID * NameIn,
    XmlString * RCC8SetIn);
  ~RCC8StateRelationType();
  void PRINTSELFDECL;

  XmlString * RCC8Set;

  bool printTypp;
};

/*********************************************************************/

class SOAPType :
  public DataThingType
{
public:
  SOAPType();
  SOAPType(
    XmlID * NameIn,
    KittingWorkstationType * KittingWorkstationIn,
    std::list<IntermediateStateRelationType *> * IntermediateStateRelationIn,
    DomainType * DomainIn);
  ~SOAPType();
  void PRINTSELFDECL;

  KittingWorkstationType * KittingWorkstation;
  std::list<IntermediateStateRelationType *> * IntermediateStateRelation;
  DomainType * Domain;

  bool printTypp;
};

/*********************************************************************/

class DecreaseType :
  public FunctionOperationType
{
public:
  DecreaseType();
  DecreaseType(
    XmlID * NameIn,
    XmlIDREF * FunctionNameIn,
    XmlInteger * ValueIn);
  ~DecreaseType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class XmlHeaderForSOAP
{
public:
  XmlHeaderForSOAP();
  XmlHeaderForSOAP(
    SchemaLocation * locationIn);
  ~XmlHeaderForSOAP();
  void PRINTSELFDECL;

  SchemaLocation * location;
};

/*********************************************************************/

#endif // SOAP_HH
