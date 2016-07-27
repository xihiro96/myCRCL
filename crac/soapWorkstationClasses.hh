/*********************************************************************/

#ifndef SOAPWORKSTATION_HH
#define SOAPWORKSTATION_HH
#include <stdio.h>
#include <list>
#include "crac/xmlSchemaInstance.hh"

/*********************************************************************/

class AngleUnitType;
class BoxVolumeType;
class BoxyShapeType;
class CylindricalShapeType;
class DataThingType;
class EndEffectorChangingStationType;
class EndEffectorGraspType;
class EndEffectorHolderType;
class EndEffectorType;
class ExternalShapeType;
class ForceUnitType;
class GripperEffectorType;
class GripperOtherType;
class GripperParallelType;
class GripperThreeFingerType;
class GripperTwoJawsType;
class HumanType;
class InternalShapeType;
class KitDesignType;
class KitTrayType;
class KitType;
class KittingWorkstationType;
class LargeBoxWithEmptyKitTraysType;
class LargeBoxWithKitsSlotType;
class LargeBoxWithKitsType;
class LargeContainerType;
class LengthUnitType;
class MarkingLayoutType;
class MarkingModelType;
class MarkingType;
class MechanicalComponentType;
class NoSkuObjectType;
class OtherGripperGraspType;
class ParallelGripperGraspType;
class PartRefAndPoseType;
class PartType;
class PartsBinType;
class PartsTrayType;
class PartsVesselType;
class PhysicalLocationType;
class PointType;
class PoseLocationInType;
class PoseLocationOnType;
class PoseLocationType;
class PoseOnlyLocationType;
class PositiveDecimalType;
class RelativeLocationInType;
class RelativeLocationOnType;
class RelativeLocationType;
class RobotType;
class ShapeDesignType;
class SkuObjectType;
class SlotType;
class SolidObjectType;
class StockKeepingUnitType;
class ThreeFingerGraspType;
class TwoJawsGraspType;
class VacuumEffectorMultiCupType;
class VacuumEffectorSingleCupType;
class VacuumEffectorSingleGraspType;
class VacuumEffectorType;
class VectorType;
class WeightUnitType;
class WorkTableAreaType;
class WorkTableType;

/*********************************************************************/
/*********************************************************************/

class AngleUnitType :
  public XmlNMTOKEN
{
public:
  AngleUnitType();
  AngleUnitType(
    const char * valIn);
  ~AngleUnitType();
  bool AngleUnitTypeIsBad();
  void PRINTSELFDECL;
};

/*********************************************************************/

class DataThingType :
  public XmlSchemaInstanceBase
{
public:
  DataThingType();
  DataThingType(
    XmlID * NameIn);
  ~DataThingType();
  void PRINTSELFDECL;

  XmlID * Name;
};

/*********************************************************************/

class EndEffectorGraspType :
  public DataThingType
{
public:
  EndEffectorGraspType();
  EndEffectorGraspType(
    XmlID * NameIn,
    PoseOnlyLocationType * HeldObjectOffsetIn);
  ~EndEffectorGraspType();
  void PRINTSELFDECL;

  PoseOnlyLocationType * HeldObjectOffset;

  bool printTypp;
};

/*********************************************************************/

class ForceUnitType :
  public XmlNMTOKEN
{
public:
  ForceUnitType();
  ForceUnitType(
    const char * valIn);
  ~ForceUnitType();
  bool ForceUnitTypeIsBad();
  void PRINTSELFDECL;
};

/*********************************************************************/

class KitDesignType :
  public DataThingType
{
public:
  KitDesignType();
  KitDesignType(
    XmlID * NameIn,
    XmlIDREF * KitTraySkuNameIn,
    std::list<PartRefAndPoseType *> * PartRefAndPoseIn);
  ~KitDesignType();
  void PRINTSELFDECL;

  XmlIDREF * KitTraySkuName;
  std::list<PartRefAndPoseType *> * PartRefAndPose;

  bool printTypp;
};

/*********************************************************************/

class LargeBoxWithKitsSlotType :
  public DataThingType
{
public:
  LargeBoxWithKitsSlotType();
  LargeBoxWithKitsSlotType(
    XmlID * NameIn,
    XmlInteger * RankIn,
    std::list<XmlIDREF *> * KitNameIn);
  ~LargeBoxWithKitsSlotType();
  void PRINTSELFDECL;

  XmlInteger * Rank;
  std::list<XmlIDREF *> * KitName;

  bool printTypp;
};

/*********************************************************************/

class LengthUnitType :
  public XmlNMTOKEN
{
public:
  LengthUnitType();
  LengthUnitType(
    const char * valIn);
  ~LengthUnitType();
  bool LengthUnitTypeIsBad();
  void PRINTSELFDECL;
};

/*********************************************************************/

class MarkingLayoutType :
  public DataThingType
{
public:
  MarkingLayoutType();
  MarkingLayoutType(
    XmlID * NameIn,
    PointType * PointIn,
    VectorType * XAxisIn,
    VectorType * YAxisIn);
  ~MarkingLayoutType();
  void PRINTSELFDECL;

  PointType * Point;
  VectorType * XAxis;
  VectorType * YAxis;

  bool printTypp;
};

/*********************************************************************/

class MarkingModelType :
  public DataThingType
{
public:
  MarkingModelType();
  MarkingModelType(
    XmlID * NameIn,
    XmlString * MarkingFormatNameIn,
    XmlString * MarkingFileNameIn,
    XmlString * MarkingNameIn);
  ~MarkingModelType();
  void PRINTSELFDECL;

  XmlString * MarkingFormatName;
  XmlString * MarkingFileName;
  XmlString * MarkingName;

  bool printTypp;
};

/*********************************************************************/

class MarkingType :
  public DataThingType
{
public:
  MarkingType();
  MarkingType(
    XmlID * NameIn,
    MarkingLayoutType * MarkingLayoutIn,
    MarkingModelType * MarkingModelIn);
  ~MarkingType();
  void PRINTSELFDECL;

  MarkingLayoutType * MarkingLayout;
  MarkingModelType * MarkingModel;

  bool printTypp;
};

/*********************************************************************/

class OtherGripperGraspType :
  public EndEffectorGraspType
{
public:
  OtherGripperGraspType();
  OtherGripperGraspType(
    XmlID * NameIn,
    PoseOnlyLocationType * HeldObjectOffsetIn,
    XmlIDREF * OtherGripperNameIn,
    std::list<PoseLocationType *> * GraspPoseIn,
    PositiveDecimalType * MaxForceIn,
    PositiveDecimalType * MinForceIn);
  ~OtherGripperGraspType();
  void PRINTSELFDECL;

  XmlIDREF * OtherGripperName;
  std::list<PoseLocationType *> * GraspPose;
  PositiveDecimalType * MaxForce;
  PositiveDecimalType * MinForce;

  bool printTypp;
};

/*********************************************************************/

class ParallelGripperGraspType :
  public EndEffectorGraspType
{
public:
  ParallelGripperGraspType();
  ParallelGripperGraspType(
    XmlID * NameIn,
    PoseOnlyLocationType * HeldObjectOffsetIn,
    XmlIDREF * ParallelGripperNameIn,
    PoseLocationType * GraspPoseIn,
    PositiveDecimalType * ApproachSeparationIn,
    PositiveDecimalType * GraspSeparationIn);
  ~ParallelGripperGraspType();
  void PRINTSELFDECL;

  XmlIDREF * ParallelGripperName;
  PoseLocationType * GraspPose;
  PositiveDecimalType * ApproachSeparation;
  PositiveDecimalType * GraspSeparation;

  bool printTypp;
};

/*********************************************************************/

class PartRefAndPoseType :
  public DataThingType
{
public:
  PartRefAndPoseType();
  PartRefAndPoseType(
    XmlID * NameIn,
    XmlIDREF * SkuNameIn,
    PointType * PointIn,
    VectorType * XAxisIn,
    VectorType * ZAxisIn);
  ~PartRefAndPoseType();
  void PRINTSELFDECL;

  XmlIDREF * SkuName;
  PointType * Point;
  VectorType * XAxis;
  VectorType * ZAxis;

  bool printTypp;
};

/*********************************************************************/

class PhysicalLocationType :
  public DataThingType
{
public:
  PhysicalLocationType();
  PhysicalLocationType(
    XmlID * NameIn,
    XmlIDREF * RefObjectNameIn,
    XmlIDREF * RefDataNameIn,
    XmlDateTime * TimestampIn);
  ~PhysicalLocationType();
  void PRINTSELFDECL;

  XmlIDREF * RefObjectName;
  XmlIDREF * RefDataName;
  XmlDateTime * Timestamp;

  bool printTypp;
};

/*********************************************************************/

class PointType :
  public DataThingType
{
public:
  PointType();
  PointType(
    XmlID * NameIn,
    XmlDecimal * XIn,
    XmlDecimal * YIn,
    XmlDecimal * ZIn);
  ~PointType();
  void PRINTSELFDECL;

  XmlDecimal * X;
  XmlDecimal * Y;
  XmlDecimal * Z;

  bool printTypp;
};

/*********************************************************************/

class PoseLocationType :
  public PhysicalLocationType
{
public:
  PoseLocationType();
  PoseLocationType(
    XmlID * NameIn,
    XmlIDREF * RefObjectNameIn,
    XmlIDREF * RefDataNameIn,
    XmlDateTime * TimestampIn,
    PointType * PointIn,
    VectorType * XAxisIn,
    VectorType * ZAxisIn,
    PositiveDecimalType * PositionStandardDeviationIn,
    PositiveDecimalType * OrientationStandardDeviationIn);
  ~PoseLocationType();
  void PRINTSELFDECL;

  PointType * Point;
  VectorType * XAxis;
  VectorType * ZAxis;
  PositiveDecimalType * PositionStandardDeviation;
  PositiveDecimalType * OrientationStandardDeviation;

  bool printTypp;
};

/*********************************************************************/

class PoseOnlyLocationType :
  public PoseLocationType
{
public:
  PoseOnlyLocationType();
  PoseOnlyLocationType(
    XmlID * NameIn,
    XmlIDREF * RefObjectNameIn,
    XmlIDREF * RefDataNameIn,
    XmlDateTime * TimestampIn,
    PointType * PointIn,
    VectorType * XAxisIn,
    VectorType * ZAxisIn,
    PositiveDecimalType * PositionStandardDeviationIn,
    PositiveDecimalType * OrientationStandardDeviationIn);
  ~PoseOnlyLocationType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class PositiveDecimalType :
  public XmlDecimal
{
public:
  PositiveDecimalType();
  PositiveDecimalType(
    const char * valIn);
  ~PositiveDecimalType();
  bool PositiveDecimalTypeIsBad();
  void PRINTSELFDECL;
};

/*********************************************************************/

class RelativeLocationType :
  public PhysicalLocationType
{
public:
  RelativeLocationType();
  RelativeLocationType(
    XmlID * NameIn,
    XmlIDREF * RefObjectNameIn,
    XmlIDREF * RefDataNameIn,
    XmlDateTime * TimestampIn,
    XmlString * DescriptionIn);
  ~RelativeLocationType();
  void PRINTSELFDECL;

  XmlString * Description;

  bool printTypp;
};

/*********************************************************************/

class ShapeDesignType :
  public DataThingType
{
public:
  ShapeDesignType();
  ShapeDesignType(
    XmlID * NameIn,
    XmlString * DescriptionIn,
    std::list<MarkingType *> * MarkingIn);
  ~ShapeDesignType();
  void PRINTSELFDECL;

  XmlString * Description;
  std::list<MarkingType *> * Marking;

  bool printTypp;
};

/*********************************************************************/

class SlotType :
  public DataThingType
{
public:
  SlotType();
  SlotType(
    XmlID * NameIn,
    XmlIDREF * PartRefAndPoseNameIn,
    XmlIDREF * PartNameIn);
  ~SlotType();
  void PRINTSELFDECL;

  XmlIDREF * PartRefAndPoseName;
  XmlIDREF * PartName;

  bool printTypp;
};

/*********************************************************************/

class SolidObjectType :
  public XmlSchemaInstanceBase
{
public:
  SolidObjectType();
  SolidObjectType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn);
  ~SolidObjectType();
  void PRINTSELFDECL;

  XmlID * Name;
  PhysicalLocationType * PrimaryLocation;
  std::list<PhysicalLocationType *> * SecondaryLocation;
};

/*********************************************************************/

class StockKeepingUnitType :
  public DataThingType
{
public:
  StockKeepingUnitType();
  StockKeepingUnitType(
    XmlID * NameIn,
    XmlString * DescriptionIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn);
  ~StockKeepingUnitType();
  void PRINTSELFDECL;

  XmlString * Description;
  InternalShapeType * InternalShape;
  ExternalShapeType * ExternalShape;
  PositiveDecimalType * Weight;
  std::list<EndEffectorGraspType *> * EffectorAndGrasp;

  bool printTypp;
};

/*********************************************************************/

class ThreeFingerGraspType :
  public EndEffectorGraspType
{
public:
  ThreeFingerGraspType();
  ThreeFingerGraspType(
    XmlID * NameIn,
    PoseOnlyLocationType * HeldObjectOffsetIn,
    XmlIDREF * ThreeFingerGripperNameIn,
    PoseLocationType * GraspPose1In,
    PoseLocationType * GraspPose2In,
    PoseLocationType * GraspPose3In,
    PositiveDecimalType * MaxForceIn,
    PositiveDecimalType * MinForceIn);
  ~ThreeFingerGraspType();
  void PRINTSELFDECL;

  XmlIDREF * ThreeFingerGripperName;
  PoseLocationType * GraspPose1;
  PoseLocationType * GraspPose2;
  PoseLocationType * GraspPose3;
  PositiveDecimalType * MaxForce;
  PositiveDecimalType * MinForce;

  bool printTypp;
};

/*********************************************************************/

class TwoJawsGraspType :
  public EndEffectorGraspType
{
public:
  TwoJawsGraspType();
  TwoJawsGraspType(
    XmlID * NameIn,
    PoseOnlyLocationType * HeldObjectOffsetIn,
    XmlIDREF * TwoJawsGripperNameIn,
    PoseLocationType * GraspPose1In,
    PoseLocationType * GraspPose2In,
    PositiveDecimalType * MaxForceIn,
    PositiveDecimalType * MinForceIn);
  ~TwoJawsGraspType();
  void PRINTSELFDECL;

  XmlIDREF * TwoJawsGripperName;
  PoseLocationType * GraspPose1;
  PoseLocationType * GraspPose2;
  PositiveDecimalType * MaxForce;
  PositiveDecimalType * MinForce;

  bool printTypp;
};

/*********************************************************************/

class VacuumEffectorSingleGraspType :
  public EndEffectorGraspType
{
public:
  VacuumEffectorSingleGraspType();
  VacuumEffectorSingleGraspType(
    XmlID * NameIn,
    PoseOnlyLocationType * HeldObjectOffsetIn,
    XmlIDREF * VacuumEffectorSingleNameIn,
    PoseLocationType * GraspPoseIn);
  ~VacuumEffectorSingleGraspType();
  void PRINTSELFDECL;

  XmlIDREF * VacuumEffectorSingleName;
  PoseLocationType * GraspPose;

  bool printTypp;
};

/*********************************************************************/

class VectorType :
  public DataThingType
{
public:
  VectorType();
  VectorType(
    XmlID * NameIn,
    XmlDecimal * IIn,
    XmlDecimal * JIn,
    XmlDecimal * KIn);
  ~VectorType();
  void PRINTSELFDECL;

  XmlDecimal * I;
  XmlDecimal * J;
  XmlDecimal * K;

  bool printTypp;
};

/*********************************************************************/

class WeightUnitType :
  public XmlNMTOKEN
{
public:
  WeightUnitType();
  WeightUnitType(
    const char * valIn);
  ~WeightUnitType();
  bool WeightUnitTypeIsBad();
  void PRINTSELFDECL;
};

/*********************************************************************/

class WorkTableAreaType :
  public DataThingType
{
public:
  WorkTableAreaType();
  WorkTableAreaType(
    XmlID * NameIn,
    XmlDecimal * XminIn,
    XmlDecimal * YminIn,
    XmlDecimal * XmaxIn,
    XmlDecimal * YmaxIn,
    XmlIDREF * SkuNameIn,
    XmlIDREF * SolidObjectNameIn);
  ~WorkTableAreaType();
  void PRINTSELFDECL;

  XmlDecimal * Xmin;
  XmlDecimal * Ymin;
  XmlDecimal * Xmax;
  XmlDecimal * Ymax;
  XmlIDREF * SkuName;
  XmlIDREF * SolidObjectName;

  bool printTypp;
};

/*********************************************************************/

class BoxVolumeType :
  public DataThingType
{
public:
  BoxVolumeType();
  BoxVolumeType(
    XmlID * NameIn,
    PointType * MaximumPointIn,
    PointType * MinimumPointIn);
  ~BoxVolumeType();
  void PRINTSELFDECL;

  PointType * MaximumPoint;
  PointType * MinimumPoint;

  bool printTypp;
};

/*********************************************************************/

class ExternalShapeType :
  public ShapeDesignType
{
public:
  ExternalShapeType();
  ExternalShapeType(
    XmlID * NameIn,
    XmlString * DescriptionIn,
    std::list<MarkingType *> * MarkingIn,
    XmlString * ModelFormatNameIn,
    XmlString * ModelFileNameIn,
    XmlString * ModelNameIn,
    XmlString * ModelIllustrationNameIn);
  ~ExternalShapeType();
  void PRINTSELFDECL;

  XmlString * ModelFormatName;
  XmlString * ModelFileName;
  XmlString * ModelName;
  XmlString * ModelIllustrationName;

  bool printTypp;
};

/*********************************************************************/

class InternalShapeType :
  public ShapeDesignType
{
public:
  InternalShapeType();
  InternalShapeType(
    XmlID * NameIn,
    XmlString * DescriptionIn,
    std::list<MarkingType *> * MarkingIn);
  ~InternalShapeType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class NoSkuObjectType :
  public SolidObjectType
{
public:
  NoSkuObjectType();
  NoSkuObjectType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn);
  ~NoSkuObjectType();
  void PRINTSELFDECL;

  InternalShapeType * InternalShape;
  ExternalShapeType * ExternalShape;
  PositiveDecimalType * Weight;
  std::list<EndEffectorGraspType *> * EffectorAndGrasp;

  bool printTypp;
};

/*********************************************************************/

class PoseLocationInType :
  public PoseLocationType
{
public:
  PoseLocationInType();
  PoseLocationInType(
    XmlID * NameIn,
    XmlIDREF * RefObjectNameIn,
    XmlIDREF * RefDataNameIn,
    XmlDateTime * TimestampIn,
    PointType * PointIn,
    VectorType * XAxisIn,
    VectorType * ZAxisIn,
    PositiveDecimalType * PositionStandardDeviationIn,
    PositiveDecimalType * OrientationStandardDeviationIn);
  ~PoseLocationInType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class PoseLocationOnType :
  public PoseLocationType
{
public:
  PoseLocationOnType();
  PoseLocationOnType(
    XmlID * NameIn,
    XmlIDREF * RefObjectNameIn,
    XmlIDREF * RefDataNameIn,
    XmlDateTime * TimestampIn,
    PointType * PointIn,
    VectorType * XAxisIn,
    VectorType * ZAxisIn,
    PositiveDecimalType * PositionStandardDeviationIn,
    PositiveDecimalType * OrientationStandardDeviationIn);
  ~PoseLocationOnType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class RelativeLocationInType :
  public RelativeLocationType
{
public:
  RelativeLocationInType();
  RelativeLocationInType(
    XmlID * NameIn,
    XmlIDREF * RefObjectNameIn,
    XmlIDREF * RefDataNameIn,
    XmlDateTime * TimestampIn,
    XmlString * DescriptionIn);
  ~RelativeLocationInType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class RelativeLocationOnType :
  public RelativeLocationType
{
public:
  RelativeLocationOnType();
  RelativeLocationOnType(
    XmlID * NameIn,
    XmlIDREF * RefObjectNameIn,
    XmlIDREF * RefDataNameIn,
    XmlDateTime * TimestampIn,
    XmlString * DescriptionIn);
  ~RelativeLocationOnType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class RobotType :
  public NoSkuObjectType
{
public:
  RobotType();
  RobotType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn,
    XmlString * DescriptionIn,
    EndEffectorType * EndEffectorIn,
    PositiveDecimalType * MaximumLoadWeightIn,
    std::list<BoxVolumeType *> * WorkVolumeIn);
  ~RobotType();
  void PRINTSELFDECL;

  XmlString * Description;
  EndEffectorType * EndEffector;
  PositiveDecimalType * MaximumLoadWeight;
  std::list<BoxVolumeType *> * WorkVolume;

  bool printTypp;
};

/*********************************************************************/

class SkuObjectType :
  public SolidObjectType
{
public:
  SkuObjectType();
  SkuObjectType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    XmlIDREF * SkuNameIn);
  ~SkuObjectType();
  void PRINTSELFDECL;

  XmlIDREF * SkuName;

  bool printTypp;
};

/*********************************************************************/

class WorkTableType :
  public NoSkuObjectType
{
public:
  WorkTableType();
  WorkTableType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn,
    std::list<SolidObjectType *> * ObjectOnTableIn,
    std::list<WorkTableAreaType *> * WorkTableAreaIn);
  ~WorkTableType();
  void PRINTSELFDECL;

  std::list<SolidObjectType *> * ObjectOnTable;
  std::list<WorkTableAreaType *> * WorkTableArea;

  bool printTypp;
};

/*********************************************************************/

class BoxyShapeType :
  public InternalShapeType
{
public:
  BoxyShapeType();
  BoxyShapeType(
    XmlID * NameIn,
    XmlString * DescriptionIn,
    std::list<MarkingType *> * MarkingIn,
    PositiveDecimalType * LengthIn,
    PositiveDecimalType * WidthIn,
    PositiveDecimalType * HeightIn,
    XmlBoolean * HasTopIn);
  ~BoxyShapeType();
  void PRINTSELFDECL;

  PositiveDecimalType * Length;
  PositiveDecimalType * Width;
  PositiveDecimalType * Height;
  XmlBoolean * HasTop;

  bool printTypp;
};

/*********************************************************************/

class CylindricalShapeType :
  public InternalShapeType
{
public:
  CylindricalShapeType();
  CylindricalShapeType(
    XmlID * NameIn,
    XmlString * DescriptionIn,
    std::list<MarkingType *> * MarkingIn,
    PositiveDecimalType * DiameterIn,
    PositiveDecimalType * HeightIn,
    XmlBoolean * HasTopIn);
  ~CylindricalShapeType();
  void PRINTSELFDECL;

  PositiveDecimalType * Diameter;
  PositiveDecimalType * Height;
  XmlBoolean * HasTop;

  bool printTypp;
};

/*********************************************************************/

class EndEffectorChangingStationType :
  public NoSkuObjectType
{
public:
  EndEffectorChangingStationType();
  EndEffectorChangingStationType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn,
    MechanicalComponentType * BaseIn,
    std::list<EndEffectorHolderType *> * EndEffectorHolderIn);
  ~EndEffectorChangingStationType();
  void PRINTSELFDECL;

  MechanicalComponentType * Base;
  std::list<EndEffectorHolderType *> * EndEffectorHolder;

  bool printTypp;
};

/*********************************************************************/

class EndEffectorHolderType :
  public NoSkuObjectType
{
public:
  EndEffectorHolderType();
  EndEffectorHolderType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn,
    std::list<XmlIDREF *> * CanHoldEndEffectorNameIn,
    EndEffectorType * EndEffectorIn);
  ~EndEffectorHolderType();
  void PRINTSELFDECL;

  std::list<XmlIDREF *> * CanHoldEndEffectorName;
  EndEffectorType * EndEffector;

  bool printTypp;
};

/*********************************************************************/

class EndEffectorType :
  public NoSkuObjectType
{
public:
  EndEffectorType();
  EndEffectorType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn,
    XmlString * DescriptionIn,
    PositiveDecimalType * MaximumLoadWeightIn,
    SolidObjectType * HeldObjectIn);
  ~EndEffectorType();
  void PRINTSELFDECL;

  XmlString * Description;
  PositiveDecimalType * MaximumLoadWeight;
  SolidObjectType * HeldObject;

  bool printTypp;
};

/*********************************************************************/

class GripperEffectorType :
  public EndEffectorType
{
public:
  GripperEffectorType();
  GripperEffectorType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn,
    XmlString * DescriptionIn,
    PositiveDecimalType * MaximumLoadWeightIn,
    SolidObjectType * HeldObjectIn,
    PositiveDecimalType * MaxGripWidthIn);
  ~GripperEffectorType();
  void PRINTSELFDECL;

  PositiveDecimalType * MaxGripWidth;

  bool printTypp;
};

/*********************************************************************/

class GripperOtherType :
  public GripperEffectorType
{
public:
  GripperOtherType();
  GripperOtherType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn,
    XmlString * DescriptionIn,
    PositiveDecimalType * MaximumLoadWeightIn,
    SolidObjectType * HeldObjectIn,
    PositiveDecimalType * MaxGripWidthIn);
  ~GripperOtherType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class GripperParallelType :
  public GripperEffectorType
{
public:
  GripperParallelType();
  GripperParallelType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn,
    XmlString * DescriptionIn,
    PositiveDecimalType * MaximumLoadWeightIn,
    SolidObjectType * HeldObjectIn,
    PositiveDecimalType * MaxGripWidthIn);
  ~GripperParallelType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class GripperThreeFingerType :
  public GripperEffectorType
{
public:
  GripperThreeFingerType();
  GripperThreeFingerType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn,
    XmlString * DescriptionIn,
    PositiveDecimalType * MaximumLoadWeightIn,
    SolidObjectType * HeldObjectIn,
    PositiveDecimalType * MaxGripWidthIn);
  ~GripperThreeFingerType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class GripperTwoJawsType :
  public GripperEffectorType
{
public:
  GripperTwoJawsType();
  GripperTwoJawsType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn,
    XmlString * DescriptionIn,
    PositiveDecimalType * MaximumLoadWeightIn,
    SolidObjectType * HeldObjectIn,
    PositiveDecimalType * MaxGripWidthIn);
  ~GripperTwoJawsType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class HumanType :
  public NoSkuObjectType
{
public:
  HumanType();
  HumanType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn);
  ~HumanType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class KitTrayType :
  public SkuObjectType
{
public:
  KitTrayType();
  KitTrayType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    XmlIDREF * SkuNameIn,
    XmlNMTOKEN * SerialNumberIn);
  ~KitTrayType();
  void PRINTSELFDECL;

  XmlNMTOKEN * SerialNumber;

  bool printTypp;
};

/*********************************************************************/

class KitType :
  public NoSkuObjectType
{
public:
  KitType();
  KitType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn,
    XmlIDREF * DesignNameIn,
    KitTrayType * KitTrayIn,
    std::list<PartType *> * PartIn,
    std::list<SlotType *> * SlotIn,
    XmlBoolean * FinishedIn);
  ~KitType();
  void PRINTSELFDECL;

  XmlIDREF * DesignName;
  KitTrayType * KitTray;
  std::list<PartType *> * Part;
  std::list<SlotType *> * Slot;
  XmlBoolean * Finished;

  bool printTypp;
};

/*********************************************************************/

class KittingWorkstationType :
  public NoSkuObjectType
{
public:
  KittingWorkstationType();
  KittingWorkstationType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn,
    AngleUnitType * AngleUnitIn,
    EndEffectorChangingStationType * ChangingStationIn,
    ForceUnitType * ForceUnitIn,
    std::list<KitDesignType *> * KitDesignIn,
    LengthUnitType * LengthUnitIn,
    std::list<SolidObjectType *> * ObjectIn,
    std::list<BoxVolumeType *> * OtherObstacleIn,
    RobotType * RobotIn,
    std::list<StockKeepingUnitType *> * SkuIn,
    WeightUnitType * WeightUnitIn);
  ~KittingWorkstationType();
  void PRINTSELFDECL;

  AngleUnitType * AngleUnit;
  EndEffectorChangingStationType * ChangingStation;
  ForceUnitType * ForceUnit;
  std::list<KitDesignType *> * KitDesign;
  LengthUnitType * LengthUnit;
  std::list<SolidObjectType *> * Object;
  std::list<BoxVolumeType *> * OtherObstacle;
  RobotType * Robot;
  std::list<StockKeepingUnitType *> * Sku;
  WeightUnitType * WeightUnit;

  bool printTypp;
};

/*********************************************************************/

class LargeBoxWithEmptyKitTraysType :
  public NoSkuObjectType
{
public:
  LargeBoxWithEmptyKitTraysType();
  LargeBoxWithEmptyKitTraysType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn,
    LargeContainerType * LargeContainerIn,
    std::list<KitTrayType *> * KitTrayIn);
  ~LargeBoxWithEmptyKitTraysType();
  void PRINTSELFDECL;

  LargeContainerType * LargeContainer;
  std::list<KitTrayType *> * KitTray;

  bool printTypp;
};

/*********************************************************************/

class LargeBoxWithKitsType :
  public NoSkuObjectType
{
public:
  LargeBoxWithKitsType();
  LargeBoxWithKitsType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn,
    LargeContainerType * LargeContainerIn,
    std::list<KitType *> * KitIn,
    XmlIDREF * KitDesignNameIn,
    std::list<LargeBoxWithKitsSlotType *> * LargeBoxWithKitsSlotIn,
    XmlPositiveInteger * CapacityIn);
  ~LargeBoxWithKitsType();
  void PRINTSELFDECL;

  LargeContainerType * LargeContainer;
  std::list<KitType *> * Kit;
  XmlIDREF * KitDesignName;
  std::list<LargeBoxWithKitsSlotType *> * LargeBoxWithKitsSlot;
  XmlPositiveInteger * Capacity;

  bool printTypp;
};

/*********************************************************************/

class LargeContainerType :
  public SkuObjectType
{
public:
  LargeContainerType();
  LargeContainerType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    XmlIDREF * SkuNameIn,
    XmlNMTOKEN * SerialNumberIn);
  ~LargeContainerType();
  void PRINTSELFDECL;

  XmlNMTOKEN * SerialNumber;

  bool printTypp;
};

/*********************************************************************/

class MechanicalComponentType :
  public NoSkuObjectType
{
public:
  MechanicalComponentType();
  MechanicalComponentType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn);
  ~MechanicalComponentType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class PartType :
  public SkuObjectType
{
public:
  PartType();
  PartType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    XmlIDREF * SkuNameIn,
    XmlNMTOKEN * SerialNumberIn);
  ~PartType();
  void PRINTSELFDECL;

  XmlNMTOKEN * SerialNumber;

  bool printTypp;
};

/*********************************************************************/

class PartsVesselType :
  public SkuObjectType
{
public:
  PartsVesselType();
  PartsVesselType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    XmlIDREF * SkuNameIn,
    XmlNMTOKEN * SerialNumberIn,
    XmlIDREF * PartSkuNameIn,
    XmlNonNegativeInteger * PartQuantityIn,
    std::list<PartType *> * PartIn);
  ~PartsVesselType();
  void PRINTSELFDECL;

  XmlNMTOKEN * SerialNumber;
  XmlIDREF * PartSkuName;
  XmlNonNegativeInteger * PartQuantity;
  std::list<PartType *> * Part;

  bool printTypp;
};

/*********************************************************************/

class VacuumEffectorType :
  public EndEffectorType
{
public:
  VacuumEffectorType();
  VacuumEffectorType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn,
    XmlString * DescriptionIn,
    PositiveDecimalType * MaximumLoadWeightIn,
    SolidObjectType * HeldObjectIn,
    PositiveDecimalType * CupDiameterIn,
    PositiveDecimalType * LengthIn);
  ~VacuumEffectorType();
  void PRINTSELFDECL;

  PositiveDecimalType * CupDiameter;
  PositiveDecimalType * Length;

  bool printTypp;
};

/*********************************************************************/

class PartsBinType :
  public PartsVesselType
{
public:
  PartsBinType();
  PartsBinType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    XmlIDREF * SkuNameIn,
    XmlNMTOKEN * SerialNumberIn,
    XmlIDREF * PartSkuNameIn,
    XmlNonNegativeInteger * PartQuantityIn,
    std::list<PartType *> * PartIn);
  ~PartsBinType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class PartsTrayType :
  public PartsVesselType
{
public:
  PartsTrayType();
  PartsTrayType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    XmlIDREF * SkuNameIn,
    XmlNMTOKEN * SerialNumberIn,
    XmlIDREF * PartSkuNameIn,
    XmlNonNegativeInteger * PartQuantityIn,
    std::list<PartType *> * PartIn);
  ~PartsTrayType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

class VacuumEffectorMultiCupType :
  public VacuumEffectorType
{
public:
  VacuumEffectorMultiCupType();
  VacuumEffectorMultiCupType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn,
    XmlString * DescriptionIn,
    PositiveDecimalType * MaximumLoadWeightIn,
    SolidObjectType * HeldObjectIn,
    PositiveDecimalType * CupDiameterIn,
    PositiveDecimalType * LengthIn,
    XmlPositiveInteger * ArrayNumberIn,
    PositiveDecimalType * ArrayRadiusIn);
  ~VacuumEffectorMultiCupType();
  void PRINTSELFDECL;

  XmlPositiveInteger * ArrayNumber;
  PositiveDecimalType * ArrayRadius;

  bool printTypp;
};

/*********************************************************************/

class VacuumEffectorSingleCupType :
  public VacuumEffectorType
{
public:
  VacuumEffectorSingleCupType();
  VacuumEffectorSingleCupType(
    XmlID * NameIn,
    PhysicalLocationType * PrimaryLocationIn,
    std::list<PhysicalLocationType *> * SecondaryLocationIn,
    InternalShapeType * InternalShapeIn,
    ExternalShapeType * ExternalShapeIn,
    PositiveDecimalType * WeightIn,
    std::list<EndEffectorGraspType *> * EffectorAndGraspIn,
    XmlString * DescriptionIn,
    PositiveDecimalType * MaximumLoadWeightIn,
    SolidObjectType * HeldObjectIn,
    PositiveDecimalType * CupDiameterIn,
    PositiveDecimalType * LengthIn);
  ~VacuumEffectorSingleCupType();
  void PRINTSELFDECL;


  bool printTypp;
};

/*********************************************************************/

#endif // SOAPWORKSTATION_HH
