
"use strict";

let TimeRange = require('./TimeRange.js');
let Histogram = require('./Histogram.js');
let PlotDataArray = require('./PlotDataArray.js');
let DepthCalibrationParameter = require('./DepthCalibrationParameter.js');
let BoundingBoxArrayWithCameraInfo = require('./BoundingBoxArrayWithCameraInfo.js');
let PolygonArray = require('./PolygonArray.js');
let ParallelEdge = require('./ParallelEdge.js');
let BoundingBoxMovement = require('./BoundingBoxMovement.js');
let Spectrum = require('./Spectrum.js');
let Circle2DArray = require('./Circle2DArray.js');
let HumanSkeletonArray = require('./HumanSkeletonArray.js');
let VectorArray = require('./VectorArray.js');
let Rect = require('./Rect.js');
let Torus = require('./Torus.js');
let SparseOccupancyGridCell = require('./SparseOccupancyGridCell.js');
let SparseOccupancyGridArray = require('./SparseOccupancyGridArray.js');
let SegmentStamped = require('./SegmentStamped.js');
let HistogramWithRangeArray = require('./HistogramWithRangeArray.js');
let SegmentArray = require('./SegmentArray.js');
let BoolStamped = require('./BoolStamped.js');
let LabelArray = require('./LabelArray.js');
let SimpleOccupancyGridArray = require('./SimpleOccupancyGridArray.js');
let ModelCoefficientsArray = require('./ModelCoefficientsArray.js');
let Circle2D = require('./Circle2D.js');
let HumanSkeleton = require('./HumanSkeleton.js');
let BoundingBoxArray = require('./BoundingBoxArray.js');
let Int32Stamped = require('./Int32Stamped.js');
let SnapItRequest = require('./SnapItRequest.js');
let DepthErrorResult = require('./DepthErrorResult.js');
let LineArray = require('./LineArray.js');
let SimpleOccupancyGrid = require('./SimpleOccupancyGrid.js');
let SparseImage = require('./SparseImage.js');
let Object = require('./Object.js');
let RotatedRectStamped = require('./RotatedRectStamped.js');
let ContactSensorArray = require('./ContactSensorArray.js');
let ColorHistogram = require('./ColorHistogram.js');
let RotatedRect = require('./RotatedRect.js');
let ObjectArray = require('./ObjectArray.js');
let BoundingBox = require('./BoundingBox.js');
let ContactSensor = require('./ContactSensor.js');
let ClusterPointIndices = require('./ClusterPointIndices.js');
let TorusArray = require('./TorusArray.js');
let SparseOccupancyGridColumn = require('./SparseOccupancyGridColumn.js');
let HistogramWithRangeBin = require('./HistogramWithRangeBin.js');
let ColorHistogramArray = require('./ColorHistogramArray.js');
let HistogramWithRange = require('./HistogramWithRange.js');
let SlicedPointCloud = require('./SlicedPointCloud.js');
let Line = require('./Line.js');
let SimpleHandle = require('./SimpleHandle.js');
let ICPResult = require('./ICPResult.js');
let Segment = require('./Segment.js');
let PeoplePose = require('./PeoplePose.js');
let PlotData = require('./PlotData.js');
let PeoplePoseArray = require('./PeoplePoseArray.js');
let PosedCameraInfo = require('./PosedCameraInfo.js');
let Label = require('./Label.js');
let WeightedPoseArray = require('./WeightedPoseArray.js');
let TrackingStatus = require('./TrackingStatus.js');
let TrackerStatus = require('./TrackerStatus.js');
let SparseOccupancyGrid = require('./SparseOccupancyGrid.js');
let Accuracy = require('./Accuracy.js');
let RectArray = require('./RectArray.js');
let ParallelEdgeArray = require('./ParallelEdgeArray.js');
let PointsArray = require('./PointsArray.js');
let ClassificationResult = require('./ClassificationResult.js');
let ImageDifferenceValue = require('./ImageDifferenceValue.js');
let HeightmapConfig = require('./HeightmapConfig.js');

module.exports = {
  TimeRange: TimeRange,
  Histogram: Histogram,
  PlotDataArray: PlotDataArray,
  DepthCalibrationParameter: DepthCalibrationParameter,
  BoundingBoxArrayWithCameraInfo: BoundingBoxArrayWithCameraInfo,
  PolygonArray: PolygonArray,
  ParallelEdge: ParallelEdge,
  BoundingBoxMovement: BoundingBoxMovement,
  Spectrum: Spectrum,
  Circle2DArray: Circle2DArray,
  HumanSkeletonArray: HumanSkeletonArray,
  VectorArray: VectorArray,
  Rect: Rect,
  Torus: Torus,
  SparseOccupancyGridCell: SparseOccupancyGridCell,
  SparseOccupancyGridArray: SparseOccupancyGridArray,
  SegmentStamped: SegmentStamped,
  HistogramWithRangeArray: HistogramWithRangeArray,
  SegmentArray: SegmentArray,
  BoolStamped: BoolStamped,
  LabelArray: LabelArray,
  SimpleOccupancyGridArray: SimpleOccupancyGridArray,
  ModelCoefficientsArray: ModelCoefficientsArray,
  Circle2D: Circle2D,
  HumanSkeleton: HumanSkeleton,
  BoundingBoxArray: BoundingBoxArray,
  Int32Stamped: Int32Stamped,
  SnapItRequest: SnapItRequest,
  DepthErrorResult: DepthErrorResult,
  LineArray: LineArray,
  SimpleOccupancyGrid: SimpleOccupancyGrid,
  SparseImage: SparseImage,
  Object: Object,
  RotatedRectStamped: RotatedRectStamped,
  ContactSensorArray: ContactSensorArray,
  ColorHistogram: ColorHistogram,
  RotatedRect: RotatedRect,
  ObjectArray: ObjectArray,
  BoundingBox: BoundingBox,
  ContactSensor: ContactSensor,
  ClusterPointIndices: ClusterPointIndices,
  TorusArray: TorusArray,
  SparseOccupancyGridColumn: SparseOccupancyGridColumn,
  HistogramWithRangeBin: HistogramWithRangeBin,
  ColorHistogramArray: ColorHistogramArray,
  HistogramWithRange: HistogramWithRange,
  SlicedPointCloud: SlicedPointCloud,
  Line: Line,
  SimpleHandle: SimpleHandle,
  ICPResult: ICPResult,
  Segment: Segment,
  PeoplePose: PeoplePose,
  PlotData: PlotData,
  PeoplePoseArray: PeoplePoseArray,
  PosedCameraInfo: PosedCameraInfo,
  Label: Label,
  WeightedPoseArray: WeightedPoseArray,
  TrackingStatus: TrackingStatus,
  TrackerStatus: TrackerStatus,
  SparseOccupancyGrid: SparseOccupancyGrid,
  Accuracy: Accuracy,
  RectArray: RectArray,
  ParallelEdgeArray: ParallelEdgeArray,
  PointsArray: PointsArray,
  ClassificationResult: ClassificationResult,
  ImageDifferenceValue: ImageDifferenceValue,
  HeightmapConfig: HeightmapConfig,
};
