
"use strict";

let LaserScan = require('./LaserScan.js');
let JoyFeedback = require('./JoyFeedback.js');
let Temperature = require('./Temperature.js');
let JointState = require('./JointState.js');
let FluidPressure = require('./FluidPressure.js');
let Image = require('./Image.js');
let RelativeHumidity = require('./RelativeHumidity.js');
let MagneticField = require('./MagneticField.js');
let PointCloud2 = require('./PointCloud2.js');
let CompressedImage = require('./CompressedImage.js');
let PointField = require('./PointField.js');
let Imu = require('./Imu.js');
let JoyFeedbackArray = require('./JoyFeedbackArray.js');
let Illuminance = require('./Illuminance.js');
let NavSatStatus = require('./NavSatStatus.js');
let TimeReference = require('./TimeReference.js');
let Joy = require('./Joy.js');
let MultiDOFJointState = require('./MultiDOFJointState.js');
let CameraInfo = require('./CameraInfo.js');
let BatteryState = require('./BatteryState.js');
let PointCloud = require('./PointCloud.js');
let NavSatFix = require('./NavSatFix.js');
let MultiEchoLaserScan = require('./MultiEchoLaserScan.js');
let ChannelFloat32 = require('./ChannelFloat32.js');
let RegionOfInterest = require('./RegionOfInterest.js');
let LaserEcho = require('./LaserEcho.js');
let Range = require('./Range.js');

module.exports = {
  LaserScan: LaserScan,
  JoyFeedback: JoyFeedback,
  Temperature: Temperature,
  JointState: JointState,
  FluidPressure: FluidPressure,
  Image: Image,
  RelativeHumidity: RelativeHumidity,
  MagneticField: MagneticField,
  PointCloud2: PointCloud2,
  CompressedImage: CompressedImage,
  PointField: PointField,
  Imu: Imu,
  JoyFeedbackArray: JoyFeedbackArray,
  Illuminance: Illuminance,
  NavSatStatus: NavSatStatus,
  TimeReference: TimeReference,
  Joy: Joy,
  MultiDOFJointState: MultiDOFJointState,
  CameraInfo: CameraInfo,
  BatteryState: BatteryState,
  PointCloud: PointCloud,
  NavSatFix: NavSatFix,
  MultiEchoLaserScan: MultiEchoLaserScan,
  ChannelFloat32: ChannelFloat32,
  RegionOfInterest: RegionOfInterest,
  LaserEcho: LaserEcho,
  Range: Range,
};
