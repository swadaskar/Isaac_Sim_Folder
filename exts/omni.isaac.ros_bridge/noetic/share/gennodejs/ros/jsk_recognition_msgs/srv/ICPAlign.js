// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

let ICPResult = require('../msg/ICPResult.js');

//-----------------------------------------------------------

class ICPAlignRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.reference_cloud = null;
      this.target_cloud = null;
    }
    else {
      if (initObj.hasOwnProperty('reference_cloud')) {
        this.reference_cloud = initObj.reference_cloud
      }
      else {
        this.reference_cloud = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('target_cloud')) {
        this.target_cloud = initObj.target_cloud
      }
      else {
        this.target_cloud = new sensor_msgs.msg.PointCloud2();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ICPAlignRequest
    // Serialize message field [reference_cloud]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.reference_cloud, buffer, bufferOffset);
    // Serialize message field [target_cloud]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.target_cloud, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ICPAlignRequest
    let len;
    let data = new ICPAlignRequest(null);
    // Deserialize message field [reference_cloud]
    data.reference_cloud = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [target_cloud]
    data.target_cloud = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.reference_cloud);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.target_cloud);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/ICPAlignRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3e0df534693afbbecb9cc87944720695';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/PointCloud2 reference_cloud
    sensor_msgs/PointCloud2 target_cloud
    
    ================================================================================
    MSG: sensor_msgs/PointCloud2
    # This message holds a collection of N-dimensional points, which may
    # contain additional information such as normals, intensity, etc. The
    # point data is stored as a binary blob, its layout described by the
    # contents of the "fields" array.
    
    # The point cloud data may be organized 2d (image-like) or 1d
    # (unordered). Point clouds organized as 2d images may be produced by
    # camera depth sensors such as stereo or time-of-flight.
    
    # Time of sensor data acquisition, and the coordinate frame ID (for 3d
    # points).
    Header header
    
    # 2D structure of the point cloud. If the cloud is unordered, height is
    # 1 and width is the length of the point cloud.
    uint32 height
    uint32 width
    
    # Describes the channels and their layout in the binary data blob.
    PointField[] fields
    
    bool    is_bigendian # Is this data bigendian?
    uint32  point_step   # Length of a point in bytes
    uint32  row_step     # Length of a row in bytes
    uint8[] data         # Actual point data, size is (row_step*height)
    
    bool is_dense        # True if there are no invalid points
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: sensor_msgs/PointField
    # This message holds the description of one point entry in the
    # PointCloud2 message format.
    uint8 INT8    = 1
    uint8 UINT8   = 2
    uint8 INT16   = 3
    uint8 UINT16  = 4
    uint8 INT32   = 5
    uint8 UINT32  = 6
    uint8 FLOAT32 = 7
    uint8 FLOAT64 = 8
    
    string name      # Name of field
    uint32 offset    # Offset from start of point struct
    uint8  datatype  # Datatype enumeration, see above
    uint32 count     # How many elements in the field
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ICPAlignRequest(null);
    if (msg.reference_cloud !== undefined) {
      resolved.reference_cloud = sensor_msgs.msg.PointCloud2.Resolve(msg.reference_cloud)
    }
    else {
      resolved.reference_cloud = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.target_cloud !== undefined) {
      resolved.target_cloud = sensor_msgs.msg.PointCloud2.Resolve(msg.target_cloud)
    }
    else {
      resolved.target_cloud = new sensor_msgs.msg.PointCloud2()
    }

    return resolved;
    }
};

class ICPAlignResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = new ICPResult();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ICPAlignResponse
    // Serialize message field [result]
    bufferOffset = ICPResult.serialize(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ICPAlignResponse
    let len;
    let data = new ICPAlignResponse(null);
    // Deserialize message field [result]
    data.result = ICPResult.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ICPResult.getMessageSize(object.result);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/ICPAlignResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a511a876c2be142caffd78741c68e4cf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    jsk_recognition_msgs/ICPResult result
    
    
    ================================================================================
    MSG: jsk_recognition_msgs/ICPResult
    Header header
    geometry_msgs/Pose pose
    string name
    float64 score
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ICPAlignResponse(null);
    if (msg.result !== undefined) {
      resolved.result = ICPResult.Resolve(msg.result)
    }
    else {
      resolved.result = new ICPResult()
    }

    return resolved;
    }
};

module.exports = {
  Request: ICPAlignRequest,
  Response: ICPAlignResponse,
  md5sum() { return '356a3a6001b7560173247ee06893ddba'; },
  datatype() { return 'jsk_recognition_msgs/ICPAlign'; }
};
