// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BoundingBox = require('../msg/BoundingBox.js');
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

let ICPResult = require('../msg/ICPResult.js');

//-----------------------------------------------------------

class ICPAlignWithBoxRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_cloud = null;
      this.target_box = null;
    }
    else {
      if (initObj.hasOwnProperty('target_cloud')) {
        this.target_cloud = initObj.target_cloud
      }
      else {
        this.target_cloud = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('target_box')) {
        this.target_box = initObj.target_box
      }
      else {
        this.target_box = new BoundingBox();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ICPAlignWithBoxRequest
    // Serialize message field [target_cloud]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.target_cloud, buffer, bufferOffset);
    // Serialize message field [target_box]
    bufferOffset = BoundingBox.serialize(obj.target_box, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ICPAlignWithBoxRequest
    let len;
    let data = new ICPAlignWithBoxRequest(null);
    // Deserialize message field [target_cloud]
    data.target_cloud = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [target_box]
    data.target_box = BoundingBox.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.target_cloud);
    length += BoundingBox.getMessageSize(object.target_box);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/ICPAlignWithBoxRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9431a0df63f5a2625b973e92c6229cf3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/PointCloud2 target_cloud
    jsk_recognition_msgs/BoundingBox target_box
    
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
    
    ================================================================================
    MSG: jsk_recognition_msgs/BoundingBox
    # BoundingBox represents a oriented bounding box.
    Header header
    geometry_msgs/Pose pose
    geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)
    # You can use this field to hold value such as likelihood
    float32 value
    uint32 label
    
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
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ICPAlignWithBoxRequest(null);
    if (msg.target_cloud !== undefined) {
      resolved.target_cloud = sensor_msgs.msg.PointCloud2.Resolve(msg.target_cloud)
    }
    else {
      resolved.target_cloud = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.target_box !== undefined) {
      resolved.target_box = BoundingBox.Resolve(msg.target_box)
    }
    else {
      resolved.target_box = new BoundingBox()
    }

    return resolved;
    }
};

class ICPAlignWithBoxResponse {
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
    // Serializes a message object of type ICPAlignWithBoxResponse
    // Serialize message field [result]
    bufferOffset = ICPResult.serialize(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ICPAlignWithBoxResponse
    let len;
    let data = new ICPAlignWithBoxResponse(null);
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
    return 'jsk_recognition_msgs/ICPAlignWithBoxResponse';
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
    const resolved = new ICPAlignWithBoxResponse(null);
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
  Request: ICPAlignWithBoxRequest,
  Response: ICPAlignWithBoxResponse,
  md5sum() { return 'd23c790c7c4a5f0e959eba05efc9d29a'; },
  datatype() { return 'jsk_recognition_msgs/ICPAlignWithBox'; }
};
