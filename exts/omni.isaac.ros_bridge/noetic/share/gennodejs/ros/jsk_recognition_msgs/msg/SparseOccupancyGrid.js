// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SparseOccupancyGridColumn = require('./SparseOccupancyGridColumn.js');
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class SparseOccupancyGrid {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.origin_pose = null;
      this.resolution = null;
      this.columns = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('origin_pose')) {
        this.origin_pose = initObj.origin_pose
      }
      else {
        this.origin_pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('resolution')) {
        this.resolution = initObj.resolution
      }
      else {
        this.resolution = 0.0;
      }
      if (initObj.hasOwnProperty('columns')) {
        this.columns = initObj.columns
      }
      else {
        this.columns = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SparseOccupancyGrid
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [origin_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.origin_pose, buffer, bufferOffset);
    // Serialize message field [resolution]
    bufferOffset = _serializer.float32(obj.resolution, buffer, bufferOffset);
    // Serialize message field [columns]
    // Serialize the length for message field [columns]
    bufferOffset = _serializer.uint32(obj.columns.length, buffer, bufferOffset);
    obj.columns.forEach((val) => {
      bufferOffset = SparseOccupancyGridColumn.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SparseOccupancyGrid
    let len;
    let data = new SparseOccupancyGrid(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [origin_pose]
    data.origin_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [resolution]
    data.resolution = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [columns]
    // Deserialize array length for message field [columns]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.columns = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.columns[i] = SparseOccupancyGridColumn.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.columns.forEach((val) => {
      length += SparseOccupancyGridColumn.getMessageSize(val);
    });
    return length + 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/SparseOccupancyGrid';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '497269ddab6058d0d4860f25dc49448f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    geometry_msgs/Pose origin_pose
    float32 resolution
    
    SparseOccupancyGridColumn[] columns
    
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
    
    ================================================================================
    MSG: jsk_recognition_msgs/SparseOccupancyGridColumn
    int32 column_index
    SparseOccupancyGridCell[] cells
    
    ================================================================================
    MSG: jsk_recognition_msgs/SparseOccupancyGridCell
    int32 row_index
    float32 value
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SparseOccupancyGrid(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.origin_pose !== undefined) {
      resolved.origin_pose = geometry_msgs.msg.Pose.Resolve(msg.origin_pose)
    }
    else {
      resolved.origin_pose = new geometry_msgs.msg.Pose()
    }

    if (msg.resolution !== undefined) {
      resolved.resolution = msg.resolution;
    }
    else {
      resolved.resolution = 0.0
    }

    if (msg.columns !== undefined) {
      resolved.columns = new Array(msg.columns.length);
      for (let i = 0; i < resolved.columns.length; ++i) {
        resolved.columns[i] = SparseOccupancyGridColumn.Resolve(msg.columns[i]);
      }
    }
    else {
      resolved.columns = []
    }

    return resolved;
    }
};

module.exports = SparseOccupancyGrid;
