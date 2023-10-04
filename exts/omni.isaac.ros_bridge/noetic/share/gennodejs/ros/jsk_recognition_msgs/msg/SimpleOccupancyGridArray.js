// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SimpleOccupancyGrid = require('./SimpleOccupancyGrid.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SimpleOccupancyGridArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.grids = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('grids')) {
        this.grids = initObj.grids
      }
      else {
        this.grids = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SimpleOccupancyGridArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [grids]
    // Serialize the length for message field [grids]
    bufferOffset = _serializer.uint32(obj.grids.length, buffer, bufferOffset);
    obj.grids.forEach((val) => {
      bufferOffset = SimpleOccupancyGrid.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SimpleOccupancyGridArray
    let len;
    let data = new SimpleOccupancyGridArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [grids]
    // Deserialize array length for message field [grids]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.grids = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.grids[i] = SimpleOccupancyGrid.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.grids.forEach((val) => {
      length += SimpleOccupancyGrid.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/SimpleOccupancyGridArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e448b458270a6ec465d66169c4180932';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    SimpleOccupancyGrid[] grids
    
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
    MSG: jsk_recognition_msgs/SimpleOccupancyGrid
    Header header
    # plane coefficients
    float32[4] coefficients
    # cells
    float32 resolution
    geometry_msgs/Point[] cells
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
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
    const resolved = new SimpleOccupancyGridArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.grids !== undefined) {
      resolved.grids = new Array(msg.grids.length);
      for (let i = 0; i < resolved.grids.length; ++i) {
        resolved.grids[i] = SimpleOccupancyGrid.Resolve(msg.grids[i]);
      }
    }
    else {
      resolved.grids = []
    }

    return resolved;
    }
};

module.exports = SimpleOccupancyGridArray;
