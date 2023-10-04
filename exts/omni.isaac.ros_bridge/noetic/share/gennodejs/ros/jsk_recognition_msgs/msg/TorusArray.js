// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Torus = require('./Torus.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TorusArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.toruses = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('toruses')) {
        this.toruses = initObj.toruses
      }
      else {
        this.toruses = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TorusArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [toruses]
    // Serialize the length for message field [toruses]
    bufferOffset = _serializer.uint32(obj.toruses.length, buffer, bufferOffset);
    obj.toruses.forEach((val) => {
      bufferOffset = Torus.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TorusArray
    let len;
    let data = new TorusArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [toruses]
    // Deserialize array length for message field [toruses]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.toruses = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.toruses[i] = Torus.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.toruses.forEach((val) => {
      length += Torus.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/TorusArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '81d0dbf46016b5714fa4ea9eca5485e0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    Torus[] toruses
    
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
    MSG: jsk_recognition_msgs/Torus
    Header header
    bool failure
    geometry_msgs/Pose pose
    float64 large_radius
    float64 small_radius
    
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
    const resolved = new TorusArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.toruses !== undefined) {
      resolved.toruses = new Array(msg.toruses.length);
      for (let i = 0; i < resolved.toruses.length; ++i) {
        resolved.toruses[i] = Torus.Resolve(msg.toruses[i]);
      }
    }
    else {
      resolved.toruses = []
    }

    return resolved;
    }
};

module.exports = TorusArray;
