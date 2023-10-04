// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class DepthErrorResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.u = null;
      this.v = null;
      this.center_u = null;
      this.center_v = null;
      this.true_depth = null;
      this.observed_depth = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('u')) {
        this.u = initObj.u
      }
      else {
        this.u = 0;
      }
      if (initObj.hasOwnProperty('v')) {
        this.v = initObj.v
      }
      else {
        this.v = 0;
      }
      if (initObj.hasOwnProperty('center_u')) {
        this.center_u = initObj.center_u
      }
      else {
        this.center_u = 0.0;
      }
      if (initObj.hasOwnProperty('center_v')) {
        this.center_v = initObj.center_v
      }
      else {
        this.center_v = 0.0;
      }
      if (initObj.hasOwnProperty('true_depth')) {
        this.true_depth = initObj.true_depth
      }
      else {
        this.true_depth = 0.0;
      }
      if (initObj.hasOwnProperty('observed_depth')) {
        this.observed_depth = initObj.observed_depth
      }
      else {
        this.observed_depth = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DepthErrorResult
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [u]
    bufferOffset = _serializer.uint32(obj.u, buffer, bufferOffset);
    // Serialize message field [v]
    bufferOffset = _serializer.uint32(obj.v, buffer, bufferOffset);
    // Serialize message field [center_u]
    bufferOffset = _serializer.float32(obj.center_u, buffer, bufferOffset);
    // Serialize message field [center_v]
    bufferOffset = _serializer.float32(obj.center_v, buffer, bufferOffset);
    // Serialize message field [true_depth]
    bufferOffset = _serializer.float32(obj.true_depth, buffer, bufferOffset);
    // Serialize message field [observed_depth]
    bufferOffset = _serializer.float32(obj.observed_depth, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DepthErrorResult
    let len;
    let data = new DepthErrorResult(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [u]
    data.u = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [v]
    data.v = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [center_u]
    data.center_u = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [center_v]
    data.center_v = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [true_depth]
    data.true_depth = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [observed_depth]
    data.observed_depth = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/DepthErrorResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cebbb2d000457fe5d971a29ee02ffb16';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    uint32 u
    uint32 v
    float32 center_u
    float32 center_v
    float32 true_depth
    float32 observed_depth
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DepthErrorResult(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.u !== undefined) {
      resolved.u = msg.u;
    }
    else {
      resolved.u = 0
    }

    if (msg.v !== undefined) {
      resolved.v = msg.v;
    }
    else {
      resolved.v = 0
    }

    if (msg.center_u !== undefined) {
      resolved.center_u = msg.center_u;
    }
    else {
      resolved.center_u = 0.0
    }

    if (msg.center_v !== undefined) {
      resolved.center_v = msg.center_v;
    }
    else {
      resolved.center_v = 0.0
    }

    if (msg.true_depth !== undefined) {
      resolved.true_depth = msg.true_depth;
    }
    else {
      resolved.true_depth = 0.0
    }

    if (msg.observed_depth !== undefined) {
      resolved.observed_depth = msg.observed_depth;
    }
    else {
      resolved.observed_depth = 0.0
    }

    return resolved;
    }
};

module.exports = DepthErrorResult;
