// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Segment = require('./Segment.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class HumanSkeleton {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.bone_names = null;
      this.bones = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('bone_names')) {
        this.bone_names = initObj.bone_names
      }
      else {
        this.bone_names = [];
      }
      if (initObj.hasOwnProperty('bones')) {
        this.bones = initObj.bones
      }
      else {
        this.bones = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HumanSkeleton
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [bone_names]
    bufferOffset = _arraySerializer.string(obj.bone_names, buffer, bufferOffset, null);
    // Serialize message field [bones]
    // Serialize the length for message field [bones]
    bufferOffset = _serializer.uint32(obj.bones.length, buffer, bufferOffset);
    obj.bones.forEach((val) => {
      bufferOffset = Segment.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HumanSkeleton
    let len;
    let data = new HumanSkeleton(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [bone_names]
    data.bone_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [bones]
    // Deserialize array length for message field [bones]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.bones = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.bones[i] = Segment.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.bone_names.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += 48 * object.bones.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/HumanSkeleton';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b8098e195ee1734e91bc2e38c969a23f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    string[] bone_names
    jsk_recognition_msgs/Segment[] bones
    
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
    MSG: jsk_recognition_msgs/Segment
    geometry_msgs/Point start_point
    geometry_msgs/Point end_point
    
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
    const resolved = new HumanSkeleton(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.bone_names !== undefined) {
      resolved.bone_names = msg.bone_names;
    }
    else {
      resolved.bone_names = []
    }

    if (msg.bones !== undefined) {
      resolved.bones = new Array(msg.bones.length);
      for (let i = 0; i < resolved.bones.length; ++i) {
        resolved.bones[i] = Segment.Resolve(msg.bones[i]);
      }
    }
    else {
      resolved.bones = []
    }

    return resolved;
    }
};

module.exports = HumanSkeleton;
