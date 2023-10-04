// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let HumanSkeleton = require('./HumanSkeleton.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class HumanSkeletonArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.human_ids = null;
      this.skeletons = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('human_ids')) {
        this.human_ids = initObj.human_ids
      }
      else {
        this.human_ids = [];
      }
      if (initObj.hasOwnProperty('skeletons')) {
        this.skeletons = initObj.skeletons
      }
      else {
        this.skeletons = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HumanSkeletonArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [human_ids]
    // Serialize the length for message field [human_ids]
    bufferOffset = _serializer.uint32(obj.human_ids.length, buffer, bufferOffset);
    obj.human_ids.forEach((val) => {
      bufferOffset = std_msgs.msg.Int32.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [skeletons]
    // Serialize the length for message field [skeletons]
    bufferOffset = _serializer.uint32(obj.skeletons.length, buffer, bufferOffset);
    obj.skeletons.forEach((val) => {
      bufferOffset = HumanSkeleton.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HumanSkeletonArray
    let len;
    let data = new HumanSkeletonArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [human_ids]
    // Deserialize array length for message field [human_ids]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.human_ids = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.human_ids[i] = std_msgs.msg.Int32.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [skeletons]
    // Deserialize array length for message field [skeletons]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.skeletons = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.skeletons[i] = HumanSkeleton.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.human_ids.length;
    object.skeletons.forEach((val) => {
      length += HumanSkeleton.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/HumanSkeletonArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4013ce44abe9aafa9055e095797a9d50';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    std_msgs/Int32[] human_ids
    jsk_recognition_msgs/HumanSkeleton[] skeletons
    
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
    MSG: std_msgs/Int32
    int32 data
    ================================================================================
    MSG: jsk_recognition_msgs/HumanSkeleton
    std_msgs/Header header
    string[] bone_names
    jsk_recognition_msgs/Segment[] bones
    
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
    const resolved = new HumanSkeletonArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.human_ids !== undefined) {
      resolved.human_ids = new Array(msg.human_ids.length);
      for (let i = 0; i < resolved.human_ids.length; ++i) {
        resolved.human_ids[i] = std_msgs.msg.Int32.Resolve(msg.human_ids[i]);
      }
    }
    else {
      resolved.human_ids = []
    }

    if (msg.skeletons !== undefined) {
      resolved.skeletons = new Array(msg.skeletons.length);
      for (let i = 0; i < resolved.skeletons.length; ++i) {
        resolved.skeletons[i] = HumanSkeleton.Resolve(msg.skeletons[i]);
      }
    }
    else {
      resolved.skeletons = []
    }

    return resolved;
    }
};

module.exports = HumanSkeletonArray;
