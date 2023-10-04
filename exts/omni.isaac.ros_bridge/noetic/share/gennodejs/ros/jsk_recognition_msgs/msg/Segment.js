// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Segment {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start_point = null;
      this.end_point = null;
    }
    else {
      if (initObj.hasOwnProperty('start_point')) {
        this.start_point = initObj.start_point
      }
      else {
        this.start_point = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('end_point')) {
        this.end_point = initObj.end_point
      }
      else {
        this.end_point = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Segment
    // Serialize message field [start_point]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.start_point, buffer, bufferOffset);
    // Serialize message field [end_point]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.end_point, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Segment
    let len;
    let data = new Segment(null);
    // Deserialize message field [start_point]
    data.start_point = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [end_point]
    data.end_point = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/Segment';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0125c553546d7123dccaeab992a9e29e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Segment(null);
    if (msg.start_point !== undefined) {
      resolved.start_point = geometry_msgs.msg.Point.Resolve(msg.start_point)
    }
    else {
      resolved.start_point = new geometry_msgs.msg.Point()
    }

    if (msg.end_point !== undefined) {
      resolved.end_point = geometry_msgs.msg.Point.Resolve(msg.end_point)
    }
    else {
      resolved.end_point = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = Segment;
