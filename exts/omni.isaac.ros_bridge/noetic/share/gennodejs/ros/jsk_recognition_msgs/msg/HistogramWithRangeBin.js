// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class HistogramWithRangeBin {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.min_value = null;
      this.max_value = null;
      this.count = null;
    }
    else {
      if (initObj.hasOwnProperty('min_value')) {
        this.min_value = initObj.min_value
      }
      else {
        this.min_value = 0.0;
      }
      if (initObj.hasOwnProperty('max_value')) {
        this.max_value = initObj.max_value
      }
      else {
        this.max_value = 0.0;
      }
      if (initObj.hasOwnProperty('count')) {
        this.count = initObj.count
      }
      else {
        this.count = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HistogramWithRangeBin
    // Serialize message field [min_value]
    bufferOffset = _serializer.float64(obj.min_value, buffer, bufferOffset);
    // Serialize message field [max_value]
    bufferOffset = _serializer.float64(obj.max_value, buffer, bufferOffset);
    // Serialize message field [count]
    bufferOffset = _serializer.uint32(obj.count, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HistogramWithRangeBin
    let len;
    let data = new HistogramWithRangeBin(null);
    // Deserialize message field [min_value]
    data.min_value = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max_value]
    data.max_value = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [count]
    data.count = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/HistogramWithRangeBin';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a7fe6c3021fcba2c6357f3db21601551';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 min_value
    float64 max_value
    uint32 count
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HistogramWithRangeBin(null);
    if (msg.min_value !== undefined) {
      resolved.min_value = msg.min_value;
    }
    else {
      resolved.min_value = 0.0
    }

    if (msg.max_value !== undefined) {
      resolved.max_value = msg.max_value;
    }
    else {
      resolved.max_value = 0.0
    }

    if (msg.count !== undefined) {
      resolved.count = msg.count;
    }
    else {
      resolved.count = 0
    }

    return resolved;
    }
};

module.exports = HistogramWithRangeBin;
