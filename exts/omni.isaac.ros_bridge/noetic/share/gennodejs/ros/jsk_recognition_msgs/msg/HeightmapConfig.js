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

class HeightmapConfig {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.min_x = null;
      this.max_x = null;
      this.min_y = null;
      this.max_y = null;
    }
    else {
      if (initObj.hasOwnProperty('min_x')) {
        this.min_x = initObj.min_x
      }
      else {
        this.min_x = 0.0;
      }
      if (initObj.hasOwnProperty('max_x')) {
        this.max_x = initObj.max_x
      }
      else {
        this.max_x = 0.0;
      }
      if (initObj.hasOwnProperty('min_y')) {
        this.min_y = initObj.min_y
      }
      else {
        this.min_y = 0.0;
      }
      if (initObj.hasOwnProperty('max_y')) {
        this.max_y = initObj.max_y
      }
      else {
        this.max_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HeightmapConfig
    // Serialize message field [min_x]
    bufferOffset = _serializer.float32(obj.min_x, buffer, bufferOffset);
    // Serialize message field [max_x]
    bufferOffset = _serializer.float32(obj.max_x, buffer, bufferOffset);
    // Serialize message field [min_y]
    bufferOffset = _serializer.float32(obj.min_y, buffer, bufferOffset);
    // Serialize message field [max_y]
    bufferOffset = _serializer.float32(obj.max_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HeightmapConfig
    let len;
    let data = new HeightmapConfig(null);
    // Deserialize message field [min_x]
    data.min_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_x]
    data.max_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [min_y]
    data.min_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_y]
    data.max_y = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/HeightmapConfig';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f2f90f6dd5aeedef48b062e1a4aabb89';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 min_x
    float32 max_x
    float32 min_y
    float32 max_y
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HeightmapConfig(null);
    if (msg.min_x !== undefined) {
      resolved.min_x = msg.min_x;
    }
    else {
      resolved.min_x = 0.0
    }

    if (msg.max_x !== undefined) {
      resolved.max_x = msg.max_x;
    }
    else {
      resolved.max_x = 0.0
    }

    if (msg.min_y !== undefined) {
      resolved.min_y = msg.min_y;
    }
    else {
      resolved.min_y = 0.0
    }

    if (msg.max_y !== undefined) {
      resolved.max_y = msg.max_y;
    }
    else {
      resolved.max_y = 0.0
    }

    return resolved;
    }
};

module.exports = HeightmapConfig;
