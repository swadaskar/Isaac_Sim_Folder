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

class DepthCalibrationParameter {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.coefficients2 = null;
      this.coefficients1 = null;
      this.coefficients0 = null;
      this.use_abs = null;
    }
    else {
      if (initObj.hasOwnProperty('coefficients2')) {
        this.coefficients2 = initObj.coefficients2
      }
      else {
        this.coefficients2 = [];
      }
      if (initObj.hasOwnProperty('coefficients1')) {
        this.coefficients1 = initObj.coefficients1
      }
      else {
        this.coefficients1 = [];
      }
      if (initObj.hasOwnProperty('coefficients0')) {
        this.coefficients0 = initObj.coefficients0
      }
      else {
        this.coefficients0 = [];
      }
      if (initObj.hasOwnProperty('use_abs')) {
        this.use_abs = initObj.use_abs
      }
      else {
        this.use_abs = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DepthCalibrationParameter
    // Serialize message field [coefficients2]
    bufferOffset = _arraySerializer.float64(obj.coefficients2, buffer, bufferOffset, null);
    // Serialize message field [coefficients1]
    bufferOffset = _arraySerializer.float64(obj.coefficients1, buffer, bufferOffset, null);
    // Serialize message field [coefficients0]
    bufferOffset = _arraySerializer.float64(obj.coefficients0, buffer, bufferOffset, null);
    // Serialize message field [use_abs]
    bufferOffset = _serializer.bool(obj.use_abs, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DepthCalibrationParameter
    let len;
    let data = new DepthCalibrationParameter(null);
    // Deserialize message field [coefficients2]
    data.coefficients2 = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [coefficients1]
    data.coefficients1 = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [coefficients0]
    data.coefficients0 = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [use_abs]
    data.use_abs = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.coefficients2.length;
    length += 8 * object.coefficients1.length;
    length += 8 * object.coefficients0.length;
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/DepthCalibrationParameter';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd8318983ee0a76ad66ecf4b504350888';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # each vector stands for C(u, v)
    # C(u, v) = a_0 * u^2 + a_1 * u + a_2 * v^2 + a_3 * v + a_4
    float64[] coefficients2
    float64[] coefficients1
    float64[] coefficients0
    bool use_abs
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DepthCalibrationParameter(null);
    if (msg.coefficients2 !== undefined) {
      resolved.coefficients2 = msg.coefficients2;
    }
    else {
      resolved.coefficients2 = []
    }

    if (msg.coefficients1 !== undefined) {
      resolved.coefficients1 = msg.coefficients1;
    }
    else {
      resolved.coefficients1 = []
    }

    if (msg.coefficients0 !== undefined) {
      resolved.coefficients0 = msg.coefficients0;
    }
    else {
      resolved.coefficients0 = []
    }

    if (msg.use_abs !== undefined) {
      resolved.use_abs = msg.use_abs;
    }
    else {
      resolved.use_abs = false
    }

    return resolved;
    }
};

module.exports = DepthCalibrationParameter;
