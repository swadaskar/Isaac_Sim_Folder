// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DepthCalibrationParameter = require('../msg/DepthCalibrationParameter.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetDepthCalibrationParameterRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.parameter = null;
    }
    else {
      if (initObj.hasOwnProperty('parameter')) {
        this.parameter = initObj.parameter
      }
      else {
        this.parameter = new DepthCalibrationParameter();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetDepthCalibrationParameterRequest
    // Serialize message field [parameter]
    bufferOffset = DepthCalibrationParameter.serialize(obj.parameter, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetDepthCalibrationParameterRequest
    let len;
    let data = new SetDepthCalibrationParameterRequest(null);
    // Deserialize message field [parameter]
    data.parameter = DepthCalibrationParameter.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += DepthCalibrationParameter.getMessageSize(object.parameter);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/SetDepthCalibrationParameterRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '46b1552cd9512adcf9bbec97800b0e0d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    jsk_recognition_msgs/DepthCalibrationParameter parameter
    
    ================================================================================
    MSG: jsk_recognition_msgs/DepthCalibrationParameter
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
    const resolved = new SetDepthCalibrationParameterRequest(null);
    if (msg.parameter !== undefined) {
      resolved.parameter = DepthCalibrationParameter.Resolve(msg.parameter)
    }
    else {
      resolved.parameter = new DepthCalibrationParameter()
    }

    return resolved;
    }
};

class SetDepthCalibrationParameterResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetDepthCalibrationParameterResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetDepthCalibrationParameterResponse
    let len;
    let data = new SetDepthCalibrationParameterResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/SetDepthCalibrationParameterResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetDepthCalibrationParameterResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetDepthCalibrationParameterRequest,
  Response: SetDepthCalibrationParameterResponse,
  md5sum() { return '46b1552cd9512adcf9bbec97800b0e0d'; },
  datatype() { return 'jsk_recognition_msgs/SetDepthCalibrationParameter'; }
};
