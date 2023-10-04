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
let pcl_msgs = _finder('pcl_msgs');

//-----------------------------------------------------------

class ModelCoefficientsArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.coefficients = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('coefficients')) {
        this.coefficients = initObj.coefficients
      }
      else {
        this.coefficients = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ModelCoefficientsArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [coefficients]
    // Serialize the length for message field [coefficients]
    bufferOffset = _serializer.uint32(obj.coefficients.length, buffer, bufferOffset);
    obj.coefficients.forEach((val) => {
      bufferOffset = pcl_msgs.msg.ModelCoefficients.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ModelCoefficientsArray
    let len;
    let data = new ModelCoefficientsArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [coefficients]
    // Deserialize array length for message field [coefficients]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.coefficients = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.coefficients[i] = pcl_msgs.msg.ModelCoefficients.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.coefficients.forEach((val) => {
      length += pcl_msgs.msg.ModelCoefficients.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/ModelCoefficientsArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '059efee897c3f4ae027a493e30c4c26b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ModelCoefficientsArray is used to represent coefficients of model
    # for each segmented clusters.
    # Simply put, ModelCoefficientsArray is a list of ModelCoefficients.
    Header header
    pcl_msgs/ModelCoefficients[] coefficients
    
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
    MSG: pcl_msgs/ModelCoefficients
    Header header
    float32[] values
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ModelCoefficientsArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.coefficients !== undefined) {
      resolved.coefficients = new Array(msg.coefficients.length);
      for (let i = 0; i < resolved.coefficients.length; ++i) {
        resolved.coefficients[i] = pcl_msgs.msg.ModelCoefficients.Resolve(msg.coefficients[i]);
      }
    }
    else {
      resolved.coefficients = []
    }

    return resolved;
    }
};

module.exports = ModelCoefficientsArray;
