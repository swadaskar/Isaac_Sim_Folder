// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ColorHistogram = require('./ColorHistogram.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ColorHistogramArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.histograms = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('histograms')) {
        this.histograms = initObj.histograms
      }
      else {
        this.histograms = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ColorHistogramArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [histograms]
    // Serialize the length for message field [histograms]
    bufferOffset = _serializer.uint32(obj.histograms.length, buffer, bufferOffset);
    obj.histograms.forEach((val) => {
      bufferOffset = ColorHistogram.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ColorHistogramArray
    let len;
    let data = new ColorHistogramArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [histograms]
    // Deserialize array length for message field [histograms]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.histograms = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.histograms[i] = ColorHistogram.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.histograms.forEach((val) => {
      length += ColorHistogram.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/ColorHistogramArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3bcc7f05c5520f311047096d5530e715';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    ColorHistogram[] histograms
    
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
    MSG: jsk_recognition_msgs/ColorHistogram
    Header header
    float32[] histogram
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ColorHistogramArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.histograms !== undefined) {
      resolved.histograms = new Array(msg.histograms.length);
      for (let i = 0; i < resolved.histograms.length; ++i) {
        resolved.histograms[i] = ColorHistogram.Resolve(msg.histograms[i]);
      }
    }
    else {
      resolved.histograms = []
    }

    return resolved;
    }
};

module.exports = ColorHistogramArray;
