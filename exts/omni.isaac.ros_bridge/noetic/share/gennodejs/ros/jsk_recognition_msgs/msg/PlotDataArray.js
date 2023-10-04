// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PlotData = require('./PlotData.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PlotDataArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.data = null;
      this.no_legend = null;
      this.legend_font_size = null;
      this.max_x = null;
      this.min_x = null;
      this.min_y = null;
      this.max_y = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = [];
      }
      if (initObj.hasOwnProperty('no_legend')) {
        this.no_legend = initObj.no_legend
      }
      else {
        this.no_legend = false;
      }
      if (initObj.hasOwnProperty('legend_font_size')) {
        this.legend_font_size = initObj.legend_font_size
      }
      else {
        this.legend_font_size = 0.0;
      }
      if (initObj.hasOwnProperty('max_x')) {
        this.max_x = initObj.max_x
      }
      else {
        this.max_x = 0.0;
      }
      if (initObj.hasOwnProperty('min_x')) {
        this.min_x = initObj.min_x
      }
      else {
        this.min_x = 0.0;
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
    // Serializes a message object of type PlotDataArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [data]
    // Serialize the length for message field [data]
    bufferOffset = _serializer.uint32(obj.data.length, buffer, bufferOffset);
    obj.data.forEach((val) => {
      bufferOffset = PlotData.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [no_legend]
    bufferOffset = _serializer.bool(obj.no_legend, buffer, bufferOffset);
    // Serialize message field [legend_font_size]
    bufferOffset = _serializer.float32(obj.legend_font_size, buffer, bufferOffset);
    // Serialize message field [max_x]
    bufferOffset = _serializer.float32(obj.max_x, buffer, bufferOffset);
    // Serialize message field [min_x]
    bufferOffset = _serializer.float32(obj.min_x, buffer, bufferOffset);
    // Serialize message field [min_y]
    bufferOffset = _serializer.float32(obj.min_y, buffer, bufferOffset);
    // Serialize message field [max_y]
    bufferOffset = _serializer.float32(obj.max_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlotDataArray
    let len;
    let data = new PlotDataArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [data]
    // Deserialize array length for message field [data]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.data = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.data[i] = PlotData.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [no_legend]
    data.no_legend = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [legend_font_size]
    data.legend_font_size = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_x]
    data.max_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [min_x]
    data.min_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [min_y]
    data.min_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_y]
    data.max_y = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.data.forEach((val) => {
      length += PlotData.getMessageSize(val);
    });
    return length + 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/PlotDataArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e83e9378b374b8436f955a6cd212770a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    jsk_recognition_msgs/PlotData[] data
    bool no_legend
    float32 legend_font_size
    float32 max_x
    float32 min_x
    float32 min_y
    float32 max_y
    
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
    MSG: jsk_recognition_msgs/PlotData
    uint32 SCATTER=1
    uint32 LINE=2
    
    Header header
    float32[] xs
    float32[] ys
    uint32 type                     #SCATTER or LINE
    string label
    bool fit_line
    bool fit_line_ransac
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlotDataArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.data !== undefined) {
      resolved.data = new Array(msg.data.length);
      for (let i = 0; i < resolved.data.length; ++i) {
        resolved.data[i] = PlotData.Resolve(msg.data[i]);
      }
    }
    else {
      resolved.data = []
    }

    if (msg.no_legend !== undefined) {
      resolved.no_legend = msg.no_legend;
    }
    else {
      resolved.no_legend = false
    }

    if (msg.legend_font_size !== undefined) {
      resolved.legend_font_size = msg.legend_font_size;
    }
    else {
      resolved.legend_font_size = 0.0
    }

    if (msg.max_x !== undefined) {
      resolved.max_x = msg.max_x;
    }
    else {
      resolved.max_x = 0.0
    }

    if (msg.min_x !== undefined) {
      resolved.min_x = msg.min_x;
    }
    else {
      resolved.min_x = 0.0
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

module.exports = PlotDataArray;
