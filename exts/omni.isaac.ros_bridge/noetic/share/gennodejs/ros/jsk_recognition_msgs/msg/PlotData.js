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

//-----------------------------------------------------------

class PlotData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.xs = null;
      this.ys = null;
      this.type = null;
      this.label = null;
      this.fit_line = null;
      this.fit_line_ransac = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('xs')) {
        this.xs = initObj.xs
      }
      else {
        this.xs = [];
      }
      if (initObj.hasOwnProperty('ys')) {
        this.ys = initObj.ys
      }
      else {
        this.ys = [];
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('label')) {
        this.label = initObj.label
      }
      else {
        this.label = '';
      }
      if (initObj.hasOwnProperty('fit_line')) {
        this.fit_line = initObj.fit_line
      }
      else {
        this.fit_line = false;
      }
      if (initObj.hasOwnProperty('fit_line_ransac')) {
        this.fit_line_ransac = initObj.fit_line_ransac
      }
      else {
        this.fit_line_ransac = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlotData
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [xs]
    bufferOffset = _arraySerializer.float32(obj.xs, buffer, bufferOffset, null);
    // Serialize message field [ys]
    bufferOffset = _arraySerializer.float32(obj.ys, buffer, bufferOffset, null);
    // Serialize message field [type]
    bufferOffset = _serializer.uint32(obj.type, buffer, bufferOffset);
    // Serialize message field [label]
    bufferOffset = _serializer.string(obj.label, buffer, bufferOffset);
    // Serialize message field [fit_line]
    bufferOffset = _serializer.bool(obj.fit_line, buffer, bufferOffset);
    // Serialize message field [fit_line_ransac]
    bufferOffset = _serializer.bool(obj.fit_line_ransac, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlotData
    let len;
    let data = new PlotData(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [xs]
    data.xs = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [ys]
    data.ys = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [type]
    data.type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [label]
    data.label = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [fit_line]
    data.fit_line = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [fit_line_ransac]
    data.fit_line_ransac = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.xs.length;
    length += 4 * object.ys.length;
    length += _getByteLength(object.label);
    return length + 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/PlotData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'abc388ba2207b305f8695ad025452af4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 SCATTER=1
    uint32 LINE=2
    
    Header header
    float32[] xs
    float32[] ys
    uint32 type                     #SCATTER or LINE
    string label
    bool fit_line
    bool fit_line_ransac
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlotData(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.xs !== undefined) {
      resolved.xs = msg.xs;
    }
    else {
      resolved.xs = []
    }

    if (msg.ys !== undefined) {
      resolved.ys = msg.ys;
    }
    else {
      resolved.ys = []
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.label !== undefined) {
      resolved.label = msg.label;
    }
    else {
      resolved.label = ''
    }

    if (msg.fit_line !== undefined) {
      resolved.fit_line = msg.fit_line;
    }
    else {
      resolved.fit_line = false
    }

    if (msg.fit_line_ransac !== undefined) {
      resolved.fit_line_ransac = msg.fit_line_ransac;
    }
    else {
      resolved.fit_line_ransac = false
    }

    return resolved;
    }
};

// Constants for message
PlotData.Constants = {
  SCATTER: 1,
  LINE: 2,
}

module.exports = PlotData;
