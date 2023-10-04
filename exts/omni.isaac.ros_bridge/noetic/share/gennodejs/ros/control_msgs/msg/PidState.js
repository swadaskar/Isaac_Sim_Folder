// Auto-generated. Do not edit!

// (in-package control_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PidState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.timestep = null;
      this.error = null;
      this.error_dot = null;
      this.p_error = null;
      this.i_error = null;
      this.d_error = null;
      this.p_term = null;
      this.i_term = null;
      this.d_term = null;
      this.i_max = null;
      this.i_min = null;
      this.output = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('timestep')) {
        this.timestep = initObj.timestep
      }
      else {
        this.timestep = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = 0.0;
      }
      if (initObj.hasOwnProperty('error_dot')) {
        this.error_dot = initObj.error_dot
      }
      else {
        this.error_dot = 0.0;
      }
      if (initObj.hasOwnProperty('p_error')) {
        this.p_error = initObj.p_error
      }
      else {
        this.p_error = 0.0;
      }
      if (initObj.hasOwnProperty('i_error')) {
        this.i_error = initObj.i_error
      }
      else {
        this.i_error = 0.0;
      }
      if (initObj.hasOwnProperty('d_error')) {
        this.d_error = initObj.d_error
      }
      else {
        this.d_error = 0.0;
      }
      if (initObj.hasOwnProperty('p_term')) {
        this.p_term = initObj.p_term
      }
      else {
        this.p_term = 0.0;
      }
      if (initObj.hasOwnProperty('i_term')) {
        this.i_term = initObj.i_term
      }
      else {
        this.i_term = 0.0;
      }
      if (initObj.hasOwnProperty('d_term')) {
        this.d_term = initObj.d_term
      }
      else {
        this.d_term = 0.0;
      }
      if (initObj.hasOwnProperty('i_max')) {
        this.i_max = initObj.i_max
      }
      else {
        this.i_max = 0.0;
      }
      if (initObj.hasOwnProperty('i_min')) {
        this.i_min = initObj.i_min
      }
      else {
        this.i_min = 0.0;
      }
      if (initObj.hasOwnProperty('output')) {
        this.output = initObj.output
      }
      else {
        this.output = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PidState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [timestep]
    bufferOffset = _serializer.duration(obj.timestep, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.float64(obj.error, buffer, bufferOffset);
    // Serialize message field [error_dot]
    bufferOffset = _serializer.float64(obj.error_dot, buffer, bufferOffset);
    // Serialize message field [p_error]
    bufferOffset = _serializer.float64(obj.p_error, buffer, bufferOffset);
    // Serialize message field [i_error]
    bufferOffset = _serializer.float64(obj.i_error, buffer, bufferOffset);
    // Serialize message field [d_error]
    bufferOffset = _serializer.float64(obj.d_error, buffer, bufferOffset);
    // Serialize message field [p_term]
    bufferOffset = _serializer.float64(obj.p_term, buffer, bufferOffset);
    // Serialize message field [i_term]
    bufferOffset = _serializer.float64(obj.i_term, buffer, bufferOffset);
    // Serialize message field [d_term]
    bufferOffset = _serializer.float64(obj.d_term, buffer, bufferOffset);
    // Serialize message field [i_max]
    bufferOffset = _serializer.float64(obj.i_max, buffer, bufferOffset);
    // Serialize message field [i_min]
    bufferOffset = _serializer.float64(obj.i_min, buffer, bufferOffset);
    // Serialize message field [output]
    bufferOffset = _serializer.float64(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PidState
    let len;
    let data = new PidState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [timestep]
    data.timestep = _deserializer.duration(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [error_dot]
    data.error_dot = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [p_error]
    data.p_error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [i_error]
    data.i_error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [d_error]
    data.d_error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [p_term]
    data.p_term = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [i_term]
    data.i_term = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [d_term]
    data.d_term = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [i_max]
    data.i_max = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [i_min]
    data.i_min = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [output]
    data.output = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 96;
  }

  static datatype() {
    // Returns string type for a message object
    return 'control_msgs/PidState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b138ec00e886c10e73f27e8712252ea6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    duration timestep
    float64 error
    float64 error_dot
    float64 p_error
    float64 i_error
    float64 d_error
    float64 p_term
    float64 i_term
    float64 d_term
    float64 i_max
    float64 i_min
    float64 output
    
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
    const resolved = new PidState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.timestep !== undefined) {
      resolved.timestep = msg.timestep;
    }
    else {
      resolved.timestep = {secs: 0, nsecs: 0}
    }

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = 0.0
    }

    if (msg.error_dot !== undefined) {
      resolved.error_dot = msg.error_dot;
    }
    else {
      resolved.error_dot = 0.0
    }

    if (msg.p_error !== undefined) {
      resolved.p_error = msg.p_error;
    }
    else {
      resolved.p_error = 0.0
    }

    if (msg.i_error !== undefined) {
      resolved.i_error = msg.i_error;
    }
    else {
      resolved.i_error = 0.0
    }

    if (msg.d_error !== undefined) {
      resolved.d_error = msg.d_error;
    }
    else {
      resolved.d_error = 0.0
    }

    if (msg.p_term !== undefined) {
      resolved.p_term = msg.p_term;
    }
    else {
      resolved.p_term = 0.0
    }

    if (msg.i_term !== undefined) {
      resolved.i_term = msg.i_term;
    }
    else {
      resolved.i_term = 0.0
    }

    if (msg.d_term !== undefined) {
      resolved.d_term = msg.d_term;
    }
    else {
      resolved.d_term = 0.0
    }

    if (msg.i_max !== undefined) {
      resolved.i_max = msg.i_max;
    }
    else {
      resolved.i_max = 0.0
    }

    if (msg.i_min !== undefined) {
      resolved.i_min = msg.i_min;
    }
    else {
      resolved.i_min = 0.0
    }

    if (msg.output !== undefined) {
      resolved.output = msg.output;
    }
    else {
      resolved.output = 0.0
    }

    return resolved;
    }
};

module.exports = PidState;
