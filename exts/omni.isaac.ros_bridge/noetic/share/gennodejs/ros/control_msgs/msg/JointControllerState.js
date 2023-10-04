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

class JointControllerState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.set_point = null;
      this.process_value = null;
      this.process_value_dot = null;
      this.error = null;
      this.time_step = null;
      this.command = null;
      this.p = null;
      this.i = null;
      this.d = null;
      this.i_clamp = null;
      this.antiwindup = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('set_point')) {
        this.set_point = initObj.set_point
      }
      else {
        this.set_point = 0.0;
      }
      if (initObj.hasOwnProperty('process_value')) {
        this.process_value = initObj.process_value
      }
      else {
        this.process_value = 0.0;
      }
      if (initObj.hasOwnProperty('process_value_dot')) {
        this.process_value_dot = initObj.process_value_dot
      }
      else {
        this.process_value_dot = 0.0;
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = 0.0;
      }
      if (initObj.hasOwnProperty('time_step')) {
        this.time_step = initObj.time_step
      }
      else {
        this.time_step = 0.0;
      }
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = 0.0;
      }
      if (initObj.hasOwnProperty('p')) {
        this.p = initObj.p
      }
      else {
        this.p = 0.0;
      }
      if (initObj.hasOwnProperty('i')) {
        this.i = initObj.i
      }
      else {
        this.i = 0.0;
      }
      if (initObj.hasOwnProperty('d')) {
        this.d = initObj.d
      }
      else {
        this.d = 0.0;
      }
      if (initObj.hasOwnProperty('i_clamp')) {
        this.i_clamp = initObj.i_clamp
      }
      else {
        this.i_clamp = 0.0;
      }
      if (initObj.hasOwnProperty('antiwindup')) {
        this.antiwindup = initObj.antiwindup
      }
      else {
        this.antiwindup = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointControllerState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [set_point]
    bufferOffset = _serializer.float64(obj.set_point, buffer, bufferOffset);
    // Serialize message field [process_value]
    bufferOffset = _serializer.float64(obj.process_value, buffer, bufferOffset);
    // Serialize message field [process_value_dot]
    bufferOffset = _serializer.float64(obj.process_value_dot, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.float64(obj.error, buffer, bufferOffset);
    // Serialize message field [time_step]
    bufferOffset = _serializer.float64(obj.time_step, buffer, bufferOffset);
    // Serialize message field [command]
    bufferOffset = _serializer.float64(obj.command, buffer, bufferOffset);
    // Serialize message field [p]
    bufferOffset = _serializer.float64(obj.p, buffer, bufferOffset);
    // Serialize message field [i]
    bufferOffset = _serializer.float64(obj.i, buffer, bufferOffset);
    // Serialize message field [d]
    bufferOffset = _serializer.float64(obj.d, buffer, bufferOffset);
    // Serialize message field [i_clamp]
    bufferOffset = _serializer.float64(obj.i_clamp, buffer, bufferOffset);
    // Serialize message field [antiwindup]
    bufferOffset = _serializer.bool(obj.antiwindup, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointControllerState
    let len;
    let data = new JointControllerState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [set_point]
    data.set_point = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [process_value]
    data.process_value = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [process_value_dot]
    data.process_value_dot = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [time_step]
    data.time_step = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [command]
    data.command = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [p]
    data.p = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [i]
    data.i = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [d]
    data.d = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [i_clamp]
    data.i_clamp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [antiwindup]
    data.antiwindup = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 81;
  }

  static datatype() {
    // Returns string type for a message object
    return 'control_msgs/JointControllerState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '987ad85e4756f3aef7f1e5e7fe0595d1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This message presents current controller state of one joint.
    
    # Header timestamp should be update time of controller state
    Header header
    
    # The set point, that is, desired state.
    float64 set_point
    
    # Current value of the process (ie: latest sensor measurement on the controlled value).
    float64 process_value
    
    # First time-derivative of the process value.
    float64 process_value_dot
    
    # The error of the controlled value, essentially process_value - set_point (for a regular PID implementation).
    float64 error
    
    # Time between two consecutive updates/execution of the control law.
    float64 time_step
    
    # Current output of the controller.
    float64 command
    
    # Current PID parameters of the controller.
    float64 p
    float64 i
    float64 d
    float64 i_clamp
    bool antiwindup
    
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
    const resolved = new JointControllerState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.set_point !== undefined) {
      resolved.set_point = msg.set_point;
    }
    else {
      resolved.set_point = 0.0
    }

    if (msg.process_value !== undefined) {
      resolved.process_value = msg.process_value;
    }
    else {
      resolved.process_value = 0.0
    }

    if (msg.process_value_dot !== undefined) {
      resolved.process_value_dot = msg.process_value_dot;
    }
    else {
      resolved.process_value_dot = 0.0
    }

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = 0.0
    }

    if (msg.time_step !== undefined) {
      resolved.time_step = msg.time_step;
    }
    else {
      resolved.time_step = 0.0
    }

    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = 0.0
    }

    if (msg.p !== undefined) {
      resolved.p = msg.p;
    }
    else {
      resolved.p = 0.0
    }

    if (msg.i !== undefined) {
      resolved.i = msg.i;
    }
    else {
      resolved.i = 0.0
    }

    if (msg.d !== undefined) {
      resolved.d = msg.d;
    }
    else {
      resolved.d = 0.0
    }

    if (msg.i_clamp !== undefined) {
      resolved.i_clamp = msg.i_clamp;
    }
    else {
      resolved.i_clamp = 0.0
    }

    if (msg.antiwindup !== undefined) {
      resolved.antiwindup = msg.antiwindup;
    }
    else {
      resolved.antiwindup = false
    }

    return resolved;
    }
};

module.exports = JointControllerState;
