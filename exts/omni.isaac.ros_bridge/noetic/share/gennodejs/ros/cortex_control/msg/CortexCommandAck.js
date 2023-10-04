// Auto-generated. Do not edit!

// (in-package cortex_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CortexCommandAck {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cortex_command_time = null;
      this.cortex_command_id = null;
      this.time_offset = null;
    }
    else {
      if (initObj.hasOwnProperty('cortex_command_time')) {
        this.cortex_command_time = initObj.cortex_command_time
      }
      else {
        this.cortex_command_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('cortex_command_id')) {
        this.cortex_command_id = initObj.cortex_command_id
      }
      else {
        this.cortex_command_id = 0;
      }
      if (initObj.hasOwnProperty('time_offset')) {
        this.time_offset = initObj.time_offset
      }
      else {
        this.time_offset = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CortexCommandAck
    // Serialize message field [cortex_command_time]
    bufferOffset = _serializer.time(obj.cortex_command_time, buffer, bufferOffset);
    // Serialize message field [cortex_command_id]
    bufferOffset = _serializer.int64(obj.cortex_command_id, buffer, bufferOffset);
    // Serialize message field [time_offset]
    bufferOffset = _serializer.duration(obj.time_offset, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CortexCommandAck
    let len;
    let data = new CortexCommandAck(null);
    // Deserialize message field [cortex_command_time]
    data.cortex_command_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [cortex_command_id]
    data.cortex_command_id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [time_offset]
    data.time_offset = _deserializer.duration(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cortex_control/CortexCommandAck';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '81e929ba8529002a2be583a8c6600952';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Command time stamp of the latest Cortex command received.
    time cortex_command_time
    
    # ID of the latest command received.
    int64 cortex_command_id
    
    # If there is a slight accumulative clock rate difference between the Cortex
    # commander and the low-level controller, the time offset gives how much
    # further ahead the controller's clock is from the Cortex commander's clock (note
    # it can be negative). So synchronizing the clocks would entail
    #
    #    <cortex_commander_time_synced> = <cortex_commander_time> + time_offset
    duration time_offset
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CortexCommandAck(null);
    if (msg.cortex_command_time !== undefined) {
      resolved.cortex_command_time = msg.cortex_command_time;
    }
    else {
      resolved.cortex_command_time = {secs: 0, nsecs: 0}
    }

    if (msg.cortex_command_id !== undefined) {
      resolved.cortex_command_id = msg.cortex_command_id;
    }
    else {
      resolved.cortex_command_id = 0
    }

    if (msg.time_offset !== undefined) {
      resolved.time_offset = msg.time_offset;
    }
    else {
      resolved.time_offset = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

module.exports = CortexCommandAck;
