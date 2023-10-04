// Auto-generated. Do not edit!

// (in-package dynamic_reconfigure.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Group = require('./Group.js');
let Config = require('./Config.js');

//-----------------------------------------------------------

class ConfigDescription {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.groups = null;
      this.max = null;
      this.min = null;
      this.dflt = null;
    }
    else {
      if (initObj.hasOwnProperty('groups')) {
        this.groups = initObj.groups
      }
      else {
        this.groups = [];
      }
      if (initObj.hasOwnProperty('max')) {
        this.max = initObj.max
      }
      else {
        this.max = new Config();
      }
      if (initObj.hasOwnProperty('min')) {
        this.min = initObj.min
      }
      else {
        this.min = new Config();
      }
      if (initObj.hasOwnProperty('dflt')) {
        this.dflt = initObj.dflt
      }
      else {
        this.dflt = new Config();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConfigDescription
    // Serialize message field [groups]
    // Serialize the length for message field [groups]
    bufferOffset = _serializer.uint32(obj.groups.length, buffer, bufferOffset);
    obj.groups.forEach((val) => {
      bufferOffset = Group.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [max]
    bufferOffset = Config.serialize(obj.max, buffer, bufferOffset);
    // Serialize message field [min]
    bufferOffset = Config.serialize(obj.min, buffer, bufferOffset);
    // Serialize message field [dflt]
    bufferOffset = Config.serialize(obj.dflt, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConfigDescription
    let len;
    let data = new ConfigDescription(null);
    // Deserialize message field [groups]
    // Deserialize array length for message field [groups]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.groups = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.groups[i] = Group.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [max]
    data.max = Config.deserialize(buffer, bufferOffset);
    // Deserialize message field [min]
    data.min = Config.deserialize(buffer, bufferOffset);
    // Deserialize message field [dflt]
    data.dflt = Config.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.groups.forEach((val) => {
      length += Group.getMessageSize(val);
    });
    length += Config.getMessageSize(object.max);
    length += Config.getMessageSize(object.min);
    length += Config.getMessageSize(object.dflt);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dynamic_reconfigure/ConfigDescription';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '757ce9d44ba8ddd801bb30bc456f946f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Group[] groups
    Config max
    Config min
    Config dflt
    
    ================================================================================
    MSG: dynamic_reconfigure/Group
    string name
    string type
    ParamDescription[] parameters
    int32 parent 
    int32 id
    
    ================================================================================
    MSG: dynamic_reconfigure/ParamDescription
    string name
    string type
    uint32 level
    string description
    string edit_method
    
    ================================================================================
    MSG: dynamic_reconfigure/Config
    BoolParameter[] bools
    IntParameter[] ints
    StrParameter[] strs
    DoubleParameter[] doubles
    GroupState[] groups
    
    ================================================================================
    MSG: dynamic_reconfigure/BoolParameter
    string name
    bool value
    
    ================================================================================
    MSG: dynamic_reconfigure/IntParameter
    string name
    int32 value
    
    ================================================================================
    MSG: dynamic_reconfigure/StrParameter
    string name
    string value
    
    ================================================================================
    MSG: dynamic_reconfigure/DoubleParameter
    string name
    float64 value
    
    ================================================================================
    MSG: dynamic_reconfigure/GroupState
    string name
    bool state
    int32 id
    int32 parent
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConfigDescription(null);
    if (msg.groups !== undefined) {
      resolved.groups = new Array(msg.groups.length);
      for (let i = 0; i < resolved.groups.length; ++i) {
        resolved.groups[i] = Group.Resolve(msg.groups[i]);
      }
    }
    else {
      resolved.groups = []
    }

    if (msg.max !== undefined) {
      resolved.max = Config.Resolve(msg.max)
    }
    else {
      resolved.max = new Config()
    }

    if (msg.min !== undefined) {
      resolved.min = Config.Resolve(msg.min)
    }
    else {
      resolved.min = new Config()
    }

    if (msg.dflt !== undefined) {
      resolved.dflt = Config.Resolve(msg.dflt)
    }
    else {
      resolved.dflt = new Config()
    }

    return resolved;
    }
};

module.exports = ConfigDescription;
