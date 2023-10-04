// Auto-generated. Do not edit!

// (in-package dynamic_reconfigure.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BoolParameter = require('./BoolParameter.js');
let IntParameter = require('./IntParameter.js');
let StrParameter = require('./StrParameter.js');
let DoubleParameter = require('./DoubleParameter.js');
let GroupState = require('./GroupState.js');

//-----------------------------------------------------------

class Config {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bools = null;
      this.ints = null;
      this.strs = null;
      this.doubles = null;
      this.groups = null;
    }
    else {
      if (initObj.hasOwnProperty('bools')) {
        this.bools = initObj.bools
      }
      else {
        this.bools = [];
      }
      if (initObj.hasOwnProperty('ints')) {
        this.ints = initObj.ints
      }
      else {
        this.ints = [];
      }
      if (initObj.hasOwnProperty('strs')) {
        this.strs = initObj.strs
      }
      else {
        this.strs = [];
      }
      if (initObj.hasOwnProperty('doubles')) {
        this.doubles = initObj.doubles
      }
      else {
        this.doubles = [];
      }
      if (initObj.hasOwnProperty('groups')) {
        this.groups = initObj.groups
      }
      else {
        this.groups = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Config
    // Serialize message field [bools]
    // Serialize the length for message field [bools]
    bufferOffset = _serializer.uint32(obj.bools.length, buffer, bufferOffset);
    obj.bools.forEach((val) => {
      bufferOffset = BoolParameter.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [ints]
    // Serialize the length for message field [ints]
    bufferOffset = _serializer.uint32(obj.ints.length, buffer, bufferOffset);
    obj.ints.forEach((val) => {
      bufferOffset = IntParameter.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [strs]
    // Serialize the length for message field [strs]
    bufferOffset = _serializer.uint32(obj.strs.length, buffer, bufferOffset);
    obj.strs.forEach((val) => {
      bufferOffset = StrParameter.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [doubles]
    // Serialize the length for message field [doubles]
    bufferOffset = _serializer.uint32(obj.doubles.length, buffer, bufferOffset);
    obj.doubles.forEach((val) => {
      bufferOffset = DoubleParameter.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [groups]
    // Serialize the length for message field [groups]
    bufferOffset = _serializer.uint32(obj.groups.length, buffer, bufferOffset);
    obj.groups.forEach((val) => {
      bufferOffset = GroupState.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Config
    let len;
    let data = new Config(null);
    // Deserialize message field [bools]
    // Deserialize array length for message field [bools]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.bools = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.bools[i] = BoolParameter.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [ints]
    // Deserialize array length for message field [ints]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.ints = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.ints[i] = IntParameter.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [strs]
    // Deserialize array length for message field [strs]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.strs = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.strs[i] = StrParameter.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [doubles]
    // Deserialize array length for message field [doubles]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.doubles = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.doubles[i] = DoubleParameter.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [groups]
    // Deserialize array length for message field [groups]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.groups = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.groups[i] = GroupState.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.bools.forEach((val) => {
      length += BoolParameter.getMessageSize(val);
    });
    object.ints.forEach((val) => {
      length += IntParameter.getMessageSize(val);
    });
    object.strs.forEach((val) => {
      length += StrParameter.getMessageSize(val);
    });
    object.doubles.forEach((val) => {
      length += DoubleParameter.getMessageSize(val);
    });
    object.groups.forEach((val) => {
      length += GroupState.getMessageSize(val);
    });
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dynamic_reconfigure/Config';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '958f16a05573709014982821e6822580';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Config(null);
    if (msg.bools !== undefined) {
      resolved.bools = new Array(msg.bools.length);
      for (let i = 0; i < resolved.bools.length; ++i) {
        resolved.bools[i] = BoolParameter.Resolve(msg.bools[i]);
      }
    }
    else {
      resolved.bools = []
    }

    if (msg.ints !== undefined) {
      resolved.ints = new Array(msg.ints.length);
      for (let i = 0; i < resolved.ints.length; ++i) {
        resolved.ints[i] = IntParameter.Resolve(msg.ints[i]);
      }
    }
    else {
      resolved.ints = []
    }

    if (msg.strs !== undefined) {
      resolved.strs = new Array(msg.strs.length);
      for (let i = 0; i < resolved.strs.length; ++i) {
        resolved.strs[i] = StrParameter.Resolve(msg.strs[i]);
      }
    }
    else {
      resolved.strs = []
    }

    if (msg.doubles !== undefined) {
      resolved.doubles = new Array(msg.doubles.length);
      for (let i = 0; i < resolved.doubles.length; ++i) {
        resolved.doubles[i] = DoubleParameter.Resolve(msg.doubles[i]);
      }
    }
    else {
      resolved.doubles = []
    }

    if (msg.groups !== undefined) {
      resolved.groups = new Array(msg.groups.length);
      for (let i = 0; i < resolved.groups.length; ++i) {
        resolved.groups[i] = GroupState.Resolve(msg.groups[i]);
      }
    }
    else {
      resolved.groups = []
    }

    return resolved;
    }
};

module.exports = Config;
