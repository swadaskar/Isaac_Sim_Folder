// Auto-generated. Do not edit!

// (in-package dynamic_reconfigure.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Config = require('../msg/Config.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class ReconfigureRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.config = null;
    }
    else {
      if (initObj.hasOwnProperty('config')) {
        this.config = initObj.config
      }
      else {
        this.config = new Config();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReconfigureRequest
    // Serialize message field [config]
    bufferOffset = Config.serialize(obj.config, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReconfigureRequest
    let len;
    let data = new ReconfigureRequest(null);
    // Deserialize message field [config]
    data.config = Config.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Config.getMessageSize(object.config);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dynamic_reconfigure/ReconfigureRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ac41a77620a4a0348b7001641796a8a1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Config config
    
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
    const resolved = new ReconfigureRequest(null);
    if (msg.config !== undefined) {
      resolved.config = Config.Resolve(msg.config)
    }
    else {
      resolved.config = new Config()
    }

    return resolved;
    }
};

class ReconfigureResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.config = null;
    }
    else {
      if (initObj.hasOwnProperty('config')) {
        this.config = initObj.config
      }
      else {
        this.config = new Config();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReconfigureResponse
    // Serialize message field [config]
    bufferOffset = Config.serialize(obj.config, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReconfigureResponse
    let len;
    let data = new ReconfigureResponse(null);
    // Deserialize message field [config]
    data.config = Config.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Config.getMessageSize(object.config);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dynamic_reconfigure/ReconfigureResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ac41a77620a4a0348b7001641796a8a1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Config config
    
    
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
    const resolved = new ReconfigureResponse(null);
    if (msg.config !== undefined) {
      resolved.config = Config.Resolve(msg.config)
    }
    else {
      resolved.config = new Config()
    }

    return resolved;
    }
};

module.exports = {
  Request: ReconfigureRequest,
  Response: ReconfigureResponse,
  md5sum() { return 'bb125d226a21982a4a98760418dc2672'; },
  datatype() { return 'dynamic_reconfigure/Reconfigure'; }
};
