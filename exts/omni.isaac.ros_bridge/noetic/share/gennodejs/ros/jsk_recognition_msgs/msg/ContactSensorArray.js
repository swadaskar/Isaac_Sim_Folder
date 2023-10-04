// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ContactSensor = require('./ContactSensor.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ContactSensorArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.datas = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('datas')) {
        this.datas = initObj.datas
      }
      else {
        this.datas = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ContactSensorArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [datas]
    // Serialize the length for message field [datas]
    bufferOffset = _serializer.uint32(obj.datas.length, buffer, bufferOffset);
    obj.datas.forEach((val) => {
      bufferOffset = ContactSensor.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ContactSensorArray
    let len;
    let data = new ContactSensorArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [datas]
    // Deserialize array length for message field [datas]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.datas = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.datas[i] = ContactSensor.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.datas.forEach((val) => {
      length += ContactSensor.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/ContactSensorArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c65f16fb3a523c0b77d7e31330b214da';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    jsk_recognition_msgs/ContactSensor[] datas
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
    MSG: jsk_recognition_msgs/ContactSensor
    # Header
    Header header
    
    # Whether sensor detects contact or not
    bool contact
    
    string link_name
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ContactSensorArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.datas !== undefined) {
      resolved.datas = new Array(msg.datas.length);
      for (let i = 0; i < resolved.datas.length; ++i) {
        resolved.datas[i] = ContactSensor.Resolve(msg.datas[i]);
      }
    }
    else {
      resolved.datas = []
    }

    return resolved;
    }
};

module.exports = ContactSensorArray;
