// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SwitchTopicRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.camera_info = null;
      this.points = null;
    }
    else {
      if (initObj.hasOwnProperty('camera_info')) {
        this.camera_info = initObj.camera_info
      }
      else {
        this.camera_info = '';
      }
      if (initObj.hasOwnProperty('points')) {
        this.points = initObj.points
      }
      else {
        this.points = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SwitchTopicRequest
    // Serialize message field [camera_info]
    bufferOffset = _serializer.string(obj.camera_info, buffer, bufferOffset);
    // Serialize message field [points]
    bufferOffset = _serializer.string(obj.points, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SwitchTopicRequest
    let len;
    let data = new SwitchTopicRequest(null);
    // Deserialize message field [camera_info]
    data.camera_info = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [points]
    data.points = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.camera_info);
    length += _getByteLength(object.points);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/SwitchTopicRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e4a276b5a9b7b8fd97441d0fd991bdb9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # new topics
    string camera_info
    string points
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SwitchTopicRequest(null);
    if (msg.camera_info !== undefined) {
      resolved.camera_info = msg.camera_info;
    }
    else {
      resolved.camera_info = ''
    }

    if (msg.points !== undefined) {
      resolved.points = msg.points;
    }
    else {
      resolved.points = ''
    }

    return resolved;
    }
};

class SwitchTopicResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SwitchTopicResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SwitchTopicResponse
    let len;
    let data = new SwitchTopicResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/SwitchTopicResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SwitchTopicResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SwitchTopicRequest,
  Response: SwitchTopicResponse,
  md5sum() { return 'e4a276b5a9b7b8fd97441d0fd991bdb9'; },
  datatype() { return 'jsk_recognition_msgs/SwitchTopic'; }
};
