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

class TowerPickUpRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tower_index = null;
    }
    else {
      if (initObj.hasOwnProperty('tower_index')) {
        this.tower_index = initObj.tower_index
      }
      else {
        this.tower_index = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TowerPickUpRequest
    // Serialize message field [tower_index]
    bufferOffset = _serializer.int32(obj.tower_index, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TowerPickUpRequest
    let len;
    let data = new TowerPickUpRequest(null);
    // Deserialize message field [tower_index]
    data.tower_index = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/TowerPickUpRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e8bd24109f26b6d833bc4570d67d71c9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 tower_index
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TowerPickUpRequest(null);
    if (msg.tower_index !== undefined) {
      resolved.tower_index = msg.tower_index;
    }
    else {
      resolved.tower_index = 0
    }

    return resolved;
    }
};

class TowerPickUpResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TowerPickUpResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TowerPickUpResponse
    let len;
    let data = new TowerPickUpResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/TowerPickUpResponse';
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
    const resolved = new TowerPickUpResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: TowerPickUpRequest,
  Response: TowerPickUpResponse,
  md5sum() { return 'e8bd24109f26b6d833bc4570d67d71c9'; },
  datatype() { return 'jsk_recognition_msgs/TowerPickUp'; }
};
