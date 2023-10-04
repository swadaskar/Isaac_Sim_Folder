// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Rect = require('../msg/Rect.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class NonMaximumSuppressionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rect = null;
      this.threshold = null;
    }
    else {
      if (initObj.hasOwnProperty('rect')) {
        this.rect = initObj.rect
      }
      else {
        this.rect = [];
      }
      if (initObj.hasOwnProperty('threshold')) {
        this.threshold = initObj.threshold
      }
      else {
        this.threshold = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NonMaximumSuppressionRequest
    // Serialize message field [rect]
    // Serialize the length for message field [rect]
    bufferOffset = _serializer.uint32(obj.rect.length, buffer, bufferOffset);
    obj.rect.forEach((val) => {
      bufferOffset = Rect.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [threshold]
    bufferOffset = _serializer.float32(obj.threshold, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NonMaximumSuppressionRequest
    let len;
    let data = new NonMaximumSuppressionRequest(null);
    // Deserialize message field [rect]
    // Deserialize array length for message field [rect]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.rect = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.rect[i] = Rect.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [threshold]
    data.threshold = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 16 * object.rect.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/NonMaximumSuppressionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '54b7e6632715e9e6592b503a3c881dfc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    jsk_recognition_msgs/Rect[] rect
    float32 threshold
    
    ================================================================================
    MSG: jsk_recognition_msgs/Rect
    int32 x
    int32 y
    int32 width
    int32 height
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NonMaximumSuppressionRequest(null);
    if (msg.rect !== undefined) {
      resolved.rect = new Array(msg.rect.length);
      for (let i = 0; i < resolved.rect.length; ++i) {
        resolved.rect[i] = Rect.Resolve(msg.rect[i]);
      }
    }
    else {
      resolved.rect = []
    }

    if (msg.threshold !== undefined) {
      resolved.threshold = msg.threshold;
    }
    else {
      resolved.threshold = 0.0
    }

    return resolved;
    }
};

class NonMaximumSuppressionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bbox = null;
      this.bbox_count = null;
    }
    else {
      if (initObj.hasOwnProperty('bbox')) {
        this.bbox = initObj.bbox
      }
      else {
        this.bbox = [];
      }
      if (initObj.hasOwnProperty('bbox_count')) {
        this.bbox_count = initObj.bbox_count
      }
      else {
        this.bbox_count = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NonMaximumSuppressionResponse
    // Serialize message field [bbox]
    // Serialize the length for message field [bbox]
    bufferOffset = _serializer.uint32(obj.bbox.length, buffer, bufferOffset);
    obj.bbox.forEach((val) => {
      bufferOffset = Rect.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [bbox_count]
    bufferOffset = _serializer.int64(obj.bbox_count, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NonMaximumSuppressionResponse
    let len;
    let data = new NonMaximumSuppressionResponse(null);
    // Deserialize message field [bbox]
    // Deserialize array length for message field [bbox]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.bbox = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.bbox[i] = Rect.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [bbox_count]
    data.bbox_count = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 16 * object.bbox.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/NonMaximumSuppressionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8db21435e67f6d13fc94ccbd355f30f1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    jsk_recognition_msgs/Rect[] bbox
    int64 bbox_count
    
    
    ================================================================================
    MSG: jsk_recognition_msgs/Rect
    int32 x
    int32 y
    int32 width
    int32 height
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NonMaximumSuppressionResponse(null);
    if (msg.bbox !== undefined) {
      resolved.bbox = new Array(msg.bbox.length);
      for (let i = 0; i < resolved.bbox.length; ++i) {
        resolved.bbox[i] = Rect.Resolve(msg.bbox[i]);
      }
    }
    else {
      resolved.bbox = []
    }

    if (msg.bbox_count !== undefined) {
      resolved.bbox_count = msg.bbox_count;
    }
    else {
      resolved.bbox_count = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: NonMaximumSuppressionRequest,
  Response: NonMaximumSuppressionResponse,
  md5sum() { return 'b5f1e6a38f5b631194f786d010dab9fa'; },
  datatype() { return 'jsk_recognition_msgs/NonMaximumSuppression'; }
};
