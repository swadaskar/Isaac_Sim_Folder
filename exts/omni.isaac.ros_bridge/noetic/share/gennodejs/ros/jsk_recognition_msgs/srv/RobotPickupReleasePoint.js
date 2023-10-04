// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class RobotPickupReleasePointRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.target_point = null;
      this.pick_or_release = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('target_point')) {
        this.target_point = initObj.target_point
      }
      else {
        this.target_point = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('pick_or_release')) {
        this.pick_or_release = initObj.pick_or_release
      }
      else {
        this.pick_or_release = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotPickupReleasePointRequest
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [target_point]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.target_point, buffer, bufferOffset);
    // Serialize message field [pick_or_release]
    bufferOffset = _serializer.byte(obj.pick_or_release, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotPickupReleasePointRequest
    let len;
    let data = new RobotPickupReleasePointRequest(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [target_point]
    data.target_point = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [pick_or_release]
    data.pick_or_release = _deserializer.byte(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 25;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/RobotPickupReleasePointRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'deed053f0da0bc3c530cdf92dcf06642';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    geometry_msgs/Point target_point
    byte pick_or_release  # 0 -> pick, 1 -> release
    
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
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotPickupReleasePointRequest(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.target_point !== undefined) {
      resolved.target_point = geometry_msgs.msg.Point.Resolve(msg.target_point)
    }
    else {
      resolved.target_point = new geometry_msgs.msg.Point()
    }

    if (msg.pick_or_release !== undefined) {
      resolved.pick_or_release = msg.pick_or_release;
    }
    else {
      resolved.pick_or_release = 0
    }

    return resolved;
    }
};

class RobotPickupReleasePointResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotPickupReleasePointResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotPickupReleasePointResponse
    let len;
    let data = new RobotPickupReleasePointResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/RobotPickupReleasePointResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotPickupReleasePointResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: RobotPickupReleasePointRequest,
  Response: RobotPickupReleasePointResponse,
  md5sum() { return 'b6469871ddb5ec099685daff904ecc2f'; },
  datatype() { return 'jsk_recognition_msgs/RobotPickupReleasePoint'; }
};
