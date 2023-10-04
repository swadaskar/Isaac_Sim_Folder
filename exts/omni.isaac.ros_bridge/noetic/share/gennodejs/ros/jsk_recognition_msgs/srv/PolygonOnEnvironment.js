// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class PolygonOnEnvironmentRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.environment_id = null;
      this.plane_index = null;
      this.polygon = null;
    }
    else {
      if (initObj.hasOwnProperty('environment_id')) {
        this.environment_id = initObj.environment_id
      }
      else {
        this.environment_id = 0;
      }
      if (initObj.hasOwnProperty('plane_index')) {
        this.plane_index = initObj.plane_index
      }
      else {
        this.plane_index = 0;
      }
      if (initObj.hasOwnProperty('polygon')) {
        this.polygon = initObj.polygon
      }
      else {
        this.polygon = new geometry_msgs.msg.PolygonStamped();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PolygonOnEnvironmentRequest
    // Serialize message field [environment_id]
    bufferOffset = _serializer.uint32(obj.environment_id, buffer, bufferOffset);
    // Serialize message field [plane_index]
    bufferOffset = _serializer.uint32(obj.plane_index, buffer, bufferOffset);
    // Serialize message field [polygon]
    bufferOffset = geometry_msgs.msg.PolygonStamped.serialize(obj.polygon, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PolygonOnEnvironmentRequest
    let len;
    let data = new PolygonOnEnvironmentRequest(null);
    // Deserialize message field [environment_id]
    data.environment_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [plane_index]
    data.plane_index = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [polygon]
    data.polygon = geometry_msgs.msg.PolygonStamped.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PolygonStamped.getMessageSize(object.polygon);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/PolygonOnEnvironmentRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5c876f97015e6a599aa3c09455882c02';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 environment_id
    uint32 plane_index
    geometry_msgs/PolygonStamped polygon
    
    ================================================================================
    MSG: geometry_msgs/PolygonStamped
    # This represents a Polygon with reference coordinate frame and timestamp
    Header header
    Polygon polygon
    
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
    MSG: geometry_msgs/Polygon
    #A specification of a polygon where the first and last points are assumed to be connected
    Point32[] points
    
    ================================================================================
    MSG: geometry_msgs/Point32
    # This contains the position of a point in free space(with 32 bits of precision).
    # It is recommeded to use Point wherever possible instead of Point32.  
    # 
    # This recommendation is to promote interoperability.  
    #
    # This message is designed to take up less space when sending
    # lots of points at once, as in the case of a PointCloud.  
    
    float32 x
    float32 y
    float32 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PolygonOnEnvironmentRequest(null);
    if (msg.environment_id !== undefined) {
      resolved.environment_id = msg.environment_id;
    }
    else {
      resolved.environment_id = 0
    }

    if (msg.plane_index !== undefined) {
      resolved.plane_index = msg.plane_index;
    }
    else {
      resolved.plane_index = 0
    }

    if (msg.polygon !== undefined) {
      resolved.polygon = geometry_msgs.msg.PolygonStamped.Resolve(msg.polygon)
    }
    else {
      resolved.polygon = new geometry_msgs.msg.PolygonStamped()
    }

    return resolved;
    }
};

class PolygonOnEnvironmentResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
      this.reason = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = false;
      }
      if (initObj.hasOwnProperty('reason')) {
        this.reason = initObj.reason
      }
      else {
        this.reason = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PolygonOnEnvironmentResponse
    // Serialize message field [result]
    bufferOffset = _serializer.bool(obj.result, buffer, bufferOffset);
    // Serialize message field [reason]
    bufferOffset = _serializer.string(obj.reason, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PolygonOnEnvironmentResponse
    let len;
    let data = new PolygonOnEnvironmentResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [reason]
    data.reason = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.reason);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/PolygonOnEnvironmentResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5d3fee08bf23ddff8ab543476a855d3f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool result
    string reason
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PolygonOnEnvironmentResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = false
    }

    if (msg.reason !== undefined) {
      resolved.reason = msg.reason;
    }
    else {
      resolved.reason = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: PolygonOnEnvironmentRequest,
  Response: PolygonOnEnvironmentResponse,
  md5sum() { return 'a8ff091210c071091863d880820e505c'; },
  datatype() { return 'jsk_recognition_msgs/PolygonOnEnvironment'; }
};
