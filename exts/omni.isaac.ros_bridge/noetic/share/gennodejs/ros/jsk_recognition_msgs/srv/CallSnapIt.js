// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SnapItRequest = require('../msg/SnapItRequest.js');

//-----------------------------------------------------------

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class CallSnapItRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.request = null;
    }
    else {
      if (initObj.hasOwnProperty('request')) {
        this.request = initObj.request
      }
      else {
        this.request = new SnapItRequest();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CallSnapItRequest
    // Serialize message field [request]
    bufferOffset = SnapItRequest.serialize(obj.request, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CallSnapItRequest
    let len;
    let data = new CallSnapItRequest(null);
    // Deserialize message field [request]
    data.request = SnapItRequest.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += SnapItRequest.getMessageSize(object.request);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/CallSnapItRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5bf1ef2f060949ff03f39c08eb7bc6de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    jsk_recognition_msgs/SnapItRequest request
    
    ================================================================================
    MSG: jsk_recognition_msgs/SnapItRequest
    Header header
    uint8 MODEL_PLANE=0
    uint8 MODEL_CYLINDER=1
    uint8 model_type
    
    geometry_msgs/PolygonStamped target_plane
    
    geometry_msgs/PointStamped center
    geometry_msgs/Vector3Stamped direction
    float64 radius
    float64 height
    # parameters, 0 means 
    float64 max_distance
    float64 eps_angle
    
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
    MSG: geometry_msgs/PolygonStamped
    # This represents a Polygon with reference coordinate frame and timestamp
    Header header
    Polygon polygon
    
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
    ================================================================================
    MSG: geometry_msgs/PointStamped
    # This represents a Point with reference coordinate frame and timestamp
    Header header
    Point point
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Vector3Stamped
    # This represents a Vector3 with reference coordinate frame and timestamp
    Header header
    Vector3 vector
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
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
    const resolved = new CallSnapItRequest(null);
    if (msg.request !== undefined) {
      resolved.request = SnapItRequest.Resolve(msg.request)
    }
    else {
      resolved.request = new SnapItRequest()
    }

    return resolved;
    }
};

class CallSnapItResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.transformation = null;
    }
    else {
      if (initObj.hasOwnProperty('transformation')) {
        this.transformation = initObj.transformation
      }
      else {
        this.transformation = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CallSnapItResponse
    // Serialize message field [transformation]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.transformation, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CallSnapItResponse
    let len;
    let data = new CallSnapItResponse(null);
    // Deserialize message field [transformation]
    data.transformation = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/CallSnapItResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd787e2767b5ea7b19a81c647df92a8de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose transformation
    
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CallSnapItResponse(null);
    if (msg.transformation !== undefined) {
      resolved.transformation = geometry_msgs.msg.Pose.Resolve(msg.transformation)
    }
    else {
      resolved.transformation = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = {
  Request: CallSnapItRequest,
  Response: CallSnapItResponse,
  md5sum() { return '00529f1ccb0137f3a14b6d5b281f9a3c'; },
  datatype() { return 'jsk_recognition_msgs/CallSnapIt'; }
};
