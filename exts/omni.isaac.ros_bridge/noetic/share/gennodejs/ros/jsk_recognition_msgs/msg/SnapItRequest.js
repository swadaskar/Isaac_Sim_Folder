// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.msg)


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

class SnapItRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.model_type = null;
      this.target_plane = null;
      this.center = null;
      this.direction = null;
      this.radius = null;
      this.height = null;
      this.max_distance = null;
      this.eps_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('model_type')) {
        this.model_type = initObj.model_type
      }
      else {
        this.model_type = 0;
      }
      if (initObj.hasOwnProperty('target_plane')) {
        this.target_plane = initObj.target_plane
      }
      else {
        this.target_plane = new geometry_msgs.msg.PolygonStamped();
      }
      if (initObj.hasOwnProperty('center')) {
        this.center = initObj.center
      }
      else {
        this.center = new geometry_msgs.msg.PointStamped();
      }
      if (initObj.hasOwnProperty('direction')) {
        this.direction = initObj.direction
      }
      else {
        this.direction = new geometry_msgs.msg.Vector3Stamped();
      }
      if (initObj.hasOwnProperty('radius')) {
        this.radius = initObj.radius
      }
      else {
        this.radius = 0.0;
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0.0;
      }
      if (initObj.hasOwnProperty('max_distance')) {
        this.max_distance = initObj.max_distance
      }
      else {
        this.max_distance = 0.0;
      }
      if (initObj.hasOwnProperty('eps_angle')) {
        this.eps_angle = initObj.eps_angle
      }
      else {
        this.eps_angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SnapItRequest
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [model_type]
    bufferOffset = _serializer.uint8(obj.model_type, buffer, bufferOffset);
    // Serialize message field [target_plane]
    bufferOffset = geometry_msgs.msg.PolygonStamped.serialize(obj.target_plane, buffer, bufferOffset);
    // Serialize message field [center]
    bufferOffset = geometry_msgs.msg.PointStamped.serialize(obj.center, buffer, bufferOffset);
    // Serialize message field [direction]
    bufferOffset = geometry_msgs.msg.Vector3Stamped.serialize(obj.direction, buffer, bufferOffset);
    // Serialize message field [radius]
    bufferOffset = _serializer.float64(obj.radius, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.float64(obj.height, buffer, bufferOffset);
    // Serialize message field [max_distance]
    bufferOffset = _serializer.float64(obj.max_distance, buffer, bufferOffset);
    // Serialize message field [eps_angle]
    bufferOffset = _serializer.float64(obj.eps_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SnapItRequest
    let len;
    let data = new SnapItRequest(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [model_type]
    data.model_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [target_plane]
    data.target_plane = geometry_msgs.msg.PolygonStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [center]
    data.center = geometry_msgs.msg.PointStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [direction]
    data.direction = geometry_msgs.msg.Vector3Stamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [radius]
    data.radius = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max_distance]
    data.max_distance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [eps_angle]
    data.eps_angle = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += geometry_msgs.msg.PolygonStamped.getMessageSize(object.target_plane);
    length += geometry_msgs.msg.PointStamped.getMessageSize(object.center);
    length += geometry_msgs.msg.Vector3Stamped.getMessageSize(object.direction);
    return length + 33;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/SnapItRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5733f480694296678d81cff0483b399b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new SnapItRequest(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.model_type !== undefined) {
      resolved.model_type = msg.model_type;
    }
    else {
      resolved.model_type = 0
    }

    if (msg.target_plane !== undefined) {
      resolved.target_plane = geometry_msgs.msg.PolygonStamped.Resolve(msg.target_plane)
    }
    else {
      resolved.target_plane = new geometry_msgs.msg.PolygonStamped()
    }

    if (msg.center !== undefined) {
      resolved.center = geometry_msgs.msg.PointStamped.Resolve(msg.center)
    }
    else {
      resolved.center = new geometry_msgs.msg.PointStamped()
    }

    if (msg.direction !== undefined) {
      resolved.direction = geometry_msgs.msg.Vector3Stamped.Resolve(msg.direction)
    }
    else {
      resolved.direction = new geometry_msgs.msg.Vector3Stamped()
    }

    if (msg.radius !== undefined) {
      resolved.radius = msg.radius;
    }
    else {
      resolved.radius = 0.0
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0.0
    }

    if (msg.max_distance !== undefined) {
      resolved.max_distance = msg.max_distance;
    }
    else {
      resolved.max_distance = 0.0
    }

    if (msg.eps_angle !== undefined) {
      resolved.eps_angle = msg.eps_angle;
    }
    else {
      resolved.eps_angle = 0.0
    }

    return resolved;
    }
};

// Constants for message
SnapItRequest.Constants = {
  MODEL_PLANE: 0,
  MODEL_CYLINDER: 1,
}

module.exports = SnapItRequest;
