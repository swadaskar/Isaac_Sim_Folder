// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BoundingBox = require('./BoundingBox.js');
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class BoundingBoxMovement {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.box = null;
      this.handle_pose = null;
      this.destination = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('box')) {
        this.box = initObj.box
      }
      else {
        this.box = new BoundingBox();
      }
      if (initObj.hasOwnProperty('handle_pose')) {
        this.handle_pose = initObj.handle_pose
      }
      else {
        this.handle_pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('destination')) {
        this.destination = initObj.destination
      }
      else {
        this.destination = new geometry_msgs.msg.PoseStamped();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BoundingBoxMovement
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [box]
    bufferOffset = BoundingBox.serialize(obj.box, buffer, bufferOffset);
    // Serialize message field [handle_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.handle_pose, buffer, bufferOffset);
    // Serialize message field [destination]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.destination, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BoundingBoxMovement
    let len;
    let data = new BoundingBoxMovement(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [box]
    data.box = BoundingBox.deserialize(buffer, bufferOffset);
    // Deserialize message field [handle_pose]
    data.handle_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [destination]
    data.destination = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += BoundingBox.getMessageSize(object.box);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.destination);
    return length + 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/BoundingBoxMovement';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '60aeb288c030d1b8a67ddeedc91812f0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    BoundingBox box
    geometry_msgs/Pose handle_pose
    geometry_msgs/PoseStamped destination
    
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
    MSG: jsk_recognition_msgs/BoundingBox
    # BoundingBox represents a oriented bounding box.
    Header header
    geometry_msgs/Pose pose
    geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)
    # You can use this field to hold value such as likelihood
    float32 value
    uint32 label
    
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
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BoundingBoxMovement(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.box !== undefined) {
      resolved.box = BoundingBox.Resolve(msg.box)
    }
    else {
      resolved.box = new BoundingBox()
    }

    if (msg.handle_pose !== undefined) {
      resolved.handle_pose = geometry_msgs.msg.Pose.Resolve(msg.handle_pose)
    }
    else {
      resolved.handle_pose = new geometry_msgs.msg.Pose()
    }

    if (msg.destination !== undefined) {
      resolved.destination = geometry_msgs.msg.PoseStamped.Resolve(msg.destination)
    }
    else {
      resolved.destination = new geometry_msgs.msg.PoseStamped()
    }

    return resolved;
    }
};

module.exports = BoundingBoxMovement;
