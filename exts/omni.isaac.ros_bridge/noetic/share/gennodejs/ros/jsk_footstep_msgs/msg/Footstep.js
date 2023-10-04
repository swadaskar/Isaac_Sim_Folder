// Auto-generated. Do not edit!

// (in-package jsk_footstep_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Footstep {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.leg = null;
      this.pose = null;
      this.duration = null;
      this.footstep_group = null;
      this.dimensions = null;
      this.offset = null;
      this.swing_height = null;
      this.cost = null;
    }
    else {
      if (initObj.hasOwnProperty('leg')) {
        this.leg = initObj.leg
      }
      else {
        this.leg = 0;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('footstep_group')) {
        this.footstep_group = initObj.footstep_group
      }
      else {
        this.footstep_group = 0;
      }
      if (initObj.hasOwnProperty('dimensions')) {
        this.dimensions = initObj.dimensions
      }
      else {
        this.dimensions = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('offset')) {
        this.offset = initObj.offset
      }
      else {
        this.offset = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('swing_height')) {
        this.swing_height = initObj.swing_height
      }
      else {
        this.swing_height = 0.0;
      }
      if (initObj.hasOwnProperty('cost')) {
        this.cost = initObj.cost
      }
      else {
        this.cost = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Footstep
    // Serialize message field [leg]
    bufferOffset = _serializer.uint8(obj.leg, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.duration(obj.duration, buffer, bufferOffset);
    // Serialize message field [footstep_group]
    bufferOffset = _serializer.uint32(obj.footstep_group, buffer, bufferOffset);
    // Serialize message field [dimensions]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.dimensions, buffer, bufferOffset);
    // Serialize message field [offset]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.offset, buffer, bufferOffset);
    // Serialize message field [swing_height]
    bufferOffset = _serializer.float32(obj.swing_height, buffer, bufferOffset);
    // Serialize message field [cost]
    bufferOffset = _serializer.float32(obj.cost, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Footstep
    let len;
    let data = new Footstep(null);
    // Deserialize message field [leg]
    data.leg = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.duration(buffer, bufferOffset);
    // Deserialize message field [footstep_group]
    data.footstep_group = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [dimensions]
    data.dimensions = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [offset]
    data.offset = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [swing_height]
    data.swing_height = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cost]
    data.cost = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 125;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_footstep_msgs/Footstep';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd890b275b63a90fe5f22a21e9a879971';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 RIGHT=2
    uint8 LEFT=1
    
    # Constants to visualize progress
    uint8 REJECTED=3
    uint8 APPROVED=4
    
    ## limb_indicator values
    uint8 LLEG=1
    uint8 RLEG=2
    uint8 LARM=5
    uint8 RARM=6
    
    uint8 leg ## value should be one of limb_indicator values.
    geometry_msgs/Pose pose ## 'pose' represents nominal pose. It may be an end-effector of limb.
    duration duration
    # optional parameters
    uint32 footstep_group
    geometry_msgs/Vector3 dimensions ## cube [length(x), width(y), height(z)]
    geometry_msgs/Vector3 offset     ## offset from pose to center of cube
    float32 swing_height
    float32 cost
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Footstep(null);
    if (msg.leg !== undefined) {
      resolved.leg = msg.leg;
    }
    else {
      resolved.leg = 0
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = {secs: 0, nsecs: 0}
    }

    if (msg.footstep_group !== undefined) {
      resolved.footstep_group = msg.footstep_group;
    }
    else {
      resolved.footstep_group = 0
    }

    if (msg.dimensions !== undefined) {
      resolved.dimensions = geometry_msgs.msg.Vector3.Resolve(msg.dimensions)
    }
    else {
      resolved.dimensions = new geometry_msgs.msg.Vector3()
    }

    if (msg.offset !== undefined) {
      resolved.offset = geometry_msgs.msg.Vector3.Resolve(msg.offset)
    }
    else {
      resolved.offset = new geometry_msgs.msg.Vector3()
    }

    if (msg.swing_height !== undefined) {
      resolved.swing_height = msg.swing_height;
    }
    else {
      resolved.swing_height = 0.0
    }

    if (msg.cost !== undefined) {
      resolved.cost = msg.cost;
    }
    else {
      resolved.cost = 0.0
    }

    return resolved;
    }
};

// Constants for message
Footstep.Constants = {
  RIGHT: 2,
  LEFT: 1,
  REJECTED: 3,
  APPROVED: 4,
  LLEG: 1,
  RLEG: 2,
  LARM: 5,
  RARM: 6,
}

module.exports = Footstep;
