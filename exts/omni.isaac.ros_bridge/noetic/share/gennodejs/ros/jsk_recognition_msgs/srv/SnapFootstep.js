// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let jsk_footstep_msgs = _finder('jsk_footstep_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SnapFootstepRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.input = null;
    }
    else {
      if (initObj.hasOwnProperty('input')) {
        this.input = initObj.input
      }
      else {
        this.input = new jsk_footstep_msgs.msg.FootstepArray();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SnapFootstepRequest
    // Serialize message field [input]
    bufferOffset = jsk_footstep_msgs.msg.FootstepArray.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SnapFootstepRequest
    let len;
    let data = new SnapFootstepRequest(null);
    // Deserialize message field [input]
    data.input = jsk_footstep_msgs.msg.FootstepArray.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += jsk_footstep_msgs.msg.FootstepArray.getMessageSize(object.input);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/SnapFootstepRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'acbed27d6a2c527657315248d9480de2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    jsk_footstep_msgs/FootstepArray input
    
    ================================================================================
    MSG: jsk_footstep_msgs/FootstepArray
    Header header
    Footstep[] footsteps
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
    MSG: jsk_footstep_msgs/Footstep
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
    const resolved = new SnapFootstepRequest(null);
    if (msg.input !== undefined) {
      resolved.input = jsk_footstep_msgs.msg.FootstepArray.Resolve(msg.input)
    }
    else {
      resolved.input = new jsk_footstep_msgs.msg.FootstepArray()
    }

    return resolved;
    }
};

class SnapFootstepResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.output = null;
    }
    else {
      if (initObj.hasOwnProperty('output')) {
        this.output = initObj.output
      }
      else {
        this.output = new jsk_footstep_msgs.msg.FootstepArray();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SnapFootstepResponse
    // Serialize message field [output]
    bufferOffset = jsk_footstep_msgs.msg.FootstepArray.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SnapFootstepResponse
    let len;
    let data = new SnapFootstepResponse(null);
    // Deserialize message field [output]
    data.output = jsk_footstep_msgs.msg.FootstepArray.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += jsk_footstep_msgs.msg.FootstepArray.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/SnapFootstepResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '626bcd752f55cb0f7d967a8b0b14cc89';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    jsk_footstep_msgs/FootstepArray output
    
    
    ================================================================================
    MSG: jsk_footstep_msgs/FootstepArray
    Header header
    Footstep[] footsteps
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
    MSG: jsk_footstep_msgs/Footstep
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
    const resolved = new SnapFootstepResponse(null);
    if (msg.output !== undefined) {
      resolved.output = jsk_footstep_msgs.msg.FootstepArray.Resolve(msg.output)
    }
    else {
      resolved.output = new jsk_footstep_msgs.msg.FootstepArray()
    }

    return resolved;
    }
};

module.exports = {
  Request: SnapFootstepRequest,
  Response: SnapFootstepResponse,
  md5sum() { return 'ea63ea1160bdd6c252d3a568e9f2ee4f'; },
  datatype() { return 'jsk_recognition_msgs/SnapFootstep'; }
};
