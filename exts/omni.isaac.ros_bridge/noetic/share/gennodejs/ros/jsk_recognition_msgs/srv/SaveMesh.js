// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BoundingBox = require('../msg/BoundingBox.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SaveMeshRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ground_frame_id = null;
      this.box = null;
    }
    else {
      if (initObj.hasOwnProperty('ground_frame_id')) {
        this.ground_frame_id = initObj.ground_frame_id
      }
      else {
        this.ground_frame_id = '';
      }
      if (initObj.hasOwnProperty('box')) {
        this.box = initObj.box
      }
      else {
        this.box = new BoundingBox();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SaveMeshRequest
    // Serialize message field [ground_frame_id]
    bufferOffset = _serializer.string(obj.ground_frame_id, buffer, bufferOffset);
    // Serialize message field [box]
    bufferOffset = BoundingBox.serialize(obj.box, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SaveMeshRequest
    let len;
    let data = new SaveMeshRequest(null);
    // Deserialize message field [ground_frame_id]
    data.ground_frame_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [box]
    data.box = BoundingBox.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.ground_frame_id);
    length += BoundingBox.getMessageSize(object.box);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/SaveMeshRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aedbb75b25dc1f22d6170286e35b1132';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string ground_frame_id  # to create solid model
    jsk_recognition_msgs/BoundingBox box  # to crop mesh
    
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
    const resolved = new SaveMeshRequest(null);
    if (msg.ground_frame_id !== undefined) {
      resolved.ground_frame_id = msg.ground_frame_id;
    }
    else {
      resolved.ground_frame_id = ''
    }

    if (msg.box !== undefined) {
      resolved.box = BoundingBox.Resolve(msg.box)
    }
    else {
      resolved.box = new BoundingBox()
    }

    return resolved;
    }
};

class SaveMeshResponse {
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
    // Serializes a message object of type SaveMeshResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SaveMeshResponse
    let len;
    let data = new SaveMeshResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/SaveMeshResponse';
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
    const resolved = new SaveMeshResponse(null);
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
  Request: SaveMeshRequest,
  Response: SaveMeshResponse,
  md5sum() { return 'bac972d77fa349fd55cd1ee98f47a7b0'; },
  datatype() { return 'jsk_recognition_msgs/SaveMesh'; }
};
