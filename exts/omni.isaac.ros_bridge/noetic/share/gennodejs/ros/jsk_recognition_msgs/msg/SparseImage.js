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

//-----------------------------------------------------------

class SparseImage {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.width = null;
      this.height = null;
      this.data16 = null;
      this.data32 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0;
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0;
      }
      if (initObj.hasOwnProperty('data16')) {
        this.data16 = initObj.data16
      }
      else {
        this.data16 = [];
      }
      if (initObj.hasOwnProperty('data32')) {
        this.data32 = initObj.data32
      }
      else {
        this.data32 = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SparseImage
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.uint32(obj.width, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.uint32(obj.height, buffer, bufferOffset);
    // Serialize message field [data16]
    bufferOffset = _arraySerializer.uint16(obj.data16, buffer, bufferOffset, null);
    // Serialize message field [data32]
    bufferOffset = _arraySerializer.uint32(obj.data32, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SparseImage
    let len;
    let data = new SparseImage(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [data16]
    data.data16 = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [data32]
    data.data32 = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 2 * object.data16.length;
    length += 4 * object.data32.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/SparseImage';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7c361d829424bc5984fc0a1831f84751';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    uint32 width
    uint32 height
    
    # each uint8 or uint16 contains position of white pixel expressed like: "(x << 8 or 16) ^ y"
    # if max(width, height) > 256(=2^8) use data32 array, else use data16 array.
    uint16[] data16
    uint32[] data32
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SparseImage(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0
    }

    if (msg.data16 !== undefined) {
      resolved.data16 = msg.data16;
    }
    else {
      resolved.data16 = []
    }

    if (msg.data32 !== undefined) {
      resolved.data32 = msg.data32;
    }
    else {
      resolved.data32 = []
    }

    return resolved;
    }
};

module.exports = SparseImage;
