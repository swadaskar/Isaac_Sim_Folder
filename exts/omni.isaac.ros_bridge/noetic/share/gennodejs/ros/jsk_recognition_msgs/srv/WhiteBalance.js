// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class WhiteBalanceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.reference_color = null;
      this.input = null;
    }
    else {
      if (initObj.hasOwnProperty('reference_color')) {
        this.reference_color = initObj.reference_color
      }
      else {
        this.reference_color = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('input')) {
        this.input = initObj.input
      }
      else {
        this.input = new sensor_msgs.msg.Image();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WhiteBalanceRequest
    // Check that the constant length array field [reference_color] has the right length
    if (obj.reference_color.length !== 3) {
      throw new Error('Unable to serialize array field reference_color - length must be 3')
    }
    // Serialize message field [reference_color]
    bufferOffset = _arraySerializer.float32(obj.reference_color, buffer, bufferOffset, 3);
    // Serialize message field [input]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WhiteBalanceRequest
    let len;
    let data = new WhiteBalanceRequest(null);
    // Deserialize message field [reference_color]
    data.reference_color = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [input]
    data.input = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.Image.getMessageSize(object.input);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/WhiteBalanceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd7702dac51626a53e0806ebd0ad35ca3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[3] reference_color
    sensor_msgs/Image input
    
    ================================================================================
    MSG: sensor_msgs/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of camera
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
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
    const resolved = new WhiteBalanceRequest(null);
    if (msg.reference_color !== undefined) {
      resolved.reference_color = msg.reference_color;
    }
    else {
      resolved.reference_color = new Array(3).fill(0)
    }

    if (msg.input !== undefined) {
      resolved.input = sensor_msgs.msg.Image.Resolve(msg.input)
    }
    else {
      resolved.input = new sensor_msgs.msg.Image()
    }

    return resolved;
    }
};

class WhiteBalanceResponse {
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
        this.output = new sensor_msgs.msg.Image();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WhiteBalanceResponse
    // Serialize message field [output]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WhiteBalanceResponse
    let len;
    let data = new WhiteBalanceResponse(null);
    // Deserialize message field [output]
    data.output = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.Image.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/WhiteBalanceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8eeb6eb8777baa5a80bbc676c219bfef';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/Image output
    
    
    ================================================================================
    MSG: sensor_msgs/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of camera
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
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
    const resolved = new WhiteBalanceResponse(null);
    if (msg.output !== undefined) {
      resolved.output = sensor_msgs.msg.Image.Resolve(msg.output)
    }
    else {
      resolved.output = new sensor_msgs.msg.Image()
    }

    return resolved;
    }
};

module.exports = {
  Request: WhiteBalanceRequest,
  Response: WhiteBalanceResponse,
  md5sum() { return 'a35c8b952e94474b5303dc3a82ca7672'; },
  datatype() { return 'jsk_recognition_msgs/WhiteBalance'; }
};
