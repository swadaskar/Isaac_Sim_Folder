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
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetTemplateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.type = null;
      this.image = null;
      this.dimx = null;
      this.dimy = null;
      this.relativepose = null;
      this.savefilename = null;
    }
    else {
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = '';
      }
      if (initObj.hasOwnProperty('image')) {
        this.image = initObj.image
      }
      else {
        this.image = new sensor_msgs.msg.Image();
      }
      if (initObj.hasOwnProperty('dimx')) {
        this.dimx = initObj.dimx
      }
      else {
        this.dimx = 0.0;
      }
      if (initObj.hasOwnProperty('dimy')) {
        this.dimy = initObj.dimy
      }
      else {
        this.dimy = 0.0;
      }
      if (initObj.hasOwnProperty('relativepose')) {
        this.relativepose = initObj.relativepose
      }
      else {
        this.relativepose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('savefilename')) {
        this.savefilename = initObj.savefilename
      }
      else {
        this.savefilename = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetTemplateRequest
    // Serialize message field [type]
    bufferOffset = _serializer.string(obj.type, buffer, bufferOffset);
    // Serialize message field [image]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.image, buffer, bufferOffset);
    // Serialize message field [dimx]
    bufferOffset = _serializer.float32(obj.dimx, buffer, bufferOffset);
    // Serialize message field [dimy]
    bufferOffset = _serializer.float32(obj.dimy, buffer, bufferOffset);
    // Serialize message field [relativepose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.relativepose, buffer, bufferOffset);
    // Serialize message field [savefilename]
    bufferOffset = _serializer.string(obj.savefilename, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetTemplateRequest
    let len;
    let data = new SetTemplateRequest(null);
    // Deserialize message field [type]
    data.type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [image]
    data.image = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    // Deserialize message field [dimx]
    data.dimx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dimy]
    data.dimy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [relativepose]
    data.relativepose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [savefilename]
    data.savefilename = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.type);
    length += sensor_msgs.msg.Image.getMessageSize(object.image);
    length += _getByteLength(object.savefilename);
    return length + 72;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/SetTemplateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '116fa80f27cbdfcd76d0b57a30ef79ec';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string type
    sensor_msgs/Image image
    # dimensions of the texture in the real world (in meters)
    float32 dimx
    float32 dimy
    geometry_msgs/Pose relativepose # used to set the coordinate system of the object relative to the texture
    string savefilename # if not empty, will save the resulting pp file to here
    
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
    const resolved = new SetTemplateRequest(null);
    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = ''
    }

    if (msg.image !== undefined) {
      resolved.image = sensor_msgs.msg.Image.Resolve(msg.image)
    }
    else {
      resolved.image = new sensor_msgs.msg.Image()
    }

    if (msg.dimx !== undefined) {
      resolved.dimx = msg.dimx;
    }
    else {
      resolved.dimx = 0.0
    }

    if (msg.dimy !== undefined) {
      resolved.dimy = msg.dimy;
    }
    else {
      resolved.dimy = 0.0
    }

    if (msg.relativepose !== undefined) {
      resolved.relativepose = geometry_msgs.msg.Pose.Resolve(msg.relativepose)
    }
    else {
      resolved.relativepose = new geometry_msgs.msg.Pose()
    }

    if (msg.savefilename !== undefined) {
      resolved.savefilename = msg.savefilename;
    }
    else {
      resolved.savefilename = ''
    }

    return resolved;
    }
};

class SetTemplateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetTemplateResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetTemplateResponse
    let len;
    let data = new SetTemplateResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/SetTemplateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetTemplateResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetTemplateRequest,
  Response: SetTemplateResponse,
  md5sum() { return '116fa80f27cbdfcd76d0b57a30ef79ec'; },
  datatype() { return 'jsk_recognition_msgs/SetTemplate'; }
};
