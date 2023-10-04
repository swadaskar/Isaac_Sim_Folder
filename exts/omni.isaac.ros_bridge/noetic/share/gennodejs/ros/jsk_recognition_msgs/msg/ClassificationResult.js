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

class ClassificationResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.labels = null;
      this.label_names = null;
      this.label_proba = null;
      this.probabilities = null;
      this.classifier = null;
      this.target_names = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('labels')) {
        this.labels = initObj.labels
      }
      else {
        this.labels = [];
      }
      if (initObj.hasOwnProperty('label_names')) {
        this.label_names = initObj.label_names
      }
      else {
        this.label_names = [];
      }
      if (initObj.hasOwnProperty('label_proba')) {
        this.label_proba = initObj.label_proba
      }
      else {
        this.label_proba = [];
      }
      if (initObj.hasOwnProperty('probabilities')) {
        this.probabilities = initObj.probabilities
      }
      else {
        this.probabilities = [];
      }
      if (initObj.hasOwnProperty('classifier')) {
        this.classifier = initObj.classifier
      }
      else {
        this.classifier = '';
      }
      if (initObj.hasOwnProperty('target_names')) {
        this.target_names = initObj.target_names
      }
      else {
        this.target_names = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ClassificationResult
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [labels]
    bufferOffset = _arraySerializer.uint32(obj.labels, buffer, bufferOffset, null);
    // Serialize message field [label_names]
    bufferOffset = _arraySerializer.string(obj.label_names, buffer, bufferOffset, null);
    // Serialize message field [label_proba]
    bufferOffset = _arraySerializer.float64(obj.label_proba, buffer, bufferOffset, null);
    // Serialize message field [probabilities]
    bufferOffset = _arraySerializer.float64(obj.probabilities, buffer, bufferOffset, null);
    // Serialize message field [classifier]
    bufferOffset = _serializer.string(obj.classifier, buffer, bufferOffset);
    // Serialize message field [target_names]
    bufferOffset = _arraySerializer.string(obj.target_names, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ClassificationResult
    let len;
    let data = new ClassificationResult(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [labels]
    data.labels = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [label_names]
    data.label_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [label_proba]
    data.label_proba = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [probabilities]
    data.probabilities = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [classifier]
    data.classifier = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [target_names]
    data.target_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.labels.length;
    object.label_names.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += 8 * object.label_proba.length;
    length += 8 * object.probabilities.length;
    length += _getByteLength(object.classifier);
    object.target_names.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/ClassificationResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cce1f8edabff85a20e9cc013e319497c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # information about frame and timestamp
    Header header
    
    # prediction results
    uint32[] labels          # numerical labels
    string[] label_names     # non-numerical labels
    float64[] label_proba    # probabilities of labels
    float64[] probabilities  # probabilities about each classification for all target_names
    
    # information about classifier
    string classifier        # name of classifier
    string[] target_names    # set in which label_names should be
    
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
    const resolved = new ClassificationResult(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.labels !== undefined) {
      resolved.labels = msg.labels;
    }
    else {
      resolved.labels = []
    }

    if (msg.label_names !== undefined) {
      resolved.label_names = msg.label_names;
    }
    else {
      resolved.label_names = []
    }

    if (msg.label_proba !== undefined) {
      resolved.label_proba = msg.label_proba;
    }
    else {
      resolved.label_proba = []
    }

    if (msg.probabilities !== undefined) {
      resolved.probabilities = msg.probabilities;
    }
    else {
      resolved.probabilities = []
    }

    if (msg.classifier !== undefined) {
      resolved.classifier = msg.classifier;
    }
    else {
      resolved.classifier = ''
    }

    if (msg.target_names !== undefined) {
      resolved.target_names = msg.target_names;
    }
    else {
      resolved.target_names = []
    }

    return resolved;
    }
};

module.exports = ClassificationResult;
