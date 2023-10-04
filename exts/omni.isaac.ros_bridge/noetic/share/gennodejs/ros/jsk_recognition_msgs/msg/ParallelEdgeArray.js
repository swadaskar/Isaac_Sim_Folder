// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ParallelEdge = require('./ParallelEdge.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ParallelEdgeArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.edge_groups = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('edge_groups')) {
        this.edge_groups = initObj.edge_groups
      }
      else {
        this.edge_groups = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ParallelEdgeArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [edge_groups]
    // Serialize the length for message field [edge_groups]
    bufferOffset = _serializer.uint32(obj.edge_groups.length, buffer, bufferOffset);
    obj.edge_groups.forEach((val) => {
      bufferOffset = ParallelEdge.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ParallelEdgeArray
    let len;
    let data = new ParallelEdgeArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [edge_groups]
    // Deserialize array length for message field [edge_groups]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.edge_groups = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.edge_groups[i] = ParallelEdge.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.edge_groups.forEach((val) => {
      length += ParallelEdge.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/ParallelEdgeArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7c8ef4f5976c55fb32293ceaa19a1189';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    ParallelEdge[] edge_groups
    
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
    MSG: jsk_recognition_msgs/ParallelEdge
    Header header
    pcl_msgs/PointIndices[] cluster_indices
    pcl_msgs/ModelCoefficients[] coefficients
    
    ================================================================================
    MSG: pcl_msgs/PointIndices
    Header header
    int32[] indices
    
    
    ================================================================================
    MSG: pcl_msgs/ModelCoefficients
    Header header
    float32[] values
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ParallelEdgeArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.edge_groups !== undefined) {
      resolved.edge_groups = new Array(msg.edge_groups.length);
      for (let i = 0; i < resolved.edge_groups.length; ++i) {
        resolved.edge_groups[i] = ParallelEdge.Resolve(msg.edge_groups[i]);
      }
    }
    else {
      resolved.edge_groups = []
    }

    return resolved;
    }
};

module.exports = ParallelEdgeArray;
