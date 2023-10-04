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
let pcl_msgs = _finder('pcl_msgs');

//-----------------------------------------------------------

class ClusterPointIndices {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.cluster_indices = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('cluster_indices')) {
        this.cluster_indices = initObj.cluster_indices
      }
      else {
        this.cluster_indices = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ClusterPointIndices
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [cluster_indices]
    // Serialize the length for message field [cluster_indices]
    bufferOffset = _serializer.uint32(obj.cluster_indices.length, buffer, bufferOffset);
    obj.cluster_indices.forEach((val) => {
      bufferOffset = pcl_msgs.msg.PointIndices.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ClusterPointIndices
    let len;
    let data = new ClusterPointIndices(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [cluster_indices]
    // Deserialize array length for message field [cluster_indices]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.cluster_indices = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.cluster_indices[i] = pcl_msgs.msg.PointIndices.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.cluster_indices.forEach((val) => {
      length += pcl_msgs.msg.PointIndices.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/ClusterPointIndices';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd43e94ea5e491effac7685a42b7b9d14';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ClusterPointIndices is used to represent segmentation result.
    # Simply put, ClusterPointIndices is a list of PointIndices.
    Header header
    pcl_msgs/PointIndices[] cluster_indices
    
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
    MSG: pcl_msgs/PointIndices
    Header header
    int32[] indices
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ClusterPointIndices(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.cluster_indices !== undefined) {
      resolved.cluster_indices = new Array(msg.cluster_indices.length);
      for (let i = 0; i < resolved.cluster_indices.length; ++i) {
        resolved.cluster_indices[i] = pcl_msgs.msg.PointIndices.Resolve(msg.cluster_indices[i]);
      }
    }
    else {
      resolved.cluster_indices = []
    }

    return resolved;
    }
};

module.exports = ClusterPointIndices;
