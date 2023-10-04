// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SparseOccupancyGridCell {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.row_index = null;
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('row_index')) {
        this.row_index = initObj.row_index
      }
      else {
        this.row_index = 0;
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SparseOccupancyGridCell
    // Serialize message field [row_index]
    bufferOffset = _serializer.int32(obj.row_index, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.float32(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SparseOccupancyGridCell
    let len;
    let data = new SparseOccupancyGridCell(null);
    // Deserialize message field [row_index]
    data.row_index = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/SparseOccupancyGridCell';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a5179e922852f82ee6322db1f097ab64';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 row_index
    float32 value
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SparseOccupancyGridCell(null);
    if (msg.row_index !== undefined) {
      resolved.row_index = msg.row_index;
    }
    else {
      resolved.row_index = 0
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0.0
    }

    return resolved;
    }
};

module.exports = SparseOccupancyGridCell;
