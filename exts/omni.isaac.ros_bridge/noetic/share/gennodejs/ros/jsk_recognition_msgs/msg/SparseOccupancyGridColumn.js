// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SparseOccupancyGridCell = require('./SparseOccupancyGridCell.js');

//-----------------------------------------------------------

class SparseOccupancyGridColumn {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.column_index = null;
      this.cells = null;
    }
    else {
      if (initObj.hasOwnProperty('column_index')) {
        this.column_index = initObj.column_index
      }
      else {
        this.column_index = 0;
      }
      if (initObj.hasOwnProperty('cells')) {
        this.cells = initObj.cells
      }
      else {
        this.cells = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SparseOccupancyGridColumn
    // Serialize message field [column_index]
    bufferOffset = _serializer.int32(obj.column_index, buffer, bufferOffset);
    // Serialize message field [cells]
    // Serialize the length for message field [cells]
    bufferOffset = _serializer.uint32(obj.cells.length, buffer, bufferOffset);
    obj.cells.forEach((val) => {
      bufferOffset = SparseOccupancyGridCell.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SparseOccupancyGridColumn
    let len;
    let data = new SparseOccupancyGridColumn(null);
    // Deserialize message field [column_index]
    data.column_index = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [cells]
    // Deserialize array length for message field [cells]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.cells = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.cells[i] = SparseOccupancyGridCell.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.cells.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jsk_recognition_msgs/SparseOccupancyGridColumn';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '55074b193e722d5ead092ffe27f06522';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 column_index
    SparseOccupancyGridCell[] cells
    
    ================================================================================
    MSG: jsk_recognition_msgs/SparseOccupancyGridCell
    int32 row_index
    float32 value
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SparseOccupancyGridColumn(null);
    if (msg.column_index !== undefined) {
      resolved.column_index = msg.column_index;
    }
    else {
      resolved.column_index = 0
    }

    if (msg.cells !== undefined) {
      resolved.cells = new Array(msg.cells.length);
      for (let i = 0; i < resolved.cells.length; ++i) {
        resolved.cells[i] = SparseOccupancyGridCell.Resolve(msg.cells[i]);
      }
    }
    else {
      resolved.cells = []
    }

    return resolved;
    }
};

module.exports = SparseOccupancyGridColumn;
