// Auto-generated. Do not edit!

// (in-package jsk_recognition_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class TowerRobotMoveCommandRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_index = null;
      this.plate_index = null;
      this.from_tower = null;
      this.to_tower = null;
      this.option_command = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_index')) {
        this.robot_index = initObj.robot_index
      }
      else {
        this.robot_index = 0;
      }
      if (initObj.hasOwnProperty('plate_index')) {
        this.plate_index = initObj.plate_index
      }
      else {
        this.plate_index = 0;
      }
      if (initObj.hasOwnProperty('from_tower')) {
        this.from_tower = initObj.from_tower
      }
      else {
        this.from_tower = 0;
      }
      if (initObj.hasOwnProperty('to_tower')) {
        this.to_tower = initObj.to_tower
      }
      else {
        this.to_tower = 0;
      }
      if (initObj.hasOwnProperty('option_command')) {
        this.option_command = initObj.option_command
      }
      else {
        this.option_command = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TowerRobotMoveCommandRequest
    // Serialize message field [robot_index]
    bufferOffset = _serializer.int32(obj.robot_index, buffer, bufferOffset);
    // Serialize message field [plate_index]
    bufferOffset = _serializer.int32(obj.plate_index, buffer, bufferOffset);
    // Serialize message field [from_tower]
    bufferOffset = _serializer.int32(obj.from_tower, buffer, bufferOffset);
    // Serialize message field [to_tower]
    bufferOffset = _serializer.int32(obj.to_tower, buffer, bufferOffset);
    // Serialize message field [option_command]
    bufferOffset = _serializer.int32(obj.option_command, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TowerRobotMoveCommandRequest
    let len;
    let data = new TowerRobotMoveCommandRequest(null);
    // Deserialize message field [robot_index]
    data.robot_index = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [plate_index]
    data.plate_index = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [from_tower]
    data.from_tower = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [to_tower]
    data.to_tower = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [option_command]
    data.option_command = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/TowerRobotMoveCommandRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aadba056bdce0494569ab50ecd2ec90c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # a service to move robot with tower index
    byte ROBOT1=1
    byte ROBOT2=2
    byte ROBOT3=3
    
    byte PLATE_SMALL=1
    byte PLATE_MIDDLE=2
    byte PLATE_LARGE=3
    
    byte TOWER_LOWEST=1
    byte TOWER_MIDDLE=2
    byte TOWER_HIGHEST=3
    byte TOWER_LOWEST2=1
    
    byte OPTION_NONE=0
    byte OPTION_MOVE_INITIAL=1
    
    int32 robot_index
    int32 plate_index
    int32 from_tower
    int32 to_tower
    int32 option_command
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TowerRobotMoveCommandRequest(null);
    if (msg.robot_index !== undefined) {
      resolved.robot_index = msg.robot_index;
    }
    else {
      resolved.robot_index = 0
    }

    if (msg.plate_index !== undefined) {
      resolved.plate_index = msg.plate_index;
    }
    else {
      resolved.plate_index = 0
    }

    if (msg.from_tower !== undefined) {
      resolved.from_tower = msg.from_tower;
    }
    else {
      resolved.from_tower = 0
    }

    if (msg.to_tower !== undefined) {
      resolved.to_tower = msg.to_tower;
    }
    else {
      resolved.to_tower = 0
    }

    if (msg.option_command !== undefined) {
      resolved.option_command = msg.option_command;
    }
    else {
      resolved.option_command = 0
    }

    return resolved;
    }
};

// Constants for message
TowerRobotMoveCommandRequest.Constants = {
  ROBOT1: 1,
  ROBOT2: 2,
  ROBOT3: 3,
  PLATE_SMALL: 1,
  PLATE_MIDDLE: 2,
  PLATE_LARGE: 3,
  TOWER_LOWEST: 1,
  TOWER_MIDDLE: 2,
  TOWER_HIGHEST: 3,
  TOWER_LOWEST2: 1,
  OPTION_NONE: 0,
  OPTION_MOVE_INITIAL: 1,
}

class TowerRobotMoveCommandResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TowerRobotMoveCommandResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TowerRobotMoveCommandResponse
    let len;
    let data = new TowerRobotMoveCommandResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'jsk_recognition_msgs/TowerRobotMoveCommandResponse';
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
    const resolved = new TowerRobotMoveCommandResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: TowerRobotMoveCommandRequest,
  Response: TowerRobotMoveCommandResponse,
  md5sum() { return 'aadba056bdce0494569ab50ecd2ec90c'; },
  datatype() { return 'jsk_recognition_msgs/TowerRobotMoveCommand'; }
};
