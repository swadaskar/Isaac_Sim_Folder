// Auto-generated. Do not edit!

// (in-package nav_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let OccupancyGrid = require('../msg/OccupancyGrid.js');

//-----------------------------------------------------------

class LoadMapRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.map_url = null;
    }
    else {
      if (initObj.hasOwnProperty('map_url')) {
        this.map_url = initObj.map_url
      }
      else {
        this.map_url = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LoadMapRequest
    // Serialize message field [map_url]
    bufferOffset = _serializer.string(obj.map_url, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LoadMapRequest
    let len;
    let data = new LoadMapRequest(null);
    // Deserialize message field [map_url]
    data.map_url = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.map_url);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'nav_msgs/LoadMapRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3813ba1ae85fbcd4dc88c90f1426b90b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # URL of map resource
    # Can be an absolute path to a file: file:///path/to/maps/floor1.yaml
    # Or, relative to a ROS package: package://my_ros_package/maps/floor2.yaml
    string map_url
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LoadMapRequest(null);
    if (msg.map_url !== undefined) {
      resolved.map_url = msg.map_url;
    }
    else {
      resolved.map_url = ''
    }

    return resolved;
    }
};

class LoadMapResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.map = null;
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('map')) {
        this.map = initObj.map
      }
      else {
        this.map = new OccupancyGrid();
      }
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LoadMapResponse
    // Serialize message field [map]
    bufferOffset = OccupancyGrid.serialize(obj.map, buffer, bufferOffset);
    // Serialize message field [result]
    bufferOffset = _serializer.uint8(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LoadMapResponse
    let len;
    let data = new LoadMapResponse(null);
    // Deserialize message field [map]
    data.map = OccupancyGrid.deserialize(buffer, bufferOffset);
    // Deserialize message field [result]
    data.result = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += OccupancyGrid.getMessageSize(object.map);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'nav_msgs/LoadMapResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '079b9c828e9f7c1918bf86932fd7267e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Result code defintions
    uint8 RESULT_SUCCESS=0
    uint8 RESULT_MAP_DOES_NOT_EXIST=1
    uint8 RESULT_INVALID_MAP_DATA=2
    uint8 RESULT_INVALID_MAP_METADATA=3
    uint8 RESULT_UNDEFINED_FAILURE=255
    
    # Returned map is only valid if result equals RESULT_SUCCESS
    nav_msgs/OccupancyGrid map
    uint8 result
    
    
    ================================================================================
    MSG: nav_msgs/OccupancyGrid
    # This represents a 2-D grid map, in which each cell represents the probability of
    # occupancy.
    
    Header header 
    
    #MetaData for the map
    MapMetaData info
    
    # The map data, in row-major order, starting with (0,0).  Occupancy
    # probabilities are in the range [0,100].  Unknown is -1.
    int8[] data
    
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
    MSG: nav_msgs/MapMetaData
    # This hold basic information about the characterists of the OccupancyGrid
    
    # The time at which the map was loaded
    time map_load_time
    # The map resolution [m/cell]
    float32 resolution
    # Map width [cells]
    uint32 width
    # Map height [cells]
    uint32 height
    # The origin of the map [m, m, rad].  This is the real-world pose of the
    # cell (0,0) in the map.
    geometry_msgs/Pose origin
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
    const resolved = new LoadMapResponse(null);
    if (msg.map !== undefined) {
      resolved.map = OccupancyGrid.Resolve(msg.map)
    }
    else {
      resolved.map = new OccupancyGrid()
    }

    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = 0
    }

    return resolved;
    }
};

// Constants for message
LoadMapResponse.Constants = {
  RESULT_SUCCESS: 0,
  RESULT_MAP_DOES_NOT_EXIST: 1,
  RESULT_INVALID_MAP_DATA: 2,
  RESULT_INVALID_MAP_METADATA: 3,
  RESULT_UNDEFINED_FAILURE: 255,
}

module.exports = {
  Request: LoadMapRequest,
  Response: LoadMapResponse,
  md5sum() { return '22e647fdfbe3b23c8c9f419908afaebd'; },
  datatype() { return 'nav_msgs/LoadMap'; }
};
