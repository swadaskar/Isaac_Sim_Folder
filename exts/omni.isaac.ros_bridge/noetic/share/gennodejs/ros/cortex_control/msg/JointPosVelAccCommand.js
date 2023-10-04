// Auto-generated. Do not edit!

// (in-package cortex_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class JointPosVelAccCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.id = null;
      this.period = null;
      this.t = null;
      this.names = null;
      this.q = null;
      this.qd = null;
      this.qdd = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('period')) {
        this.period = initObj.period
      }
      else {
        this.period = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('t')) {
        this.t = initObj.t
      }
      else {
        this.t = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('names')) {
        this.names = initObj.names
      }
      else {
        this.names = [];
      }
      if (initObj.hasOwnProperty('q')) {
        this.q = initObj.q
      }
      else {
        this.q = [];
      }
      if (initObj.hasOwnProperty('qd')) {
        this.qd = initObj.qd
      }
      else {
        this.qd = [];
      }
      if (initObj.hasOwnProperty('qdd')) {
        this.qdd = initObj.qdd
      }
      else {
        this.qdd = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointPosVelAccCommand
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.int64(obj.id, buffer, bufferOffset);
    // Serialize message field [period]
    bufferOffset = _serializer.duration(obj.period, buffer, bufferOffset);
    // Serialize message field [t]
    bufferOffset = _serializer.time(obj.t, buffer, bufferOffset);
    // Serialize message field [names]
    bufferOffset = _arraySerializer.string(obj.names, buffer, bufferOffset, null);
    // Serialize message field [q]
    bufferOffset = _arraySerializer.float64(obj.q, buffer, bufferOffset, null);
    // Serialize message field [qd]
    bufferOffset = _arraySerializer.float64(obj.qd, buffer, bufferOffset, null);
    // Serialize message field [qdd]
    bufferOffset = _arraySerializer.float64(obj.qdd, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointPosVelAccCommand
    let len;
    let data = new JointPosVelAccCommand(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [period]
    data.period = _deserializer.duration(buffer, bufferOffset);
    // Deserialize message field [t]
    data.t = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [names]
    data.names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [q]
    data.q = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [qd]
    data.qd = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [qdd]
    data.qdd = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.names.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += 8 * object.q.length;
    length += 8 * object.qd.length;
    length += 8 * object.qdd.length;
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cortex_control/JointPosVelAccCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '150bb2ee3b92d2156bc3b45a48477ca0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Basic joint position, velocity, and feed-forward acceleration command.
    # Typically, q and qd are used for PID control, and qdd is used by inverse
    # dynamics to compute a feed forward term.
    #
    # These message contain enough information to reconstruct the specific integral
    # curve on the receiving side using quintic polynomial interpolation
    # (rectifying the messages against jitter). See CommandStreamInterpolator.
    
    # Contains the wall-clock time stamp (unless otherwise specified explicitly
    # during construction of the Cortex commander).
    std_msgs/Header header
    
    # id's increment by one each, and period gives the amount of time between the
    # the previous message (with message ID id-1) and this
    int64 id
    duration period
    
    # This time stamp is the exact controller time in the sense that
    #
    #   msg[id+1].t - msg[id].t = msg[id+1].period
    #
    # The header gives the wall-clock time at publication (unless otherwise
    # specified during initialization of the Cortex commander) so we can observe any
    # jitter in plotters such as rqt_plot which read the header time stamps to
    # rectify the incoming messages.
    time t
    
    string[] names
    float64[] q
    float64[] qd
    float64[] qdd
    
    #SymmetricMatrix32 metric
    
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
    const resolved = new JointPosVelAccCommand(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.period !== undefined) {
      resolved.period = msg.period;
    }
    else {
      resolved.period = {secs: 0, nsecs: 0}
    }

    if (msg.t !== undefined) {
      resolved.t = msg.t;
    }
    else {
      resolved.t = {secs: 0, nsecs: 0}
    }

    if (msg.names !== undefined) {
      resolved.names = msg.names;
    }
    else {
      resolved.names = []
    }

    if (msg.q !== undefined) {
      resolved.q = msg.q;
    }
    else {
      resolved.q = []
    }

    if (msg.qd !== undefined) {
      resolved.qd = msg.qd;
    }
    else {
      resolved.qd = []
    }

    if (msg.qdd !== undefined) {
      resolved.qdd = msg.qdd;
    }
    else {
      resolved.qdd = []
    }

    return resolved;
    }
};

module.exports = JointPosVelAccCommand;
