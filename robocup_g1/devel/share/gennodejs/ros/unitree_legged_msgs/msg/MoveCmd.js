// Auto-generated. Do not edit!

// (in-package unitree_legged_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MoveCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vx = null;
      this.vy = null;
      this.vyaw = null;
    }
    else {
      if (initObj.hasOwnProperty('vx')) {
        this.vx = initObj.vx
      }
      else {
        this.vx = 0;
      }
      if (initObj.hasOwnProperty('vy')) {
        this.vy = initObj.vy
      }
      else {
        this.vy = 0;
      }
      if (initObj.hasOwnProperty('vyaw')) {
        this.vyaw = initObj.vyaw
      }
      else {
        this.vyaw = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveCmd
    // Serialize message field [vx]
    bufferOffset = _serializer.int8(obj.vx, buffer, bufferOffset);
    // Serialize message field [vy]
    bufferOffset = _serializer.int8(obj.vy, buffer, bufferOffset);
    // Serialize message field [vyaw]
    bufferOffset = _serializer.int8(obj.vyaw, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveCmd
    let len;
    let data = new MoveCmd(null);
    // Deserialize message field [vx]
    data.vx = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [vy]
    data.vy = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [vyaw]
    data.vyaw = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 3;
  }

  static datatype() {
    // Returns string type for a message object
    return 'unitree_legged_msgs/MoveCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '18fcabcb0c44aa42a27c53f36efde302';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 vx
    int8 vy
    int8 vyaw
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveCmd(null);
    if (msg.vx !== undefined) {
      resolved.vx = msg.vx;
    }
    else {
      resolved.vx = 0
    }

    if (msg.vy !== undefined) {
      resolved.vy = msg.vy;
    }
    else {
      resolved.vy = 0
    }

    if (msg.vyaw !== undefined) {
      resolved.vyaw = msg.vyaw;
    }
    else {
      resolved.vyaw = 0
    }

    return resolved;
    }
};

module.exports = MoveCmd;
