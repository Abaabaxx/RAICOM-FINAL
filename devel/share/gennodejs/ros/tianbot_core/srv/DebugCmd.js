// Auto-generated. Do not edit!

// (in-package tianbot_core.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class DebugCmdRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd = null;
    }
    else {
      if (initObj.hasOwnProperty('cmd')) {
        this.cmd = initObj.cmd
      }
      else {
        this.cmd = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DebugCmdRequest
    // Serialize message field [cmd]
    bufferOffset = _serializer.string(obj.cmd, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DebugCmdRequest
    let len;
    let data = new DebugCmdRequest(null);
    // Deserialize message field [cmd]
    data.cmd = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.cmd.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tianbot_core/DebugCmdRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '43a54fa49066cddcf148717d9d4a6353';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string cmd
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DebugCmdRequest(null);
    if (msg.cmd !== undefined) {
      resolved.cmd = msg.cmd;
    }
    else {
      resolved.cmd = ''
    }

    return resolved;
    }
};

class DebugCmdResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DebugCmdResponse
    // Serialize message field [result]
    bufferOffset = _serializer.string(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DebugCmdResponse
    let len;
    let data = new DebugCmdResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.result.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tianbot_core/DebugCmdResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c22f2a1ed8654a0b365f1bb3f7ff2c0f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string result
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DebugCmdResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: DebugCmdRequest,
  Response: DebugCmdResponse,
  md5sum() { return 'a34869486bf541f5f26e7eea27f4ece8'; },
  datatype() { return 'tianbot_core/DebugCmd'; }
};
