// Auto-generated. Do not edit!

// (in-package franka_example_controllers.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class State {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position = null;
      this.position_d = null;
      this.orientation = null;
      this.orientation_d = null;
      this.tau_commanded = null;
      this.error_orientation = null;
      this.error_position = null;
      this.tau__ext = null;
    }
    else {
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('position_d')) {
        this.position_d = initObj.position_d
      }
      else {
        this.position_d = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('orientation')) {
        this.orientation = initObj.orientation
      }
      else {
        this.orientation = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('orientation_d')) {
        this.orientation_d = initObj.orientation_d
      }
      else {
        this.orientation_d = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('tau_commanded')) {
        this.tau_commanded = initObj.tau_commanded
      }
      else {
        this.tau_commanded = new Array(7).fill(0);
      }
      if (initObj.hasOwnProperty('error_orientation')) {
        this.error_orientation = initObj.error_orientation
      }
      else {
        this.error_orientation = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('error_position')) {
        this.error_position = initObj.error_position
      }
      else {
        this.error_position = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('tau__ext')) {
        this.tau__ext = initObj.tau__ext
      }
      else {
        this.tau__ext = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type State
    // Check that the constant length array field [position] has the right length
    if (obj.position.length !== 3) {
      throw new Error('Unable to serialize array field position - length must be 3')
    }
    // Serialize message field [position]
    bufferOffset = _arraySerializer.float64(obj.position, buffer, bufferOffset, 3);
    // Check that the constant length array field [position_d] has the right length
    if (obj.position_d.length !== 3) {
      throw new Error('Unable to serialize array field position_d - length must be 3')
    }
    // Serialize message field [position_d]
    bufferOffset = _arraySerializer.float64(obj.position_d, buffer, bufferOffset, 3);
    // Check that the constant length array field [orientation] has the right length
    if (obj.orientation.length !== 3) {
      throw new Error('Unable to serialize array field orientation - length must be 3')
    }
    // Serialize message field [orientation]
    bufferOffset = _arraySerializer.float64(obj.orientation, buffer, bufferOffset, 3);
    // Check that the constant length array field [orientation_d] has the right length
    if (obj.orientation_d.length !== 3) {
      throw new Error('Unable to serialize array field orientation_d - length must be 3')
    }
    // Serialize message field [orientation_d]
    bufferOffset = _arraySerializer.float64(obj.orientation_d, buffer, bufferOffset, 3);
    // Check that the constant length array field [tau_commanded] has the right length
    if (obj.tau_commanded.length !== 7) {
      throw new Error('Unable to serialize array field tau_commanded - length must be 7')
    }
    // Serialize message field [tau_commanded]
    bufferOffset = _arraySerializer.float64(obj.tau_commanded, buffer, bufferOffset, 7);
    // Check that the constant length array field [error_orientation] has the right length
    if (obj.error_orientation.length !== 3) {
      throw new Error('Unable to serialize array field error_orientation - length must be 3')
    }
    // Serialize message field [error_orientation]
    bufferOffset = _arraySerializer.float64(obj.error_orientation, buffer, bufferOffset, 3);
    // Check that the constant length array field [error_position] has the right length
    if (obj.error_position.length !== 3) {
      throw new Error('Unable to serialize array field error_position - length must be 3')
    }
    // Serialize message field [error_position]
    bufferOffset = _arraySerializer.float64(obj.error_position, buffer, bufferOffset, 3);
    // Check that the constant length array field [tau__ext] has the right length
    if (obj.tau__ext.length !== 6) {
      throw new Error('Unable to serialize array field tau__ext - length must be 6')
    }
    // Serialize message field [tau__ext]
    bufferOffset = _arraySerializer.float64(obj.tau__ext, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type State
    let len;
    let data = new State(null);
    // Deserialize message field [position]
    data.position = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [position_d]
    data.position_d = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [orientation]
    data.orientation = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [orientation_d]
    data.orientation_d = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [tau_commanded]
    data.tau_commanded = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    // Deserialize message field [error_orientation]
    data.error_orientation = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [error_position]
    data.error_position = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [tau__ext]
    data.tau__ext = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    return 248;
  }

  static datatype() {
    // Returns string type for a message object
    return 'franka_example_controllers/State';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2557e31f2a34e246ce8c881d4f7d2cb2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[3] position
    float64[3] position_d
    float64[3] orientation
    float64[3] orientation_d
    float64[7] tau_commanded
    float64[3] error_orientation
    float64[3] error_position
    float64[6] tau__ext
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new State(null);
    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = new Array(3).fill(0)
    }

    if (msg.position_d !== undefined) {
      resolved.position_d = msg.position_d;
    }
    else {
      resolved.position_d = new Array(3).fill(0)
    }

    if (msg.orientation !== undefined) {
      resolved.orientation = msg.orientation;
    }
    else {
      resolved.orientation = new Array(3).fill(0)
    }

    if (msg.orientation_d !== undefined) {
      resolved.orientation_d = msg.orientation_d;
    }
    else {
      resolved.orientation_d = new Array(3).fill(0)
    }

    if (msg.tau_commanded !== undefined) {
      resolved.tau_commanded = msg.tau_commanded;
    }
    else {
      resolved.tau_commanded = new Array(7).fill(0)
    }

    if (msg.error_orientation !== undefined) {
      resolved.error_orientation = msg.error_orientation;
    }
    else {
      resolved.error_orientation = new Array(3).fill(0)
    }

    if (msg.error_position !== undefined) {
      resolved.error_position = msg.error_position;
    }
    else {
      resolved.error_position = new Array(3).fill(0)
    }

    if (msg.tau__ext !== undefined) {
      resolved.tau__ext = msg.tau__ext;
    }
    else {
      resolved.tau__ext = new Array(6).fill(0)
    }

    return resolved;
    }
};

module.exports = State;
