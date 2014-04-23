var mavlink = require('../implementations/mavlink_ardupilotmega_v1.0'),
  should = require('should');

describe('MAVLink message registry', function() {

  it('defines constructors for every message', function() {
    mavlink.messages['battery_status'].should.be.a.function;
  });

  it('assigns message properties to each message', function() {
    var m = new mavlink.messages['battery_status']();
    m.format.should.equal("<HHHHHHhBb");
    m.order_map.should.eql([7, 0, 1, 2, 3, 4, 5, 6, 8]); // should.eql = shallow comparison
    m.crc_extra.should.equal(42);
    m.id.should.equal(mavlink.MAVLINK_MSG_ID_BATTERY_STATUS);
  });

});

describe('Complete MAVLink packet', function() {

  it('encodes to match a reference packet generated through the Python version', function() {

    var heartbeat = new mavlink.messages.heartbeat(
      mavlink.MAV_TYPE_GCS, // 6
      mavlink.MAV_AUTOPILOT_INVALID, // 8
      0, // base mode, mavlink.MAV_MODE_FLAG_***
      0, // custom mode
      0, // system status
      3 // MAVLink version
    );
    
    // Set header properties
    _.extend(heartbeat, {
      seq: 2,
      srcSystem: 255,
      srcComponent: 0
    });

    // Create a buffer that matches what the Python version of MAVLink creates
    var reference = new Buffer([0xfe, 0x09, 0x02, 0xff , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x06 , 0x08 , 0x00 , 0x00 , 0x03 , 0x75 , 0x22]);
    new Buffer(heartbeat.pack()).should.eql(reference);

  });

});

describe('MAVLink header', function() {

  beforeEach(function() {
    this.h = new mavlink.header(mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 1, 2, 3, 4);
  })

  it('Can pack itself', function() {
    this.h.pack().should.eql([254, 1, 2, 3, 4, 147]);
  });

});

describe('MAVLink message', function() {

  beforeEach(function() {

    // This is a heartbeat packet from a GCS to the APM.
    this.heartbeat = new mavlink.messages.heartbeat(
      mavlink.MAV_TYPE_GCS, // 6
      mavlink.MAV_AUTOPILOT_INVALID, // 8
      0, // base mode, mavlink.MAV_MODE_FLAG_***
      0, // custom mode
      mavlink.MAV_STATE_STANDBY, // system status
      3 // MAVLink version
    );

  });

  it('has a set function to facilitate vivifying the object', function() {
    this.heartbeat.type.should.equal(mavlink.MAV_TYPE_GCS);
    this.heartbeat.autopilot.should.equal(mavlink.MAV_AUTOPILOT_INVALID);
    this.heartbeat.base_mode.should.equal(0);
    this.heartbeat.custom_mode.should.equal(0);
    this.heartbeat.system_status.should.equal(mavlink.MAV_STATE_STANDBY);
  });

  // TODO: the length below (9) should perhaps be instead 7.  See mavlink.unpack().
  // might have to do with the length of the encoding (<I is 4 symbols in the array) 
  it('Can pack itself', function() {
    
    var packed = this.heartbeat.pack();
    packed.should.eql([254, 9, 0, 0, 0, mavlink.MAVLINK_MSG_ID_HEARTBEAT, // that bit is the header,
      // this is the payload, arranged in the order map specified in the protocol,
      // which differs from the constructor.
      0, 0, 0, 0, // custom bitfield -- length 4 (type=I)
      mavlink.MAV_TYPE_GCS,
      mavlink.MAV_AUTOPILOT_INVALID,
      0,
      mavlink.MAV_STATE_STANDBY,
      3,
      109, // CRC
      79 // CRC
      ]);

  });

  describe('decode function', function() {

    beforeEach(function() {
      this.m = new MAVLink();
    });

    // need to add tests for the header fields as well, specifying seq etc.
    it('Can decode itself', function() {

      var packed = this.heartbeat.pack();
      var message = this.m.decode(packed);

      // this.fieldnames = ['type', 'autopilot', 'base_mode', 'custom_mode', 'system_status', 'mavlink_version'];
      message.type.should.equal(mavlink.MAV_TYPE_GCS);  // supposed to be 6
      message.autopilot.should.equal(mavlink.MAV_AUTOPILOT_INVALID); // supposed to be 8
      message.base_mode.should.equal(0); // supposed to be 0
      message.custom_mode.should.equal(0);
      message.system_status.should.equal(mavlink.MAV_STATE_STANDBY); // supposed to be 3
      message.mavlink_version.should.equal(3); //?

    });

    it('throws an error if the message has a bad prefix', function() {
      var packed = [0, 3, 5, 7, 9, 11]; // bad data prefix in header (0, not 254)
      var m = this.m;
      (function() { m.decode(packed); }).should.throw('Invalid MAVLink prefix (0)');
    });

    it('throws an error if the message ID is not known', function() {
      var packed = [254, 1, 0, 3, 0, 200, 1, 0, 0]; // 200 = invalid ID
      var m = this.m;
      (function() { m.decode(packed); }).should.throw('Unknown MAVLink message ID (200)');
    });

    it('throws an error if the message length is invalid', function() {
      var packed = [254, 3, 257, 0, 0, 0, 0, 0];
      var m = this.m;
      (function() { m.decode(packed); }).should.throw('Invalid MAVLink message length.  Got 0 expected 3, msgId=0');
    });

    it('throws an error if the CRC cannot be unpacked', function() {
      
    });

    it('throws an error if the CRC can not be decoded', function() {

    });

    it('throws an error if it is unable to unpack the payload', function() {

    });

    it('throws an error if it is unable to instantiate a MAVLink message object from the payload', function() {

    });

  });
});