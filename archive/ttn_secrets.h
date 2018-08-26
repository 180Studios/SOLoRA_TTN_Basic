/*
 * ttn_secrets.h - Store your keys here!
 *
 */

// Device EUI
// The RFM95 does not have a unique id. To ensure uniqueness
// generate the Device EUI in the TTN console.
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. 
#define SECRET_DEV_EUI { 0x3E, 0xA7, 0xBD, 0x7F, 0xEE, 0x11, 0xCF, 0x00 }


// Application EUI
// This should also be in little endian format, see above.
// For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70....
#define SECRET_APP_EUI { 0xA9, 0x09, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }


// App key
// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied AS-IS!!!.
#define SECRET_APP_KEY { 0x9F, 0x13, 0xE7, 0xB2, 0x8A, 0xCB, 0x63, 0xC8, 0xC1, 0x89, 0x6D, 0x77, 0x2D, 0x12, 0x84, 0x8F }

