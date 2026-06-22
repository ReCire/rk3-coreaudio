/*
 * caiaq_protocol.h — RigKontrol 3 EP1 command protocol.
 *
 * Ported from the Linux snd-usb-caiaq driver (device.h / device.c):
 * the vendor command set carried over bulk EP1, the device-info handshake,
 * and the audio-parameter negotiation. Transport-agnostic except that it
 * uses struct rk3_usb's bulk helpers.
 */

#ifndef CAIAQ_PROTOCOL_H
#define CAIAQ_PROTOCOL_H

#include "usb_iokit.h"
#include <stdint.h>

/* EP1 command bytes (device.h:31). */
#define EP1_CMD_GET_DEVICE_INFO 0x1
#define EP1_CMD_READ_ERP        0x2
#define EP1_CMD_READ_ANALOG     0x3
#define EP1_CMD_READ_IO         0x4
#define EP1_CMD_WRITE_IO        0x5
#define EP1_CMD_MIDI_READ       0x6
#define EP1_CMD_MIDI_WRITE      0x7
#define EP1_CMD_AUDIO_PARAMS    0x9
#define EP1_CMD_AUTO_MSG        0xb
#define EP1_CMD_DIMM_LEDS       0xc
#define EP1_CMD_DIRECT_MON      0xe   /* setDirectMonitoringMode (NI kext) */
#define EP1_CMD_GET_DIRECT_MON  0x12  /* getCurrentDirectMonitoringMode */

#define EP1_BUFSIZE 64

/* Sample-rate codes for EP1_CMD_AUDIO_PARAMS (device.c:68). */
enum {
	CAIAQ_SR_44100 = 0,
	CAIAQ_SR_48000 = 1,
	CAIAQ_SR_96000 = 2,
	CAIAQ_SR_192000 = 3,
	CAIAQ_SR_88200 = 4,
	CAIAQ_SR_INVALID = 0xff,
};

/* Bit depth codes (device.c:79). */
enum {
	CAIAQ_DEPTH_NONE = 0,
	CAIAQ_DEPTH_16 = 1,
	CAIAQ_DEPTH_24 = 2,
	CAIAQ_DEPTH_32 = 3,
};

/* Device-info reply payload (device.h:42). fw_version is little-endian. */
struct caiaq_device_spec {
	uint16_t fw_version;
	uint8_t  hw_subtype;
	uint8_t  num_erp;
	uint8_t  num_analog_in;
	uint8_t  num_digital_in;
	uint8_t  num_digital_out;
	uint8_t  num_analog_audio_out;
	uint8_t  num_analog_audio_in;
	uint8_t  num_digital_audio_out;
	uint8_t  num_digital_audio_in;
	uint8_t  num_midi_out;
	uint8_t  num_midi_in;
	uint8_t  data_alignment;
} __attribute__((packed));

/*
 * Send a command: ep1_out[0]=command, payload follows (device.c:216).
 * Returns 0 on success.
 */
int caiaq_send_command(struct rk3_usb *u, uint8_t command,
		       const uint8_t *buffer, int len);

/*
 * Run the GET_DEVICE_INFO handshake and fill *spec (host byte order).
 * Reads EP1 IN, skipping any unsolicited messages, until the matching
 * reply arrives or it times out. Returns 0 on success.
 */
int caiaq_get_device_info(struct rk3_usb *u, struct caiaq_device_spec *spec);

/*
 * Negotiate audio params (device.c:263). rate in Hz, depth in bits (16/24),
 * bpp = bytes per isoc packet. Returns 0 if the device acked (answer==1).
 */
int caiaq_set_audio_params(struct rk3_usb *u, int rate, int depth, int bpp);

/* Compute bpp the way the Linux driver does (audio.c:266). */
int caiaq_compute_bpp(int rate, int data_alignment, int n_streams);

/*
 * Hardware direct monitoring (RE'd from NI's NIUSBAudioDriver.kext).
 * setDirectMonitoringMode sends EP1 cmd 0x0e with a single mode byte; the
 * device routes its analog input to its analog output internally, bypassing
 * the host round-trip (near-zero monitoring latency). The mode-byte semantics
 * are not in the Linux driver — sweep them on hardware. Returns 0 on success.
 */
int caiaq_set_direct_mon(struct rk3_usb *u, uint8_t mode);

/*
 * Query the device's current direct-monitoring mode (EP1 cmd 0x12). Writes the
 * mode byte from the reply into *mode_out. Returns 0 on success.
 */
int caiaq_get_direct_mon(struct rk3_usb *u, uint8_t *mode_out);

#endif /* CAIAQ_PROTOCOL_H */
