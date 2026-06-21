/*
 * handshake.c — Step 2: open the RK3, run GET_DEVICE_INFO, print the
 * device spec (crucially data_alignment), then negotiate AUDIO_PARAMS
 * for 48 kHz / 24-bit and confirm the device acks.
 *
 * Build: make handshake   Run: ./build/handshake  (RK3 plugged in)
 */

#include "../usb_iokit.h"
#include "../caiaq_protocol.h"

#include <stdio.h>

int main(void)
{
	struct rk3_usb u;
	struct caiaq_device_spec spec;
	int rc;

	printf("RigKontrol 3 handshake\n");

	if (rk3_usb_open(&u) != 0)
		return 1;

	printf("opened: cmd_out=pipe%u cmd_in=pipe%u "
	       "iso_in=pipe%u (maxpkt %u) iso_out=pipe%u (maxpkt %u)\n",
	       u.pipe_cmd_out, u.pipe_cmd_in,
	       u.pipe_iso_in, u.iso_in_maxpacket,
	       u.pipe_iso_out, u.iso_out_maxpacket);

	rc = caiaq_get_device_info(&u, &spec);
	if (rc != 0) {
		rk3_usb_close(&u);
		return 1;
	}

	printf("\n=== caiaq_device_spec ===\n");
	printf("  fw_version            : %u\n", spec.fw_version);
	printf("  hw_subtype            : %u\n", spec.hw_subtype);
	printf("  num_erp               : %u\n", spec.num_erp);
	printf("  num_analog_in         : %u\n", spec.num_analog_in);
	printf("  num_digital_in/out    : %u / %u\n",
	       spec.num_digital_in, spec.num_digital_out);
	printf("  num_analog_audio_out  : %u\n", spec.num_analog_audio_out);
	printf("  num_analog_audio_in   : %u\n", spec.num_analog_audio_in);
	printf("  num_digital_audio_out : %u\n", spec.num_digital_audio_out);
	printf("  num_digital_audio_in  : %u\n", spec.num_digital_audio_in);
	printf("  num_midi_out/in       : %u / %u\n",
	       spec.num_midi_out, spec.num_midi_in);
	printf("  *** data_alignment    : %u ***  (selects packet codec)\n",
	       spec.data_alignment);

	/* RK3 = 2 in / 2 out => 1 stereo stream. */
	int n_audio_in  = spec.num_analog_audio_in / 2;
	int n_audio_out = spec.num_analog_audio_out / 2;
	int n_streams = n_audio_in > n_audio_out ? n_audio_in : n_audio_out;
	if (n_streams < 1)
		n_streams = 1;

	int rate = 48000, depth = 24;
	int bpp = caiaq_compute_bpp(rate, spec.data_alignment, n_streams);
	printf("\nn_streams=%d, computed bpp=%d for %d Hz / %d-bit\n",
	       n_streams, bpp, rate, depth);

	rc = caiaq_set_audio_params(&u, rate, depth, bpp);
	if (rc == 0)
		printf("AUDIO_PARAMS accepted by device (ack==1). Step 2 OK.\n");
	else
		printf("AUDIO_PARAMS NOT accepted.\n");

	rk3_usb_close(&u);
	return rc == 0 ? 0 : 1;
}
