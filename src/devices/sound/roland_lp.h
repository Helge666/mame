// license:BSD-3-Clause
// copyright-holders:Valley Bell
#ifndef MAME_SOUND_ROLAND_LP_H
#define MAME_SOUND_ROLAND_LP_H

#pragma once

#include "dirom.h"

class mb87419_mb87420_device : public device_t, public device_sound_interface, public device_rom_interface<22>
{
public:
	mb87419_mb87420_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	auto int_callback() { return m_int_callback.bind(); }

	// Sample format.  The CM-32P driver keeps the historical differential reading; the
	// U-110's wave data is 8-bit float (sign + 3-bit exponent + 4-bit mantissa) holding a
	// DELTA, which the chip integrates -- see roland_u110.cpp and analysis/ROM-ANALYSIS.md.
	enum pcm_mode : u8 { PCM_DIFFERENTIAL = 0, PCM_LINEAR8 = 1, PCM_FLOAT8 = 2,
	                     PCM_FLOAT8_DELTA = 3 };
	void set_pcm_mode(pcm_mode mode) { m_pcm_mode = mode; }

	// PCM_FLOAT8_DELTA only: corner of the leak on the accumulator, in Hz.  A pure
	// integrator (0 Hz) has infinite DC gain, so this exists as a safety valve -- but it
	// should stay at 0.  Over one loop traversal the deltas sum to EXACTLY zero for all
	// 226 wave ROM samples (30 ping-pong, 26 one-shot, 170 forward), so a correct player
	// cannot drift and has nothing to leak away.  Every nonzero value swept here measured
	// worse.  If DC does walk, the bug is in the path walked, not the lack of a leak.
	void set_delta_dc_hz(double hz) { m_delta_dc_hz = hz; }

	// Output routing.  The CM-32P keeps the historical 2-channel behaviour (every voice to
	// both); the U-110 asks for 6, one per Multi Output, and supplies a per-voice output
	// mask -- bit k enables Multi Output k+1, exactly as the BU3905's slot registers work.
	void set_output_count(int count) { m_out_count = count; }
	void set_voice_mask(u8 voice, u8 mask) { if (voice < NUM_CHANNELS) m_voice_mask[voice] = mask; }

	// Amplitude envelope.  The chip generates one; the CPU writes a voice's volume once
	// at note-on and never touches it again, yet hardware keeps fading -- measured in
	// listen/2/ENVELOPE.md.  Rates are dB/s, both exponential (a straight line in dB fits
	// the measured release at R2 0.92-1.00, better than a straight line in amplitude).
	// The U-110's sysex exposes exactly two envelope parameters per part, ENV ATTACK RATE
	// and ENV RELEASE RATE, each -7..+7 around a default -- so attack and release are the
	// only rates the instrument itself lets you set.
	void set_env_release_db_per_s(double db) { m_env_release = db; }
	void set_env_decay_db_per_s(double db) { m_env_decay = db; }

	// Output rate divider.  The CM-32P's 32.768 MHz / 1024 gives 32 kHz; the U-110 reaches
	// the same 32 kHz from 34.816 MHz / 1088, measured against real hardware.
	void set_rate_divider(u32 div) { m_rate_div = div; }

	u8 read(offs_t offset);
	void write(offs_t offset, u8 data);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// device_sound_interface implementation
	virtual void sound_stream_update(sound_stream &stream) override;

	// device_rom_interface implementation
	virtual void rom_bank_pre_change() override;

	static int16_t decode_sample(int8_t data);
	static int16_t sample_interpolate(int16_t smp1, int16_t smp2, uint16_t frac);

private:
	static constexpr unsigned NUM_CHANNELS = 32;

	struct pcm_channel
	{
		pcm_channel() { }

		// registers
		uint16_t mode = 0;
		uint16_t bank = 0;
		uint16_t step = 0;      // 2.14 fixed point (0x4000 equals 32000 Hz)
		uint16_t volume = 0;
		uint32_t start = 0;     // start address (18.14 fixed point)
		uint16_t end = 0;       // end offset (high word)
		uint16_t loop = 0;      // loop offset (high word)

		// work variables
		bool enable = false;
		int8_t play_dir = 0;    // playing direction, -1 [backwards] / 0 [stopped] / +1 [forwards]
		uint32_t addr = 0;      // current address
		int16_t smpl_cur = 0;   // current sample
		int16_t smpl_nxt = 0;   // next sample
		double  acc = 0.0;      // PCM_FLOAT8_DELTA: the running sum of the deltas
		double  acc_prev = 0.0; // ...and its value one fetch earlier
		// Ping-pong is reflected about the accumulator's value AT THE TURN, not about
		// zero: out = pp_sign * acc + pp_off.  The reasoning is at the fold site, next to
		// the address advance in sound_stream_update -- NOT in the case 2 handler below,
		// which is dead code: a ping-pong turn has to be handled before the read.
		int     pp_sign = 1;
		double  pp_off = 0.0;

		// envelope
		double  env = 1.0;      // linear gain applied on top of the volume registers
		bool    releasing = false;
	};

	devcb_write_line m_int_callback;

	uint32_t m_clock;                   // clock
	uint32_t m_rate;                    // sample rate (usually 32000 Hz)
	sound_stream* m_stream;             // stream handle
	pcm_channel m_chns[NUM_CHANNELS];   // channel memory
	uint8_t m_sel_chn;
	pcm_mode m_pcm_mode = PCM_DIFFERENTIAL;
	int m_out_count = 2;
	u8 m_voice_mask[NUM_CHANNELS];
	u32 m_rate_div = 1024;
	double m_delta_dc_hz = 0.0;         // leak corner for PCM_FLOAT8_DELTA
	double m_delta_keep = 1.0;          // per-sample retention derived from it
	double m_env_release = 0.0;         // dB/s; 0 disables the envelope entirely
	double m_env_decay = 0.0;           // dB/s while the key is held
	double m_env_release_step = 1.0;    // per-sample multipliers, derived from the above
	double m_env_decay_step = 1.0;
};

DECLARE_DEVICE_TYPE(MB87419_MB87420, mb87419_mb87420_device)

#endif // MAME_SOUND_ROLAND_LP_H
