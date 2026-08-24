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

	// Sample format.  The CM-32P driver keeps the historical differential reading;
	// the U-110's wave data is plain two's complement 8-bit PCM (see roland_u110.cpp).
	enum pcm_mode : u8 { PCM_DIFFERENTIAL = 0, PCM_LINEAR8 = 1, PCM_FLOAT8 = 2 };
	void set_pcm_mode(pcm_mode mode) { m_pcm_mode = mode; }

	// Output routing.  The CM-32P keeps the historical 2-channel behaviour (every voice to
	// both); the U-110 asks for 6, one per Multi Output, and supplies a per-voice output
	// mask -- bit k enables Multi Output k+1, exactly as the BU3905's slot registers work.
	void set_output_count(int count) { m_out_count = count; }
	void set_voice_mask(u8 voice, u8 mask) { if (voice < NUM_CHANNELS) m_voice_mask[voice] = mask; }

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
	u32 m_rate_div = 1024;                  // selected channel
};

DECLARE_DEVICE_TYPE(MB87419_MB87420, mb87419_mb87420_device)

#endif // MAME_SOUND_ROLAND_LP_H
