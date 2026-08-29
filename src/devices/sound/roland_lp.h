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

	// The envelope ramp generator, and the interrupt that drives it.  OFF by default, so
	// the CM-32P / D-70 / R-8 / U-20 drivers keep the behaviour they were tuned against.
	//
	// Registers 06/07 are one envelope SEGMENT, not a volume: 07 is the TARGET level (log,
	// 16 units per octave) and 06 is a SIGNED ramp RATE.  The chip walks a linear level
	// toward the target and raises INT when it arrives; the CPU's handler then writes the
	// next segment.  Measured on hardware (listen/env/ANALYSIS.md, listen/env2):
	//
	//   * going UP the ramp is linear in AMPLITUDE -- 0.14 dB rms over 29 dB, against
	//     2.36 dB rms for a straight line in dB;
	//   * the rate byte is a pure exponential, 8.01 counts per doubling (2.4% residual
	//     over 41 trials at rate bytes 2..40).  A 3-bit-mantissa float gives 15.4% and is
	//     ruled out;
	//   * one increment of 2^(rate/8) * 64 per output sample on a 26-bit level reproduces
	//     the measured attack times at rate bytes 8, 24 and 40 (2.456 / 0.616 / 0.153 s at
	//     target 211) to better than 1%.
	//
	// Falling ramps measure EXPONENTIAL, which is not what a linear ramp generator does --
	// but it does not have to be the chip: the firmware's handler at 0x42B3 adds
	// (3986[voice] * reg07) >> 9 to the rate with reg07 refreshed from this level readback,
	// i.e. a rate proportional to the current level.  That IS an exponential, assembled one
	// interrupt at a time.  If this model is right, the decay falls out on its own.
	void set_env_engine(bool on) { m_env_engine = on; }

	// The interrupt is a free-running ROUND-ROBIN scanner, not a signal that a ramp
	// finished: the handler reads register 00 to learn WHICH slot it is being offered,
	// checks that slot's own state and usually does nothing.  Reading it as "interrupt on
	// arrival" instead produces an interrupt storm -- the CPU answers each one with a fresh
	// segment, that segment arrives at once, and it interrupts again.
	//
	// The chip raises INT when a voice's ramp ARRIVES at its target, naming that voice in
	// register 00; the handler's phase machine then advances one step and programs the next
	// segment.  The advance at 0xBAD4 is an unconditional `incb`, so one interrupt IS one
	// envelope step -- which is why a free-running round-robin over all 34 slots cannot be
	// right: to give a piano its attack it would have to run at ~11000 a second, and the
	// handler costs far too much for that.  Arrival-driven, a note needs about four.
	//
	// This is paced rather than instantaneous: the line is offered on a timer, so several
	// voices arriving together queue instead of racing, and the rate below is a CEILING on
	// how often the CPU can be interrupted, not a fixed cadence.  Idle voices cost nothing.
	//
	// Registers 0x19 / 0x1B / 0x1D are NOT scanner configuration, though the 0x00/0x21/0x64
	// the firmware writes to them at 0x43DA looks like it: the same trio is rewritten at
	// 0xB4C6 from the patch's CHORUS/TREMOLO byte (0x280E) through a table at 0xA726, and
	// the code beside it writes 0x378C / 0x378E -- exactly the RAM the handler's 0x20 / 0x21
	// branches read.  That trio is the chorus/tremolo subsystem and those two slots are its
	// LFOs.  Driving the scanner from 0x1D switched the envelope off whenever the firmware
	// retuned the chorus.
	void set_env_max_int_hz(u32 hz) { m_env_scan_hz = hz ? hz : 1; }


	// The older stand-in for all of the above: a fixed exponential fade the device applies
	// itself, with no interrupt and no level readback.  Superseded by set_env_engine(true)
	// and left for the drivers that have not been measured.
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

	TIMER_CALLBACK_MEMBER(env_tick);

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

		// envelope ramp generator (m_env_engine).  env_level is the chip's own 26-bit
		// LINEAR level -- the quantity the firmware reads back and normalises.
		double  env_level = 0.0;
		double  env_target = 0.0;
		double  env_rate = 0.0;     // signed, per output sample; 0 = arrived, holding
	};

	// The level is 26 bits.  The firmware reads it as two fields (low 16 through select
	// 0x10, high 10 through 0x12), glues them into one 32-bit word, shifts left by 6 and
	// normalises -- so the top of the 26-bit range lands at the top of the 32-bit word,
	// and the log value it derives is exactly register 07's scale.  See the readback at
	// 0x7655 in analysis/ROM-ANALYSIS.md.
	static constexpr double ENV_FULL = 67108864.0;      // 2^26, level at reg 07 = 0xFF
	// The rate byte's increment carries six fractional bits: 2^(rate/8) counts the level's
	// 64ths.  This is the constant the hardware attack times pin down.
	static constexpr double ENV_RATE_SCALE = 64.0;
	// A falling ramp moves 16x slower than a rising one at the same rate byte -- the rise
	// adds 2^(rate/8) << 6 to the level, the fall subtracts 2^(rate/8) << 2.  Measured, not
	// fitted: against the ENV RELEASE sweep on E. Organ 1 the ratio is 15.0-15.7x across
	// five settings spanning a 15x range of rate, constant to +/-2%, and hardware and
	// emulator both double cleanly step for step over that range.
	static constexpr double ENV_FALL_DIVISOR = 16.0;


	double env_level_for(uint8_t log_level) const;
	void env_segment(pcm_channel &chn);
	void env_scan();
	void env_arrived(unsigned voice);

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

	// envelope ramp generator
	bool m_env_engine = false;
	u32 m_env_scan_hz = 8000;           // ceiling on interrupts/s
	u32 m_env_pending = 0;              // one bit per voice whose ramp has arrived
	u32 m_env_slots = 0x22;             // 32 voices plus the two chorus/tremolo slots
	emu_timer *m_env_timer = nullptr;   // ...which is its own timer, not the sound stream:
	                                    // pulsing once per stream block would collapse a
	                                    // whole block's worth of offers into one edge
	uint8_t m_env_service = 0;          // the slot register 0x00 is currently reporting
	bool m_env_int = false;             // state of the INT line
	uint8_t m_status_voice = 0;         // voice selected for the 0x02/0x03 readback
	uint8_t m_status_field = 0;         // 0 = low 16 bits, 1 = high 10 bits, 2 = log level
};

DECLARE_DEVICE_TYPE(MB87419_MB87420, mb87419_mb87420_device)

#endif // MAME_SOUND_ROLAND_LP_H
