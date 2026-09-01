// license:BSD-3-Clause
// copyright-holders:Valley Bell, Elliott H. Liggett

// register write:
//  00/01 - ??
//  02/03 - ROM bank (bits 10-13) / loop mode (bits 14-15)
//  04/05 - frequency (2.14 fixed point, 0x4000 = 32000 Hz)
//  06/07 - volume
//  08/09 - sample start address, fraction (2.14 fixed point, i.e. 1 byte = 0x4000)
//  0A/0B - sample start address (high word, i.e. address bits 2..17)
//  0C/0D - sample end address (high word)
//  0E/0F - sample loop address (high word)
//  10/12 - ?? (sometimes value 1F is written)
//  11/13/15/17 - voice enable mask (11 = least significant 8 bits, 17 = most significant 8 bits)
//  19/1B/1D - ?? (written at init time)
//  1A - ??
//  1F - voice select
//
// register read:
//  01 - last sample data (used by main CPU to read sample table from PCM ROM)
//  02/03 - ?? (read after writing to 10/12)

#include "emu.h"
#include "roland_lp.h"

// Register-level tracing of the envelope handshake, for the decay investigation.  Off
// unless U110_ENVTRACE is set in the environment, so it costs one predictable branch.
static bool env_trace_enabled()
{
	static bool const on = getenv("U110_ENVTRACE") != nullptr;
	return on;
}
#define ENV_TRACE env_trace_enabled()

// Interpolator precision, for the 6-9 kHz brightness investigation.  U110_INTERP=
//   0  truncate the interpolated sample to a 12-bit integer (the historical behaviour)
//   1  round it to nearest instead -- one adder in hardware, and it removes the -0.5 LSB
//      DC bias that flooring leaves
//   2  carry the 14 fractional bits into the envelope multiply and quantise once, later
// Reconstruction kernel, for the image-rejection investigation.  U110_RECON=
//   0  linear interpolation between the two neighbouring samples (sinc^2)
//   1  quadratic B-spline over three (sinc^3)
// A sample played well below its reference note -- A. Piano 1 note 43 runs at step
// 0.2952, i.e. a 9450 Hz source -- repeats its spectrum every 9450 Hz, and the kernel is
// the only thing suppressing those copies.  Linear rejects an image of 1 kHz content by
// 37 dB; one more order rejects it by 55 dB, while barely changing an image of 4 kHz
// content.  That asymmetry is the shape of the measured error.
// A band-limited reconstruction, for U110_RECON=2.  Linear interpolation is wrong in two
// ways at once and they pull opposite ways: sinc^2 droops the PASSBAND by up to 7 dB
// approaching the source Nyquist, and it leaks the IMAGES beyond it.  A tone whose energy
// sits below its own Nyquist therefore comes out too dark (VIB 1, marimba, fingered bass)
// while one whose content images into the band comes out too bright (BELL 1, A. PIANO 1 at
// note 43) -- which is why a smoother kernel helped the second family and hurt the first.
//
// This is the other direction: flat to fs/2, steep past it.  A 16-tap Hann-windowed sinc,
// 256 fractional phases, evaluated with a constant delay of 7 source samples so every tap
// comes from history.  Only for step <= 1 (playing at or below the sample's own rate); a
// step above 1 is decimation and wants a narrower cutoff, so it keeps the linear path.
static constexpr int SINC_TAPS = 16;
static constexpr int SINC_PHASES = 256;
static double const *sinc_table()
{
	static std::vector<double> tbl = [] {
		std::vector<double> t(SINC_PHASES * SINC_TAPS);
		for (int ph = 0; ph < SINC_PHASES; ph ++)
		{
			double const frac = double(ph) / SINC_PHASES;
			double sum = 0.0;
			for (int j = 0; j < SINC_TAPS; j ++)
			{
				double const d = frac - (SINC_TAPS / 2) + j;   // spans [frac-8, frac+7]
				double const x = M_PI * d;
				double const si = (fabs(d) < 1e-9) ? 1.0 : sin(x) / x;
				double const w = 0.5 * (1.0 + cos(M_PI * d / (SINC_TAPS / 2)));
				t[ph * SINC_TAPS + j] = si * w;
				sum += si * w;
			}
			for (int j = 0; j < SINC_TAPS; j ++)    // unity DC gain
				t[ph * SINC_TAPS + j] /= sum;
		}
		return t;
	}();
	return tbl.data();
}

static int recon_mode()
{
	static int const mode = getenv("U110_RECON") ? atoi(getenv("U110_RECON")) : 0;
	return mode;
}

static int interp_mode()
{
	static int const mode = getenv("U110_INTERP") ? atoi(getenv("U110_INTERP")) : 0;
	return mode;
}

#include <cmath>
#include <vector>
#include <algorithm>


DEFINE_DEVICE_TYPE(MB87419_MB87420, mb87419_mb87420_device, "mb87419_mb87420", "Roland LP MB87419/MB87420 PCM")

mb87419_mb87420_device::mb87419_mb87420_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, MB87419_MB87420, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, device_rom_interface(mconfig, *this)
	, m_int_callback(*this)
	, m_clock(0)
	, m_rate(0)
	, m_stream(nullptr)
	, m_sel_chn(0)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void mb87419_mb87420_device::device_start()
{
	m_clock = clock() / 2;
	m_rate = clock() / m_rate_div;  // 32 kHz on both the CM-32P and the U-110

	std::fill(std::begin(m_voice_mask), std::end(m_voice_mask), 0x03);

	// dB/s -> per-sample linear multiplier.  Exponential decay is one multiply per sample
	// and is what the measurement says the chip does; see roland_lp.h.
	m_delta_keep = m_delta_dc_hz > 0.0
			? exp(-2.0 * M_PI * m_delta_dc_hz / m_rate) : 1.0;

	m_env_release_step = m_env_release > 0.0 ? pow(10.0, -m_env_release / 20.0 / m_rate) : 1.0;
	m_env_decay_step = m_env_decay > 0.0 ? pow(10.0, -m_env_decay / 20.0 / m_rate) : 1.0;

	m_stream = stream_alloc(0, m_out_count, m_rate);

	m_env_timer = timer_alloc(FUNC(mb87419_mb87420_device::env_tick), this);
	if (m_env_engine)
	{
		// Free-running from reset.  The handler cannot run before the firmware unmasks
		// EXTINT at 0x4530 anyway, and by then it has cleared the per-voice state at
		// 0x445D -- so an offer made earlier simply waits as a pending interrupt.
		attotime const period = attotime::from_hz(2 * m_env_scan_hz);
		m_env_timer->adjust(period, 0, period);
		logerror("Envelope interrupts: arrival driven, at most %u/s\n", m_env_scan_hz);
	}

	logerror("Roland PCM: Clock %u, Rate %u\n", m_clock, m_rate);
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void mb87419_mb87420_device::device_reset()
{
	m_env_int = false;
	m_env_service = 0;
	m_env_pending = 0;
	m_env_offered = false;
	m_env_offer_age = 0;
	m_status_voice = 0;
	m_status_field = 0;
	for (auto &chn : m_chns)
		chn.env_level = chn.env_target = chn.env_rate = 0.0;
	m_int_callback(CLEAR_LINE);
}

//-------------------------------------------------
//  rom_bank_pre_change - refresh the stream if the
//  ROM banking changes
//-------------------------------------------------

void mb87419_mb87420_device::rom_bank_pre_change()
{
	// unused right now
	m_stream->update();
}


u8 mb87419_mb87420_device::read(offs_t offset)
{
	if (m_env_engine && offset <= 0x03)
	{
		// 00 - which voice is asking for its next envelope segment.  Reading it is the
		// acknowledge: the line drops, and rises again if another voice is still waiting.
		if (offset == 0x00)
		{
			// Reading it is the acknowledge for that voice.
			if (ENV_TRACE)
				logerror("ENVACK %f v%02X pending=%08X %s\n",
						machine().time().as_double(), m_env_service, m_env_pending,
						BIT(m_env_pending, m_env_service) ? "real" : "STALE");
			m_env_pending &= ~(1u << m_env_service);
			m_env_offered = false;
			return m_env_service;
		}
		// 02/03 - the selected voice's current level, in whichever field the last select
		// asked for.  The firmware glues the low 16 and high 10 together and normalises.
		m_stream->update();
		u32 const level = (u32)std::clamp(m_chns[m_status_voice].env_level, 0.0, ENV_FULL - 1.0);
		u16 value;
		switch (m_status_field)
		{
		case 0:  value = level & 0xFFFF; break;                 // select 0x10
		case 1:  value = (level >> 16) & 0x03FF; break;         // select 0x12
		// Select 0x1A is NOT a level at all: it reads the voice's current PLAY ADDRESS.
		// 0x71AA selects a voice here, reads 0x1404, and compares it with 3720[voice] --
		// and 3720[voice] is written at 0x66BA from the very word the firmware just sent to
		// register 0x0E, the sample LOOP address.  So the question being asked is "has this
		// voice reached its loop point yet", which is what gates several envelope phases.
		// Same units as reg 0x0E: the address is 18.14 fixed point and the loop offset is
		// in 4-byte units, so the high word of the accumulator is exactly it.
		case 3:  value = (m_chns[m_status_voice].addr >> 16) & 0xFFFF; break;
		default:                                                // select 0x16
			{
				// Select 0x16 hands back the voice's whole current SEGMENT: the level it has
				// reached in the high byte, on register 07's own scale (what the firmware's
				// normalise-and-take-the-exponent at 0x7683 produces), and the rate it is
				// running at in the low byte.
				//
				// The low byte matters.  The release handler at 0xBEBD reads this word,
				// scales the rate down through 0xC0C4 (`shrab 42, #01`) and writes it back,
				// which is how an exponential release is built out of linear segments -- and
				// the sustain handler at 0xBE99 has to `clrb` that byte to get a rate of
				// zero, which only makes sense if it is normally non-zero.  Returning 0 here
				// left every release a single segment followed by a permanent hold.
				// Select 0x16 hands back the voice's SEGMENT REGISTER PAIR verbatim -- the
				// target level in the high byte, the ramp rate in the low.  Nothing is
				// computed here; this is a read-back of what the CPU wrote.
				//
				// The firmware says so at 0x7655.  That routine selects 0x16, reads it,
				// DISCARDS the value, then selects 0x10 and 0x12, glues the low 16 and high
				// 10 bits of the LINEAR level together, and derives the logarithm itself:
				//
				//     7690: shlb 40, #04     shift count << 4   -> exponent
				//     7693: shrb 47, #03
				//     7696: andb 47, #0f                        -> 4-bit mantissa
				//     7699: orb  40, 47      16 units per octave
				//
				// If 0x16 already carried a log level that routine would use it instead of
				// paying for the normalise, so it does not carry one -- and a gate array
				// driving 31 voices in 1988 has no business computing log2 in the first
				// place.  The discarded read is there for its side effect: it latches the
				// level so the 0x10/0x12 pair cannot tear while a ramp is running.
				//
				// This matters because the ladder handler at 0x42D0 folds
				// (reported - target)/2 into every segment's rate.  Computing a log of the
				// INSTANTANEOUS level made that term +8 -- one whole rung, so half the decay
				// speed -- for any segment whose target sat 16 units below the current
				// level.  Scored over listen/hardware/env3 (76 trials,
				// tools/scratch_analyse.py) returning the register pair takes the mean
				// absolute error from 28.5% to 14.1%, the worst segment from 65% to 33%,
				// Fretless Bass from 53% to 12% and VELO SENS from 21% to 11%.
				//
				// `[I]` Bell 1 still decays 62% FAST and is untouched by this, so at least
				// one more fault lives nearby.
				value = m_chns[m_status_voice].volume;
			}
			break;
		}
		return (offset == 0x02) ? (value & 0xFF) : (value >> 8);
	}

	// Note: only offset 0x01 is verified, the rest is probably all wrong
	if (offset != 0x01)
		logerror("Reading Reg %02X\n", offset);
	if (offset < 0x10)
	{
		pcm_channel& chn = m_chns[m_sel_chn];
		switch(offset)
		{
		case 0x00:
			return (chn.mode >> 0) & 0xFF;
		case 0x01:
			m_stream->update();
			{
				offs_t addr = (chn.addr >> 14) | ((chn.bank & 0x3C00) << 8);
				return read_byte(addr); // return sample data
			}
		case 0x02:  // ROM bank LSB
			return (chn.bank >> 0) & 0xFF;
		case 0x03:  // ROM bank MSB
			return (chn.bank >> 8) & 0xFF;
		case 0x04:  // sample step LSB
			return (chn.step >> 0) & 0xFF;
		case 0x05:  // sample step LSB
			return (chn.step >> 8) & 0xFF;
		case 0x06:  // volume LSB
			return (chn.volume >> 0) & 0xFF;
		case 0x07:  // volume MSB
			return (chn.volume >> 8) & 0xFF;
		case 0x08:  // current address, fraction LSB
			return (chn.start >>  0) & 0xFF;
		case 0x09:  // current address, fraction MSB
			return (chn.start >>  8) & 0xFF;
		case 0x0A:  // current address LSB
			return (chn.start >> 16) & 0xFF;
		case 0x0B:  // current address MSB
			return (chn.start >> 24) & 0xFF;
		case 0x0C:  // sample end address LSB
			return (chn.end >> 0) & 0xFF;
		case 0x0D:  // sample end address MSB
			return (chn.end >> 8) & 0xFF;
		case 0x0E:  // sample loop address LSB
			return (chn.loop >> 0) & 0xFF;
		case 0x0F:  // sample loop address MSB
			return (chn.loop >> 8) & 0xFF;
		}
	}
	else
	{
		switch(offset)
		{
		case 0x11:
		case 0x13:
		case 0x15:
		case 0x17:
			m_stream->update();
			{
				uint8_t basechn = ((offset >> 1) & 0x03) * 8;
				uint8_t result = 0x00;
				for (uint8_t ch_id = 0; ch_id < 8; ch_id ++)
					result |= (m_chns[basechn + ch_id].enable << ch_id);
				return result;
			}
			break;
		case 0x1F:
			return m_sel_chn;
		}
	}
	return 0x00;
}

void mb87419_mb87420_device::write(offs_t offset, u8 data)
{
	logerror("Reg %02X = %02X\n", offset, data);
	if (offset < 0x10)
	{
		pcm_channel& chn = m_chns[m_sel_chn];
		switch(offset)
		{
		case 0x00:
			chn.mode = (chn.mode & 0xFF00) | (data << 0);
			break;
		case 0x01:
			chn.mode = (chn.mode & 0x00FF) | (data << 8);
			break;
		case 0x02:  // ROM bank LSB
			chn.bank = (chn.bank & 0xFF00) | (data << 0);
			break;
		case 0x03:  // ROM bank MSB / loop mode
			chn.bank = (chn.bank & 0x00FF) | (data << 8);
			break;
		case 0x04:  // sample step LSB
			chn.step = (chn.step & 0xFF00) | (data << 0);
			break;
		case 0x05:  // sample step LSB
			chn.step = (chn.step & 0x00FF) | (data << 8);
			break;
		case 0x06:  // envelope rate (signed) -- see roland_lp.h
			if (m_env_engine)
				m_stream->update();     // gated: an extra update changes the engine-off output
			chn.volume = (chn.volume & 0xFF00) | (data << 0);
			// Deliberately NOT env_segment() here.  The firmware always writes the pair as
			// one 16-bit store and the bus splits it into 06 then 07, so acting on the low
			// byte alone evaluates the new rate against the OLD target -- which trivially
			// looks like an arrival and advances the phase machine a step early.
			break;
		case 0x07:  // envelope target level (log, 16 units per octave)
			if (m_env_engine)
				m_stream->update();
			chn.volume = (chn.volume & 0x00FF) | (data << 8);
			if (m_env_engine)
				env_segment(chn);
			break;
		case 0x08:  // current address, fraction LSB
			chn.start = (chn.start & 0xFFFFFF00) | (data <<  0);
			if (chn.enable)
				logerror("Roland PCM, channel %u: changing start address while playing!\n", m_sel_chn);
			break;
		case 0x09:  // current address, fraction MSB
			chn.start = (chn.start & 0xFFFF00FF) | (data <<  8);
			if (chn.enable)
				logerror("Roland PCM, channel %u: changing start address while playing!\n", m_sel_chn);
			break;
		case 0x0A:  // current address LSB
			chn.start = (chn.start & 0xFF00FFFF) | (data << 16);
			if (chn.enable)
				logerror("Roland PCM, channel %u: changing start address while playing!\n", m_sel_chn);
			break;
		case 0x0B:  // current address MSB
			chn.start = (chn.start & 0x00FFFFFF) | (data << 24);
			if (chn.enable)
				logerror("Roland PCM, channel %u: changing start address while playing!\n", m_sel_chn);
			break;
		case 0x0C:  // sample end address, LSB
			chn.end = (chn.end & 0xFF00) | (data << 0);
			break;
		case 0x0D:  // sample end address MSB
			chn.end = (chn.end & 0x00FF) | (data << 8);
			break;
		case 0x0E:  // sample loop address LSB
			chn.loop = (chn.loop & 0xFF00) | (data << 0);
			break;
		case 0x0F:  // sample loop address MSB
			chn.loop = (chn.loop & 0x00FF) | (data << 8);
			break;
		}
	}
	else
	{
		switch(offset)
		{
		case 0x11:
		case 0x13:
		case 0x15:
		case 0x17:
			{
				uint8_t basechn = ((offset >> 1) & 0x03) * 8;
				for (uint8_t ch_id = 0; ch_id < 8; ch_id ++)
				{
					pcm_channel& chn = m_chns[basechn + ch_id];
					bool play = static_cast<bool>((data >> ch_id) & 1);

					if (play && ! chn.enable)
					{
						chn.env = 1.0;
						chn.releasing = false;
						if (m_env_engine)
						{
							// A new note starts from silence and the attack walks up; the
							// volume word the firmware wrote just before is the first
							// segment.
							m_env_pending &= ~(1u << (basechn + ch_id));
							chn.env_level = 0.0;
							env_segment(chn);
						}
						chn.acc = chn.acc_prev = 0.0;
						chn.pp_sign = 1;
						chn.pp_off = 0.0;
						chn.addr = chn.start;
						offs_t addr = (chn.addr >> 14) | ((chn.bank & 0x3C00) << 8);
						chn.smpl_cur = chn.smpl_prv = 0;
						std::fill(std::begin(chn.hist), std::end(chn.hist), int16_t(0));
						chn.hist_pos = 0;
						chn.smpl_nxt = (m_pcm_mode == PCM_LINEAR8)
								? (int16_t)(int8_t)read_byte(addr) * 16
								: (m_pcm_mode == PCM_FLOAT8_DELTA)
								? (int16_t)(chn.acc = decode_sample((int8_t)read_byte(addr)))
								: decode_sample((int8_t)read_byte(addr));
						chn.play_dir = +1;
						logerror("Starting channel %u, bank 0x%04X, addr 0x%05X.%03X\n",
							basechn + ch_id, chn.bank, chn.start >> 14, (chn.start & 0x3FFF) << 2);
						logerror("Smpl End Ofs: 0x%04X, Loop Ofs 0x%04X, Step 0x%04X, Volume %04X\n",
							chn.end << 2, chn.loop << 2, chn.step, chn.volume);
					}
					// Clearing the enable bit is the note-off.  The chip does not cut the
					// voice there -- hardware keeps sounding for another 120-250 ms while
					// it fades.  Flag the release and let the mixer run the voice down;
					// `enable` still tracks the gate exactly, so a note-on arriving during
					// the release is still seen as one (the voice is free to be reused the
					// moment the CPU wants it, which is what the firmware assumes).
					if (! play && chn.enable && m_env_release > 0.0)
						chn.releasing = true;
					chn.enable = play;
				}
			}
			break;
		// The three status selects.  Each names a voice AND picks which field of its level
		// register 02/03 will hand back; the firmware reads all three in turn at 0x7655.
		case 0x10:
		case 0x12:
		case 0x16:
		case 0x1A:
			if (m_env_engine)
			{
				m_status_voice = data & 0x1F;
				m_status_field = (offset == 0x10) ? 0 : (offset == 0x12) ? 1
							   : (offset == 0x16) ? 2 : 3;
			}
			else
			{
				logerror("Writing unknown reg %02X = %02X\n", offset, data);
			}
			break;
		case 0x1F:
			m_sel_chn = data & 0x1F;
			break;
		default:
			logerror("Writing unknown reg %02X = %02X\n", offset, data);
			break;
		}
	}
}

// ---------------------------------------------------------------------------------
// The envelope ramp generator.  See roland_lp.h for what was measured and where.
// ---------------------------------------------------------------------------------

// Register 07 -> the chip's linear level.  The firmware's own readback normalises the
// 26-bit level into exponent*16 + mantissa, so this is that map run backwards.  Level 0 is
// a special case and has to be exactly zero: the power-up loop at 0xCC1F spins until the
// level reads 0 or 1, and an asymptote at 2^10 would hang it forever.
double mb87419_mb87420_device::env_level_for(uint8_t log_level) const
{
	if (! log_level)
		return 0.0;
	return ENV_FULL * exp2((double(log_level) - 255.0) / 16.0);
}

// One write of the volume word is one envelope segment: where to go, and how fast.
void mb87419_mb87420_device::env_segment(pcm_channel &chn)
{
	chn.env_target = env_level_for(chn.volume >> 8);

	int const rate = (int8_t)(chn.volume & 0xFF);    // SIGNED: + rises, - falls
	if (! rate)
	{
		chn.env_rate = 0.0;
		return;
	}
	// 8.01 counts per doubling, measured; the sign says which way, the magnitude how fast.
	// `[I]` Rising and falling are not symmetric, and only the rise is understood.  The
	// rising scale is pinned hard by the hardware attack times (emulator 2.142 / 0.518 /
	// 0.116 s against 2.164 / 0.540 / 0.122 measured), and the firmware's rate bytes have
	// been checked instruction by instruction and are right -- key 36, scale nibble 12,
	// 56 - 13 = 43, which is the -43 the decay uses.  Yet the same scale applied downwards
	// runs the decay about 4x too fast, and no single divisor fixes it (see
	// ENV_FALL_DIVISOR).  That echoes the hardware measurement that rising ramps are linear
	// in amplitude while falling ones are exponential, which this does not yet model.
	double scale = ENV_RATE_SCALE;
	if (rate < 0)
		scale /= ENV_FALL_DIVISOR;
	double const speed = exp2(double(abs(rate)) / 8.0) * scale;
	chn.env_rate = (rate > 0) ? speed : -speed;

	// A segment can be written that is already at, or past, its target.  It is not a no-op:
	// it finishes IMMEDIATELY, and the handler has to be told so, because that interrupt is
	// what advances the envelope to its next phase.
	//
	// Tone data does this on purpose.  Marimba's phase 1 asks for a decay to exactly the
	// level the attack just reached -- a zero-length segment whose only job is to move the
	// phase on, with the real decay in a later phase.  Swallowing it silently left marimba
	// and Fretless Bass stuck in phase 1, holding flat for the whole note.
	if ((chn.env_rate > 0.0 && chn.env_level >= chn.env_target)
			|| (chn.env_rate < 0.0 && chn.env_level <= chn.env_target))
	{
		chn.env_level = chn.env_target;
		chn.env_rate = 0.0;
		env_arrived(&chn - &m_chns[0]);
	}
}

TIMER_CALLBACK_MEMBER(mb87419_mb87420_device::env_tick)
{
	// Advance the audio -- and with it the envelope ramps -- before deciding what to offer
	// the CPU.  Without this the level only moves when MAME next updates the sound stream,
	// which is every 20 ms, so a voice could arrive at its target and be told about it at
	// most fifty times a second.  An envelope segment is often much shorter than that: the
	// release chain quantised to a flat 20 ms per segment and saturated around 316 dB/s
	// however large a rate the firmware asked for, against 5 to 1200 dB/s on hardware.
	m_stream->update();
	env_scan();
}

// Offer the next slot to the CPU.  Free-running and round-robin: the handler at 0x41C4
// reads register 00 to learn which slot it has been given, and decides for itself whether
// that slot needs anything.  EXTINT is edge triggered on the 8x9x, so the line goes up
// here and comes down when register 00 is read.  If the CPU has not answered the previous
// offer yet, this one is simply lost -- which is what the hardware would do too.
void mb87419_mb87420_device::env_scan()
{
	// Nothing has arrived: hold the line low and leave the CPU alone.
	if (! m_env_pending)
	{
		if (m_env_int)
		{
			m_env_int = false;
			m_int_callback(CLEAR_LINE);
		}
		return;
	}
	// The 8x9x latches EXTINT on a RISING edge (i8x9x.cpp, execute_set_input), so INT is
	// PULSED: this timer runs at twice the ceiling rate and toggles, giving one clean
	// low-high transition per offer.  Holding the line high and waiting for the handler to
	// acknowledge does not work -- the first edge is made before the firmware unmasks the
	// interrupt, it is lost, and with the line already high no further edge is ever
	// produced -- and dropping and raising it in the same instant does not reliably reach
	// the CPU as a single edge either.
	//
	// ONE rising edge per offer, and no further edge until the CPU has taken this one.
	// Toggling on every tick regardless -- which is what this did -- puts an edge on the
	// line every other tick for as long as ANY voice is waiting.  The firmware's handler
	// runs about 300 us and the tick is 62.5 us, so several of those edges land while the
	// CPU is still inside the handler; the 8x9x latches one of them, and it is answered
	// AFTER the read of register 00 that acknowledged the real offer.  Register 00 then
	// names the voice it named last time, and the handler -- which does not verify that
	// the voice has arrived -- advances that voice's phase machine a second time.
	//
	// On A. Piano 1 that lands on the second voice of the V-MIX every time (see
	// analysis/ENVELOPE-DECAY.md): it is given its phase-3 segment 0.3 ms after its
	// phase-2 segment, while its level is still at the attack peak, so the rung is far
	// longer than it should be, never arrives, and the ladder stops there.
	if (m_env_int)
	{
		m_env_int = false;
		m_int_callback(CLEAR_LINE);
		return;
	}
	// An offer already on the table stays there.  The re-pulse below is only a safety
	// valve for an edge lost because the firmware had EXTINT masked; while the offer is
	// outstanding its pending bit is still set, so re-offering it names the same voice
	// and costs nothing.
	if (m_env_offered && ++ m_env_offer_age < ENV_OFFER_RETRY)
		return;

	for (unsigned i = 0; i < NUM_CHANNELS; i ++)
		if (BIT(m_env_pending, i))
		{
			m_env_service = i;
			break;
		}
	m_env_offered = true;
	m_env_offer_age = 0;
	m_env_int = true;
	m_int_callback(ASSERT_LINE);
}

// A voice's ramp reached its target: it needs its next envelope segment.
void mb87419_mb87420_device::env_arrived(unsigned voice)
{
	if (ENV_TRACE)
		logerror("ENVARR %f v%02X level=%.0f target=%.0f\n",
				machine().time().as_double(), voice, m_chns[voice].env_level,
				m_chns[voice].env_target);
	m_env_pending |= 1u << voice;
}

// The voice volume pair is NOT a linear 16-bit magnitude, and it is not two levels: it is
// an ENVELOPE SEGMENT.  Register 07 is the TARGET level and register 06 is the SIGNED ramp
// RATE the chip uses to get there (+1..+0x7F rising, 0x80..0xFF = -128..-1 falling).
//
// Register 07 is **logarithmic**.  The firmware's own conversion table at 0xAEC6 reads
// 143, 159, 175, 191, 207, 223, 239, 255 at indices 1, 2, 4, 8, 16, 32, 64, 128 -- every
// doubling of amplitude adds exactly 16, so the scale is 16 units per octave, i.e.
// 6.0206/16 = 0.3763 dB per unit, full scale at 0xFF.
//
// Read linearly (as MAME did) a velocity sweep spanning 21.3 dB on real hardware collapses
// to 5.3 dB.  Decoded this way the same sweep predicts -20.6 dB where hardware measures
// -21.3 dB.  `[I]` That 1.3 dB residual is still open.
//
// The rate byte is ignored here, and so is the whole envelope, because this device never
// asserts m_int_callback.  On hardware the chip ramps the level and raises EXTINT when a
// voice needs its next segment; the firmware's handler at 0x41C4 is the U-110's entire TVA
// envelope generator (see analysis/ROM-ANALYSIS.md, "The amplitude envelope lives in the
// CPU").  Without the interrupt every voice holds its note-on target forever -- which is
// exactly why nothing decays.  Using only reg 07 is therefore right as far as it goes: it
// is the level the envelope is heading to.
static inline uint32_t volume_gain(uint16_t vol)
{
	static bool init = false;
	static uint32_t lut[256];
	if (! init)
	{
		for (int i = 0; i < 256; i ++)
			lut[i] = (uint32_t)(65535.0 * pow(2.0, (i - 255) / 16.0) + 0.5);
		init = true;
	}
	return lut[(vol >> 8) & 0xFF];
}

//-------------------------------------------------
//  set_mix_gain_db - output gain, applied where a
//  voice joins the mix
//-------------------------------------------------

void mb87419_mb87420_device::set_mix_gain_db(double db)
{
	// Everything already rendered was rendered at the old gain, so flush first: without
	// this an adjustment made mid-block retroactively rescales the block.
	if (m_stream != nullptr)
		m_stream->update();
	m_out_gain_db = db;
	m_out_scale = pow(10.0, db / 20.0) / VOICE_FS;
}


//-------------------------------------------------
//  sound_stream_update - handle a stream update
//-------------------------------------------------

void mb87419_mb87420_device::sound_stream_update(sound_stream &stream)
{
	for (auto& chn : m_chns)
	{
		// With the ramp generator the fade after note-off is the envelope's, driven by the
		// firmware, so the voice lives exactly as long as its enable bit says.  Without it,
		// `releasing` keeps the voice alive for the device's own stand-in fade.
		if (m_env_engine ? ! chn.enable : (! chn.enable && ! chn.releasing))
			continue;
		if (chn.play_dir == 0)
			continue;

		for (int smpl = 0; smpl < stream.samples(); smpl ++)
		{
			s32 smp_raw;
			if (recon_mode() == 3 && chn.play_dir > 0 && chn.step <= 0x4000)
			{
				// Catmull-Rom.  The three kernels measured before this one bracket the
				// hardware: in the stopband linear (sinc^2) is 11 dB too bright and a
				// 16-tap windowed sinc 13 dB too dark, while the quadratic B-spline
				// (sinc^3) lands at 1.9; in the passband linear is 4 dB too dark and the
				// sinc is right.  So the chip wants a FLAT passband with a stopband
				// between sinc^2 and sinc^3, which is an interpolating cubic -- and four
				// taps is buildable silicon in a way a polyphase FIR is not.
				//
				// Evaluated one source sample behind so the fourth tap comes from history
				// rather than needing a look-ahead; that costs a constant delay of one
				// source sample and nothing else.
				double const t = double(chn.addr & 0x3FFF) / 16384.0;
				double const p3 = chn.smpl_nxt;
				double const p2 = chn.hist[chn.hist_pos];
				double const p1 = chn.hist[(chn.hist_pos + HIST_LEN - 1) % HIST_LEN];
				double const p0 = chn.hist[(chn.hist_pos + HIST_LEN - 2) % HIST_LEN];
				double const v = 0.5 * ((2.0 * p1)
						+ (-p0 + p2) * t
						+ (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t * t
						+ (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t * t * t);
				smp_raw = (s32)(v * 16384.0);
			}
			else if (recon_mode() == 2 && chn.play_dir > 0 && chn.step <= 0x4000)
			{
				double const *const k = sinc_table()
						+ (((chn.addr & 0x3FFF) * SINC_PHASES) >> 14) * SINC_TAPS;
				double v = 0.0;
				for (int j = 0; j < SINC_TAPS; j ++)
				{
					int16_t const smp = (j == 0) ? chn.smpl_nxt
							: chn.hist[(chn.hist_pos + HIST_LEN - (j - 1)) % HIST_LEN];
					v += k[j] * double(smp);
				}
				smp_raw = (s32)(v * 16384.0);
			}
			else if (recon_mode() == 1 && chn.play_dir > 0 && chn.step <= 0x4000)
			{
				// Quadratic B-spline.  Weights sum to 1;
				//
				// STEP <= 1 ONLY, and the gate is not optional.  smpl_cur and smpl_nxt are
				// always adjacent bytes, but the third tap comes from the previous OUTPUT
				// sample, which is `step` bytes back -- one byte or two, depending where
				// the fraction lands.  Above step 1 that makes the three taps irregularly
				// spaced while these weights assume unit spacing, and the spacing pattern
				// beats against the fractional part of the step: at P-27 Synth Bass note 57
				// (steps 1.1279 and 1.3351, loops of 123 and 97 bytes) it beats every ~8
				// output samples on a strongly periodic tone, which is plainly audible as
				// roughness.  Linear uses only the adjacent pair and is unaffected. the kernel is one order wider
				// than the triangle a linear interpolation convolves with, which costs a
				// constant half-source-sample of delay and buys a factor of sinc.
				double const t = double(chn.addr & 0x3FFF) / 16384.0;
				double const a = (chn.play_dir > 0) ? chn.smpl_prv : chn.smpl_nxt;
				double const c = (chn.play_dir > 0) ? chn.smpl_nxt : chn.smpl_prv;
				double const v = 0.5 * (1.0 - t) * (1.0 - t) * a
						+ (0.75 - (t - 0.5) * (t - 0.5)) * double(chn.smpl_cur)
						+ 0.5 * t * t * c;
				smp_raw = (s32)(v * 16384.0);
			}
			else
				smp_raw = (chn.play_dir > 0)
					? sample_interpolate_raw(chn.smpl_cur, chn.smpl_nxt, chn.addr & 0x3FFF)
					: sample_interpolate_raw(chn.smpl_nxt, chn.smpl_cur, chn.addr & 0x3FFF);
			// Mode 2 keeps the fraction and lets the envelope multiply see it; 0 and 1
			// quantise here, flooring or rounding.  smp_scale carries the divisor mode 2
			// still owes.
			double smp_scale = 1.0;
			s32 smp_data;
			switch (interp_mode())
			{
			case 1:  smp_data = (smp_raw + 0x2000) >> 14; break;
			case 2:  smp_data = smp_raw; smp_scale = 1.0 / 16384.0; break;
			default: smp_data = smp_raw >> 14; break;
			}
			// Note the casts: volume_gain() returns uint32_t, and s32 * uint32_t is
			// evaluated UNSIGNED, so a negative sample would wrap to a huge positive
			// value before the envelope multiply ever saw it.
			if (m_env_engine)
			{
				// The ramp generator IS the volume: the level is where the envelope has
				// got to, on the same 0..2^26 scale full output sits at.  volume_gain()'s
				// 16-bit result is what the mixer's 32768 << 14 divisor expects, so scale
				// to that.
				smp_data = (s32)((double)smp_data * smp_scale * chn.env_level * (65536.0 / ENV_FULL));

				if (chn.env_rate > 0.0)
				{
					chn.env_level += chn.env_rate;
					if (chn.env_level >= chn.env_target)
					{
						chn.env_level = chn.env_target;
						chn.env_rate = 0.0;
						env_arrived(&chn - &m_chns[0]);
					}
				}
				else if (chn.env_rate < 0.0)
				{
					chn.env_level += chn.env_rate;
					if (chn.env_level <= chn.env_target)
					{
						chn.env_level = chn.env_target;
						chn.env_rate = 0.0;
						env_arrived(&chn - &m_chns[0]);
					}
				}
			}
			else
			{
				smp_data = (s32)((double)smp_data * smp_scale * (double)volume_gain(chn.volume) * chn.env);
				chn.env *= chn.releasing ? m_env_release_step : m_env_decay_step;
				if (chn.releasing && chn.env < 6.0e-5)   // -84 dB: below the DAC's own floor
				{
					chn.releasing = false;
					chn.play_dir = 0;
					chn.env = 0.0;
					break;
				}
			}
			// Scale ONCE, here, where smp_data is still the full-width integer product
			// of the decoded sample and the envelope multiplier.  m_out_scale is
			// gain / VOICE_FS -- see VOICE_FS in the header for where 2^27 comes from,
			// and set_mix_gain_db() for the gain.
			auto const out_smp = sound_stream::sample_t(double(smp_data) * m_out_scale);
			if (m_out_count <= 2)
			{
				// historical behaviour: every voice to both channels
				stream.add(0, smpl, out_smp);
				stream.add(1, smpl, out_smp);
			}
			else
			{
				// one stream per Multi Output; a voice may feed several (a centred group
				// carries mask 0x03, i.e. Multi Outputs 1 and 2 together)
				u8 const mask = m_voice_mask[&chn - &m_chns[0]];
				for (int out = 0; out < m_out_count; out ++)
					if (BIT(mask, out))
						stream.add(out, smpl, out_smp);
			}

			uint32_t old_addr = chn.addr;
			if (chn.play_dir > 0)
				chn.addr += chn.step;
			else if (chn.play_dir < 0)
				chn.addr -= chn.step;

			// Ping-pong reverses BEFORE the read, unlike the other loop modes.  Reading
			// first means the byte fetched at a reversal comes from beyond `end`, which
			// in this ROM is silence, and that one wrong sample is an audible click on
			// every cycle -- measured against hardware, which has none (8-9 per 8 s note
			// on Choir 3).  The forward/backward asymmetry is why it is once per cycle
			// and not twice: the read past `loop` still lands inside the sample.
			// Ping-pong folds BEFORE the read, unlike the other loop modes.  Reading first
			// takes the address one byte past the turning point -- outside the sample -- and
			// applies that byte's delta to the accumulator, so the value used to reflect
			// about is the wrong one.  The error is small per turn but accumulates: measured
			// against tools/stitch_pingpong.py the residual went 0.30 before the first turn,
			// 0.69 after it and 0.94 by the third.  Folding first keeps every read inside
			// [loop, end] and makes chn.acc exactly the pivot value.
			bool turned = false;
			uint32_t pivot_b = 0;       // byte index of the turning point, when `turned`
			bool wrapped = false;       // forward loop wrapped; walk across it in two parts
			uint32_t wrap_end_b = 0, wrap_start_b = 0;
			if ((chn.bank >> 14) == 2)
			{
				uint32_t const hi = (uint32_t)chn.end << 16;     // `end` is played
				uint32_t const lo = (uint32_t)chn.loop << 16;
				// The +0x4000 corrects for the interpolator's direction-dependent lag.
				// sample_interpolate(a, b, f) evaluates a*(1-f) + b*f, and after a fetch at
				// byte `b` smpl_nxt = w[b] while smpl_cur is whichever byte was fetched
				// BEFORE it -- w[b-1] running forward, w[b+1] running backward.  So the
				// output position is (addr - 1) forward but (addr) backward.  Reflecting
				// `addr` alone about the pivot therefore reflects the OUTPUT about
				// pivot - 0.5, and the backward turn does not even reverse: the position
				// keeps descending through `lo` before it climbs again.  Reflect the output
				// position instead -- q_new = 2*pivot - q_cont -- which in address terms is
				// addr_new = 2*pivot + 1 - addr for both turns.
				if (chn.play_dir > 0 && chn.addr >= hi + 0x4000)
				{
					chn.addr = 2 * hi + 0x4000 - chn.addr;
					chn.play_dir = -1;
					turned = true;
					pivot_b = hi >> 14;
				}
				else if (chn.play_dir < 0 && chn.addr < lo)
				{
					chn.addr = 2 * lo + 0x4000 - chn.addr;
					chn.play_dir = +1;
					turned = true;
					pivot_b = lo >> 14;
				}
			}
			else if (m_pcm_mode == PCM_FLOAT8_DELTA && (chn.bank >> 14) == 0
					&& chn.play_dir > 0 && (chn.addr >> 16) >= chn.end)
			{
				// A forward loop has to wrap BEFORE the read too, for the same reason the
				// ping-pong turn does.  The generic handler below wraps after the fetch, so
				// the bytes between `end` and the overshoot get integrated even though they
				// lie outside the loop -- they belong to whatever sample follows in the ROM.
				//
				// Below one byte per output sample this never bites: the byte index rises by
				// exactly one, so it lands ON `end`, and since every loop's deltas sum to
				// zero, W[end] == W[loop] and the wrap is seamless.  Above one byte per
				// sample the index jumps to end+1 and a foreign delta is applied once per
				// loop, which is precisely the DC ramp measured on Choir 3 at note 67.
				wrapped = true;
				wrap_end_b = (uint32_t)chn.end << 2;
				wrap_start_b = (uint32_t)chn.loop << 2;
				chn.addr = chn.addr + ((chn.loop - chn.end) << 16);
			}

			// The ROM byte is a DELTA, not the sample: the chip integrates.  Measured against
			// hardware, the wave ROM's spectrum sits a clean 6 dB/octave above the U-110's own
			// output across six octaves, which is exactly one integration
			// (analysis/deemphasis.pdf).
			//
			// Direction matters.  If the byte is a delta then x[n] = x[n-1] + d[n], so stepping
			// BACKWARD subtracts the delta of the byte being left behind.  Always adding (what
			// the CM-32P code does) means the reverse pass of a ping-pong integrates fresh
			// values instead of retracing the waveform -- the loop is then not a loop at all,
			// the two passes share no samples, and there is nothing to weld at the turn.
			//
			// EVERY byte on the path has to be integrated, including bytes that are stepped
			// over and never output.  The rest of this device fetches at most one byte per
			// output sample, which is right for ordinary PCM -- skipping a byte there is just
			// aliasing -- but wrong for delta data: a skipped delta is never applied, so a
			// DC-balanced loop stops summing to zero and the accumulator ramps.  Measured on
			// Choir 3 at note 67 (step 0x42CF = 1.044 bytes/sample), DC walked to -6468 over a
			// 15 s hold while the hardware held +/-1.  All 226 wave ROM loops sum to exactly
			// zero, so a correct walk cannot drift.  8 of 228 voices in the reference capture
			// exceed one byte per sample, the worst at 1.880.
			if (m_pcm_mode == PCM_FLOAT8_DELTA)
			{
				offs_t const bank_base = (chn.bank & 0x3C00) << 8;
				uint32_t const old_b = old_addr >> 14;
				uint32_t const new_b = chn.addr >> 14;

				// Walk from byte `from_b` to byte `to_b`, applying each delta with the sign
				// its direction of travel calls for.  acc_prev trails one byte behind, which
				// is the second interpolation tap.
				auto walk = [&] (uint32_t from_b, uint32_t to_b, int dir)
				{
					if (dir > 0)
						for (uint32_t b = from_b; b < to_b; b ++)
						{
							chn.acc_prev = chn.acc;
							chn.acc = chn.acc * m_delta_keep
									+ decode_sample((int8_t)read_byte((b + 1) | bank_base));
						}
					else
						for (uint32_t b = from_b; b > to_b; b --)
						{
							chn.acc_prev = chn.acc;
							chn.acc = chn.acc * m_delta_keep
									- decode_sample((int8_t)read_byte(b | bank_base));
						}
				};

				if (wrapped)
				{
					// Up to `end`, then carry on from `loop`.  No delta is applied across the
					// join itself: the loop sums to zero, so W[end] == W[loop] already.
					walk(old_b, wrap_end_b, +1);
					walk(wrap_start_b, new_b, +1);
				}
				else if (turned)
				{
					// Reach the pivot in the OLD direction first, so the value reflected about
					// is the one AT the turning point, then walk back out in the new direction.
					// With a step under one byte these two walks are one byte and zero-or-one
					// byte, which is what the previous single-fetch code happened to do.
					walk(old_b, pivot_b, -chn.play_dir);
					// out = s*acc + o; continuity at a turn where acc = c needs
					//     o_new = o_old + 2*s_old*c ,  s_new = -s_old
					chn.pp_off += 2.0 * chn.pp_sign * chn.acc;
					chn.pp_sign = -chn.pp_sign;
					walk(pivot_b, new_b, chn.play_dir);
				}
				else
					walk(old_b, new_b, chn.play_dir);

				if (turned || wrapped || new_b != old_b)
				{
					// Store both taps already transformed.  Interpolating raw accumulator
					// values and applying sign/offset afterwards is wrong across a turn,
					// because the two taps belong to different sign/offset contexts; the
					// pair either side of a turn must each carry its own.  smpl_cur cannot
					// be the previous smpl_nxt any more: with a step over one byte the
					// previous fetch is not the neighbouring byte.
					chn.smpl_prv = chn.smpl_cur;
					chn.smpl_cur = (int16_t)std::clamp(
							chn.pp_sign * chn.acc_prev + chn.pp_off, -8191.0, +8191.0);
					chn.push_hist(chn.smpl_cur);
					chn.smpl_nxt = (int16_t)std::clamp(
							chn.pp_sign * chn.acc + chn.pp_off, -8191.0, +8191.0);
				}
			}
			// Note: The sample data is read after incrementing the address, but before handling loop points.
			//       Reading the sample data after loop point handling breaks the test mode square wave.
			//       (Sample 0xA7 on the CM-32P.)
			else if ((chn.addr >> 14) != (old_addr >> 14))
			{
				offs_t const bank_base = (chn.bank & 0x3C00) << 8;
				offs_t addr = (chn.addr >> 14) | bank_base;
				//logerror("Chn %u: sample addr 0x%05X.%03X\n", &chn - &m_chns[0], chn.addr >> 14, (chn.addr & 0x3FFF) << 2);
				chn.smpl_prv = chn.smpl_cur;
				chn.smpl_cur = chn.smpl_nxt;
				chn.push_hist(chn.smpl_cur);
				if (m_pcm_mode == PCM_FLOAT8)
				{
					// Sarayan's 1-3-4 rule is the correct decoder, but it is memoryless:
					// the byte IS the sample, not a delta.  Verified against the U-110's own
					// Sound Check tone (wave-ROM sample 212), which the firmware plays as a
					// pure sine: read linearly the ROM data is a trapezoid with h3 at
					// -15.3 dB, and decoded here it is a sine to 0.99975 with h3 at -66.7 dB.
					// The decoded/ideal ratio holds constant to 2.5% over a 14:1 amplitude
					// range, against 32% for the linear reading.  Full scale is +/-1984, so
					// no scaling and no clamp are needed.
					chn.smpl_nxt = decode_sample((int8_t)read_byte(addr));
				}
				else if (m_pcm_mode == PCM_LINEAR8)
				{
					// Plain two's complement 8-bit PCM.  Kept for comparison; this is NOT how
					// the U-110 reads its wave ROM.  x16 puts a full-scale sample at +/-2048.
					chn.smpl_nxt = (int16_t)(int8_t)read_byte(addr) * 16;
				}
				else
				{
					chn.smpl_nxt += decode_sample((int8_t)read_byte(addr)); // This was verified to be independent from play_dir.

					// until the decoding is fixed, we prevent overflow bugs (due to DC offsets when looping) this way
					chn.smpl_nxt = std::clamp<int16_t>(chn.smpl_nxt, -0x7FF, +0x7FF);
				}
			}

			bool reachedEnd = false;
			if (chn.play_dir > 0)
			{
				// For ping-pong, `end` names a sample that IS played: the integral of the
				// deltas returns to zero exactly there (-1 on wave-ROM sample 121, -15 on
				// 122, against waveforms swinging +/-800), and so does `loop`.  Roland put
				// both turning points on zero crossings of the integrated waveform.  Turning
				// one byte early lands on +151 instead and the join jumps.
				if ((chn.bank >> 14) != 2 && (chn.addr >> 16) >= chn.end)
					reachedEnd = true;
			}
			else if (chn.play_dir < 0)
			{
				if ((chn.bank >> 14) != 2 && (chn.addr >> 16) < chn.loop)
					reachedEnd = true;
			}
			if (reachedEnd)
			{
				offs_t oldAddr = chn.addr;
				logerror("Chn %u: Sample End at addr 0x%05X.%03X. (Dir %d, Loop Mode %u)\n",
					&chn - &m_chns[0], chn.addr >> 14, (chn.addr & 0x3FFF) << 2,
					chn.play_dir, chn.bank >> 14);
				switch(chn.bank >> 14)
				{
				case 0: // normal loop
					chn.addr = chn.addr + ((chn.loop - chn.end) << 16);
					logerror("addr 0x%05X.%03X -> addr 0x%05X.%03X\n",
						oldAddr >> 14, (oldAddr & 0x3FFF) << 2,
						chn.addr >> 14, (chn.addr & 0x3FFF) << 2);
					break;
				case 1: // no loop
				case 3: // invalid, assume "no loop"
					chn.play_dir = 0;
					break;
				case 2: // ping-pong
					// Unreachable: both reachedEnd tests above exclude mode 2, because a
					// ping-pong turn has to be handled BEFORE the read, not after it.  The
					// live implementation is up there with the address advance.  This branch
					// once held a half-sample pivot and a non-inverting reverse pass; both
					// were wrong and the reasoning that replaced them is at the fold site.
					break;
				}
				if (chn.play_dir == 0)
					break;
			}
		}
	}

	return;
}

int16_t mb87419_mb87420_device::decode_sample(int8_t data)
{
	int16_t val;
	int16_t sign;
	uint8_t shift;
	int16_t result;

	if (data < 0)
	{
		sign = -1;
		val = -data;
	}
	else
	{
		sign = +1;
		val = data;
	}

	// thanks to Sarayan for figuring out the decoding formula
	shift = val >> 4;
	val &= 0x0F;
	if (! shift)
		result = val;
	else
		result = (0x10 + val) << (shift - 1);
	return result * sign;
}

int16_t mb87419_mb87420_device::sample_interpolate(int16_t smp1, int16_t smp2, uint16_t frac)
{
	return (int16_t)(sample_interpolate_raw(smp1, smp2, frac) >> 14);
}

// The same interpolation with the 14 fractional bits still attached.  smp1 and smp2 span
// +/-2048, so this spans +/-2048 << 14 and cannot overflow a s32.
//
// Which of the two the mixer uses is the question U110_INTERP exists to answer.  The
// truncating form above quantises every voice to a 12-bit integer BEFORE the envelope
// multiplies it, so its error is signal-scaled -- it follows the note down instead of
// sitting at a fixed floor, which is exactly the shape of the 6-9 kHz excess.
int32_t mb87419_mb87420_device::sample_interpolate_raw(int16_t smp1, int16_t smp2, uint16_t frac)
{
	int32_t const smpfrac0 = (int32_t)smp1 * (0x4000 - frac);
	int32_t const smpfrac1 = (int32_t)smp2 * frac;
	return smpfrac0 + smpfrac1;
}
