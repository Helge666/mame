// license:BSD-3-Clause
// copyright-holders:Valley Bell

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

#include <cmath>


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

	logerror("Roland PCM: Clock %u, Rate %u\n", m_clock, m_rate);
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void mb87419_mb87420_device::device_reset()
{
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
		case 0x06:  // volume LSB
			chn.volume = (chn.volume & 0xFF00) | (data << 0);
			break;
		case 0x07:  // volume MSB
			chn.volume = (chn.volume & 0x00FF) | (data << 8);
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
						chn.acc = chn.acc_prev = 0.0;
						chn.pp_sign = 1;
						chn.pp_off = 0.0;
						chn.addr = chn.start;
						offs_t addr = (chn.addr >> 14) | ((chn.bank & 0x3C00) << 8);
						chn.smpl_cur = 0;
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
		case 0x1F:
			m_sel_chn = data & 0x1F;
			break;
		default:
			logerror("Writing unknown reg %02X = %02X\n", offset, data);
			break;
		}
	}
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
//  sound_stream_update - handle a stream update
//-------------------------------------------------

void mb87419_mb87420_device::sound_stream_update(sound_stream &stream)
{
	for (auto& chn : m_chns)
	{
		if ((! chn.enable && ! chn.releasing) || chn.play_dir == 0)
			continue;

		for (int smpl = 0; smpl < stream.samples(); smpl ++)
		{
			s32 smp_data;
			if (chn.play_dir > 0)
				smp_data = sample_interpolate(chn.smpl_cur, chn.smpl_nxt, chn.addr & 0x3FFF);
			else
				smp_data = sample_interpolate(chn.smpl_nxt, chn.smpl_cur, chn.addr & 0x3FFF);
			// Note the casts: volume_gain() returns uint32_t, and s32 * uint32_t is
			// evaluated UNSIGNED, so a negative sample would wrap to a huge positive
			// value before the envelope multiply ever saw it.
			smp_data = (s32)((double)smp_data * (double)volume_gain(chn.volume) * chn.env);
			chn.env *= chn.releasing ? m_env_release_step : m_env_decay_step;
			if (chn.releasing && chn.env < 6.0e-5)   // -84 dB: below the DAC's own floor
			{
				chn.releasing = false;
				chn.play_dir = 0;
				chn.env = 0.0;
				break;
			}
			if (m_out_count <= 2)
			{
				// historical behaviour: every voice to both channels
				stream.add_int(0, smpl, smp_data, 32768 << 14); // >>14 results in a good overall volume
				stream.add_int(1, smpl, smp_data, 32768 << 14);
			}
			else
			{
				// one stream per Multi Output; a voice may feed several (a centred group
				// carries mask 0x03, i.e. Multi Outputs 1 and 2 together)
				u8 const mask = m_voice_mask[&chn - &m_chns[0]];
				for (int out = 0; out < m_out_count; out ++)
					if (BIT(mask, out))
						stream.add_int(out, smpl, smp_data, 32768 << 14);
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
					chn.smpl_cur = (int16_t)std::clamp(
							chn.pp_sign * chn.acc_prev + chn.pp_off, -8191.0, +8191.0);
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
				chn.smpl_cur = chn.smpl_nxt;
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
	int32_t smpfrac0 = (int32_t)smp1 * (0x4000 - frac);
	int32_t smpfrac1 = (int32_t)smp2 * frac;
	return (int16_t)((smpfrac0 + smpfrac1) >> 14);
}
