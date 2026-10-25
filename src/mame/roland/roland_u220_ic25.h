// license:BSD-3-Clause
// copyright-holders:Helge Thorsten Kautz
/***************************************************************************************

    Roland U-220: what is known of IC25 (Toshiba TC23SC140AF-007), shared by the MAME
    driver and the plugin core so the two cannot drift apart.

    IC25 is a small DSP.  The firmware loads it at boot with a 256-step program of 24-bit
    words from EPROM 0x1A000 (routine 0xBB10) and keeps rewriting single words of it
    while the machine plays.  The CPU reaches it through window bank 0x40:

      0x00-0x02   a 24-bit data latch, high byte first
      0x04        write the latch to data memory at this address (32 entries)
      0x06        write the latch to program memory at this address
      0x0A        read program memory at this address into the latch (the firmware does
                  read-modify-write on program words, so this has to answer correctly)
      0x0C/0x0D   control

    The MIXER is part of that program, and it is all that is modelled here.  Every one of
    the 32 voices of the tone generator has its own block of program words; their
    addresses are a table at EPROM 0x1803E, six bytes per voice (L, R, then two pairs the
    firmware fills alongside).  At each note the firmware writes the voice's L and R gain
    into the LOW byte of its two words and its destination into bits 1-5 of the MIDDLE
    byte (routines 0xBC4B and on):

      destination 0x1E/0x1F  dry          0x12/0x13  reverb     0x14/0x15  chorus
                  0x18-0x1B  direct outs (taken as dry here)

    -- the part's "Output Asgn" (Dry/Rev/Cho) and its pan, read off the emulated menus.
    The gains are linear: the hi-hat's 2B/3F and the high tom's 1E/47 come out 3.7 and
    7.8 dB right of centre on Helge's U-220, against 3.3 and 7.5 dB computed.  A melodic
    part at centre gets 1A/1A and the rhythm part 36/36, which is the 6 dB the drums were
    missing while every voice went to both channels at unity; 0x1A is taken as unity.

    The reverb and chorus themselves (the rest of the program) are NOT modelled here.

***************************************************************************************/

#ifndef MAME_ROLAND_U220_IC25_H
#define MAME_ROLAND_U220_IC25_H

#pragma once

#include <cstdint>
#include <cstring>

namespace roland_u220 {

/// Buses the mixer feeds, as outputs of the tone generator's gain matrix.
enum : int { BUS_DRY_L = 0, BUS_DRY_R, BUS_REV_L, BUS_REV_R, BUS_CHO_L, BUS_CHO_R, NUM_BUSES };

constexpr uint32_t IC25_PROGRAM_ROM   = 0x1a000;   // 256 x 3 bytes, high byte first
constexpr uint32_t IC25_VOICE_TABLE   = 0x1803e;   // 32 x 6 bytes of program addresses
constexpr unsigned IC25_NUM_VOICES    = 32;
constexpr float    IC25_UNITY_GAIN    = float(0x1a);

class ic25_mixer
{
public:
	/// `progrom` is the whole 128 KB firmware image: it holds the voice table.
	void reset(const uint8_t *progrom)
	{
		std::memset(m_latch, 0, sizeof(m_latch));
		std::memset(m_prog, 0, sizeof(m_prog));
		std::memset(m_data, 0, sizeof(m_data));
		std::memset(m_owner, 0xff, sizeof(m_owner));
		// Six program addresses per voice.  The firmware writes a side's gain into the
		// word at entry 0 (left) / 1 (right), then sets its destination by read-modify-
		// write of the word at entry 3 / 5.  For most voices those are the same word;
		// for voices 7, 9, 11, 13 and 20 they are not (v20: a7 a4 a9 a7 a6 a3), and
		// reading gain and destination from one word left v20 silent on the right --
		// fast snares jumping to the left in Ostrich, never on the unit.  So the gain is
		// taken from the one word and the destination from the other.
		for (unsigned v = 0; v < IC25_NUM_VOICES; v ++)
		{
			const uint8_t *e = progrom + IC25_VOICE_TABLE + v * 6;
			m_gain[v][0] = e[0];
			m_gain[v][1] = e[1];
			m_dest[v][0] = e[3];
			m_dest[v][1] = e[5];
			for (int k : { 0, 1, 3, 5 })
				m_owner[e[k]] = uint8_t(v);
		}
	}

	/// A CPU write into the window; returns the voice whose gains may have changed, or -1.
	int write(unsigned offset, uint8_t data)
	{
		switch (offset & 0x0f)
		{
		case 0x00: case 0x01: case 0x02:
			m_latch[offset & 0x0f] = data;
			return -1;
		case 0x04:
			m_data[data & 0x1f] = word();
			return -1;
		case 0x06:
			m_prog[data] = word();
			return m_owner[data] == 0xff ? -1 : m_owner[data];
		case 0x0a:
			m_latch[0] = uint8_t(m_prog[data] >> 16);
			m_latch[1] = uint8_t(m_prog[data] >> 8);
			m_latch[2] = uint8_t(m_prog[data]);
			return -1;
		default:
			return -1;
		}
	}

	uint8_t read(unsigned offset) const
	{
		return (offset & 0x0f) <= 2 ? m_latch[offset & 0x0f] : 0xff;
	}

	/// The gain of voice `v` on each bus, from its two program words.
	void voice_gains(unsigned v, float gains[NUM_BUSES]) const
	{
		for (int b = 0; b < NUM_BUSES; b ++)
			gains[b] = 0.0f;
		for (int side = 0; side < 2; side ++)
			route((m_prog[m_dest[v][side]] & ~0xffu) | (m_prog[m_gain[v][side]] & 0xffu), side, gains);
	}

	uint32_t program(unsigned a) const { return m_prog[a & 0xff]; }

private:
	uint32_t word() const { return uint32_t(m_latch[0]) << 16 | uint32_t(m_latch[1]) << 8 | m_latch[2]; }

	static void route(uint32_t w, int side, float gains[NUM_BUSES])
	{
		const unsigned dest = (w >> 9) & 0x1f;
		const float g = float(w & 0xff) / IC25_UNITY_GAIN;
		int bus;
		switch (dest & 0x1e)
		{
		case 0x12: bus = BUS_REV_L; break;
		case 0x14: bus = BUS_CHO_L; break;
		case 0x18: case 0x1a: case 0x1e: bus = BUS_DRY_L; break;
		default: return;                 // not a mixer word (yet): the voice is silent
		}
		gains[bus + side] += g;
	}

	uint8_t  m_latch[3];
	uint32_t m_prog[256];
	uint32_t m_data[32];
	uint8_t  m_gain[IC25_NUM_VOICES][2], m_dest[IC25_NUM_VOICES][2];   // program addresses
	uint8_t  m_owner[256];
};

} // namespace roland_u220

#endif // MAME_ROLAND_U220_IC25_H
