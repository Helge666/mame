// license:BSD-3-Clause
// copyright-holders:Elliott H. Liggett
/***************************************************************************************

    Roland U-110 -- shared data tables and pure functions

    Everything here is a property of the HARDWARE, not of any particular host: the ROM
    address and data permutations, the fifty output-routing modes, the measured pan
    positions, the reconstruction filter's component values, and the fitted output EQ.

    It is split out because two programs need it.  MAME's roland_u110.cpp is one; the
    Voltaire 110 plugin core is the other (see PLUGIN-PLAN.md section 3).  Machine wiring --
    machine_config, input ports, ROM regions -- correctly differs between them and stays
    in the driver.  These tables must NOT differ, so they live in one file and are
    included by both.

    THIS HEADER DEPENDS ON NOTHING.  Not emu.h, not MAME's core types -- only <cstdint> and
    <cstddef>.  That is deliberate and load-bearing: the plugin compiles it without a MAME
    tree in the include path.  Anything needing device_t belongs in the driver instead.

    Licensed BSD-3-Clause, like the device sources it accompanies, so the emulation
    findings can still be contributed upstream to MAME.  Do not paste GPL plugin-side code
    into this file.

***************************************************************************************/

#ifndef MAME_ROLAND_ROLAND_U110_DATA_H
#define MAME_ROLAND_ROLAND_U110_DATA_H

#pragma once

#include <cstddef>
#include <cstdint>

namespace roland_u110 {

/***************************************************************************************
    Wave ROM scrambling

    Both the address and data lines between the mask ROMs and the sound chip are
    permuted on the board.  A dump therefore has to be put back in order before the
    chip can address it.
***************************************************************************************/

// Same semantics as MAME's bitswap<N>(): each argument names the SOURCE bit that
// supplies one DESTINATION bit, most significant destination first.  Reimplemented here
// only so this header does not need MAME's coretmpl.h; the argument lists below are
// character-for-character what the driver used.
template <typename T, typename... B>
constexpr uint32_t bitswap_from(T value, B... bits)
{
	constexpr size_t N = sizeof...(bits);
	size_t const src[N] = { size_t(bits)... };
	uint32_t r = 0;
	for (size_t i = 0; i < N; i ++)
		r |= uint32_t((uint64_t(value) >> src[i]) & 1) << (N - 1 - i);
	return r;
}

// ROM dump offset -> proper (descrambled) offset.
constexpr uint32_t unscramble_addr(uint32_t offset)
{
	return bitswap_from(offset, 18,17,15,14,16,12,11, 7, 9,13,10, 8, 3, 2, 1, 6, 4, 5, 0);
}

constexpr uint8_t unscramble_data(uint8_t data)
{
	return uint8_t(bitswap_from(data, 1,2,7,3,5,0,4,6));
}

// Descramble one 512K bank in place-to-place.  dst must have room for len bytes at the
// permuted offsets, which for a full bank means len bytes.
inline void descramble_pcm(uint8_t *dst, const uint8_t *src, size_t len)
{
	for (size_t srcpos = 0; srcpos < len; srcpos ++)
		dst[unscramble_addr(uint32_t(srcpos))] = unscramble_data(src[srcpos]);
}

// Card N occupies N * CARD_STRIDE + CARD_OFFSET in the descrambled PCM space.
constexpr size_t CARD_SIZE   = 0x80000;
constexpr size_t CARD_STRIDE = 0x100000;
constexpr size_t CARD_OFFSET = 0x80000;

// The sound chip fetches two bytes ahead of the address the CPU parks in the phase
// accumulator.  The U-110 firmware compensates for this itself at 0x7BB2
// ("SUB RW56,RW50,#0x2"), so the emulated read port has to put it back.  The CM-32P
// driver does the same thing.  If cards refuse to mount but internal banks read fine,
// try 0.
constexpr int PCM_READ_PREFETCH = 2;


/***************************************************************************************
    Output routing -- the fifty Output Modes

    Each mode partitions the 31 sounding voices across the six Multi Output jacks.  The
    firmware keeps the mode index (mode - 1) at RAM 0x280E and uses it to block-copy eight
    routing bytes from its own table into 0x1F00-0x1F07 (routine at 0xB721).

    Modes 21-50 put Multi Outputs 1 and 2 on one group with the effect switchable: 'M' is
    dry and centred, 'L/R' is the wet stereo pair.  The manual's own footnote.  Voices are
    numbered 1-31 -- voice 0 is the CPU's wave-ROM read port and never sounds.

    Transcribed from the scan and checked against the invariant that every row partitions
    exactly 31 voices; mode 10 was recovered from that constraint alone where the scan had
    lost its "8 8".  Mode 20 reproduces the six measured Wide Piano groups exactly.
***************************************************************************************/

struct output_mode
{
	uint8_t kind;        // 0 = plain, 1 = L/R pair on outputs 1+2, 2 = centred on outputs 1+2
	uint8_t sizes[6];
};

constexpr int NUM_OUTPUT_MODES = 50;
constexpr int NUM_OUTPUTS = 6;
constexpr int NUM_VOICE_SLOTS = 32;

constexpr output_mode OUTPUT_MODES[NUM_OUTPUT_MODES] =
{
	{ 0, { 31,  0,  0,  0,  0,  0 } },   //  1
	{ 0, { 27,  4,  0,  0,  0,  0 } },   //  2
	{ 0, { 23,  8,  0,  0,  0,  0 } },   //  3
	{ 0, { 23,  4,  4,  0,  0,  0 } },   //  4
	{ 0, { 19, 12,  0,  0,  0,  0 } },   //  5
	{ 0, { 19,  8,  4,  0,  0,  0 } },   //  6
	{ 0, { 19,  4,  4,  4,  0,  0 } },   //  7
	{ 0, { 15, 16,  0,  0,  0,  0 } },   //  8
	{ 0, { 15, 12,  4,  0,  0,  0 } },   //  9
	{ 0, { 15,  8,  8,  0,  0,  0 } },   // 10
	{ 0, { 15,  8,  4,  4,  0,  0 } },   // 11
	{ 0, { 15,  4,  4,  4,  4,  0 } },   // 12
	{ 0, { 11, 12,  8,  0,  0,  0 } },   // 13
	{ 0, { 11, 12,  4,  4,  0,  0 } },   // 14
	{ 0, { 11,  8,  8,  4,  0,  0 } },   // 15
	{ 0, { 11,  8,  4,  4,  4,  0 } },   // 16
	{ 0, { 11,  4,  4,  4,  4,  4 } },   // 17
	{ 0, {  7,  8,  8,  8,  0,  0 } },   // 18
	{ 0, {  7,  8,  8,  4,  4,  0 } },   // 19
	{ 0, {  7,  8,  4,  4,  4,  4 } },   // 20
	{ 1, { 31,  0,  0,  0,  0,  0 } },   // 21  L/R pair
	{ 2, { 31,  0,  0,  0,  0,  0 } },   // 22  centred
	{ 1, { 16, 15,  0,  0,  0,  0 } },   // 23  L/R pair
	{ 2, { 16, 15,  0,  0,  0,  0 } },   // 24  centred
	{ 1, { 16, 11,  4,  0,  0,  0 } },   // 25  L/R pair
	{ 2, { 16, 11,  4,  0,  0,  0 } },   // 26  centred
	{ 1, { 16,  7,  8,  0,  0,  0 } },   // 27  L/R pair
	{ 2, { 16,  7,  8,  0,  0,  0 } },   // 28  centred
	{ 1, { 16,  7,  4,  4,  0,  0 } },   // 29  L/R pair
	{ 2, { 16,  7,  4,  4,  0,  0 } },   // 30  centred
	{ 1, { 16,  3,  4,  4,  4,  0 } },   // 31  L/R pair
	{ 2, { 16,  3,  4,  4,  4,  0 } },   // 32  centred
	{ 1, {  8, 23,  0,  0,  0,  0 } },   // 33  L/R pair
	{ 2, {  8, 23,  0,  0,  0,  0 } },   // 34  centred
	{ 1, {  8, 19,  4,  0,  0,  0 } },   // 35  L/R pair
	{ 2, {  8, 19,  4,  0,  0,  0 } },   // 36  centred
	{ 1, {  8, 15,  8,  0,  0,  0 } },   // 37  L/R pair
	{ 2, {  8, 15,  8,  0,  0,  0 } },   // 38  centred
	{ 1, {  8, 15,  4,  4,  0,  0 } },   // 39  L/R pair
	{ 2, {  8, 15,  4,  4,  0,  0 } },   // 40  centred
	{ 1, {  8, 11, 12,  0,  0,  0 } },   // 41  L/R pair
	{ 2, {  8, 11, 12,  0,  0,  0 } },   // 42  centred
	{ 1, {  8, 11,  8,  4,  0,  0 } },   // 43  L/R pair
	{ 2, {  8, 11,  8,  4,  0,  0 } },   // 44  centred
	{ 1, {  8, 11,  4,  4,  4,  0 } },   // 45  L/R pair
	{ 2, {  8, 11,  4,  4,  4,  0 } },   // 46  centred
	{ 1, {  8,  7,  8,  8,  0,  0 } },   // 47  L/R pair
	{ 2, {  8,  7,  8,  8,  0,  0 } },   // 48  centred
	{ 1, {  8,  7,  8,  4,  4,  0 } },   // 49  L/R pair
	{ 2, {  8,  7,  8,  4,  4,  0 } },   // 50  centred
};

// Turn an Output Mode index (mode - 1) into a per-voice output bitmask.
//
// mask[v] has bit N set when voice v feeds Multi Output N+1.  Voice 0 is always silent --
// it is the CPU's wave-ROM read port.  Out-of-range indices leave the mask all zero,
// which is what the driver wants during boot before 0x280E has been written.
inline void voice_masks_for_mode(unsigned idx, uint8_t (&mask)[NUM_VOICE_SLOTS])
{
	for (int v = 0; v < NUM_VOICE_SLOTS; v ++)
		mask[v] = 0;
	if (idx >= NUM_OUTPUT_MODES)
		return;

	output_mode const &m = OUTPUT_MODES[idx];
	uint8_t voice = 1;                       // voice 0 is the CPU's wave-ROM read port
	for (int g = 0; g < NUM_OUTPUTS && m.sizes[g] != 0; g ++)
	{
		// In modes 21-50 the first group occupies Multi Outputs 1 and 2 together -- dry and
		// centred (M), or the wet stereo pair (L/R).  Later groups then start at output 3.
		uint8_t const gm = (g == 0 && m.kind != 0)
				? 0x03
				: uint8_t(1 << (m.kind != 0 ? g + 1 : g));
		for (int i = 0; i < m.sizes[g] && voice < NUM_VOICE_SLOTS; i ++)
			mask[voice ++] = gm;
	}
}


/***************************************************************************************
    Pan positions of the six Multi Outputs

    Measured on real hardware with service test 11, which steps the jacks in order
    ("Jack 1 2 3 4 5 6 VOICE- CHORUS TREMORO", firmware string at 0x08F18).  R-L came out
    +65.9, -65.9, 0.0, 0.0, +6.7, -6.8 dB, with L/R correlation 1.000 and zero delay --
    the multiplex routes, it does not time-shift.

    `[I]` The capture reads jack 1 as hard RIGHT, but Roland labels OUTPUT 1 as MIX L, so
    the recording is almost certainly channel-swapped at the interface; the table below is
    mirrored to the labelling.  Wide Piano then sweeps low-to-high left-to-right, which is
    what confirms the jack-to-position assignment either way.
***************************************************************************************/

constexpr double PAN_L[NUM_OUTPUTS] = { 1.000000, 0.000507, 0.707107, 0.707107, 0.907800, 0.415900 };
constexpr double PAN_R[NUM_OUTPUTS] = { 0.000507, 1.000000, 0.707107, 0.707107, 0.419800, 0.909900 };


/***************************************************************************************
    Reconstruction filter -- service notes Fig. 4

    Two unity-gain Sallen-Key lowpass sections built round IC30-35 (uPC4570) plus an
    output RC.  These are the schematic's component values; both hosts derive fc and Q
    from them rather than storing the derived numbers, so the two cannot drift.

        R39/R40 10k, C53 8200p, C52 680p  ->  6740 Hz, Q 1.74
        R41/R42 10k, C55 3300p, C54 560p  -> 11708 Hz, Q 1.21
        R78 10k, C56 2200p                ->  7234 Hz, one pole

    Fifth order overall; the service notes' own simulation gives max +2.17 dB, min
    -48.57 dB.

    The hardware has six of these chains, one per Multi Output.  They are identical and
    the filter is linear, so summing first and filtering the two mix buses is equivalent
    to filtering all six and then summing -- two chains instead of eighteen.

    `[!]` As modelled this cascade peaks +4.29 dB at 6065 Hz, twice the resonance the
    service notes' own simulation reports.  That discrepancy is unresolved and is the
    leading suspect for the measured HF excess the EQ below corrects.  See
    PLUGIN-PLAN.md section 10.2.
***************************************************************************************/

// Ohms and Farads, so either host can apply its own resistor/capacitor macros.
constexpr double SK1_R1 = 10e3,  SK1_R2 = 10e3,  SK1_C1 = 8200e-12, SK1_C2 = 680e-12;
constexpr double SK2_R1 = 10e3,  SK2_R2 = 10e3,  SK2_C1 = 3300e-12, SK2_C2 = 560e-12;
constexpr double RC_R   = 10e3,  RC_C   = 2200e-12;

// The Sallen-Key feedback divider: R3 absent, R4 shorted, i.e. unity gain.
constexpr double SK_R3 = 999.99e6;
constexpr double SK_R4 = 0.001;


/***************************************************************************************
    Output EQ correction -- MEASURED, not modelled

    Against the hardware captures in listen/hardware/3, level-matched over 200 Hz - 2 kHz,
    the emulator runs +7.0 dB hot at 6 kHz (analysis/hf_excess.pdf).  The excess is a
    broad bell: flat below 2 kHz, peaking at 6-6.5 kHz, back to zero by 10 kHz.  Seven
    patches agree within +5.9 to +8.9 dB, and rendering natively at 32 kHz reproduces it,
    so it is neither patch-specific nor a resampling artefact.

    One RBJ peaking cut cancels it to a weighted rms residual of 0.53 dB, taking the
    300 Hz - 10 kHz error across ten patches from 2.83 dB rms to 1.20 dB.

    `[!]` This is a CORRECTION LAYER, not the physics -- see the note on the Sallen-Key
    cascade above.  It is honest only while it stays labelled as compensation and its
    cause is still being chased.  When the chain is fixed this must be RE-MEASURED, not
    kept.

    Q is in MAME's K = tan(pi*fc/fs) convention; gain is multiplicative.
***************************************************************************************/

constexpr double EQ_FC   = 5933.3;    // Hz
constexpr double EQ_Q    = 1.9306;
constexpr double EQ_GAIN = 0.436787;  // -7.195 dB


/***************************************************************************************
    Output gain

    The U-110's own scaling is quiet -- a single max-velocity note on a sustained tone
    with part level 127 peaks around -16 dBFS even after the sound device's 12 dB
    calibration fix (VOICE_FS in roland_lp.h), because the firmware asks the chip for a
    sustain level of 0xDB, 13.5 dB below its full scale, to leave room for 31 voices.
    Real behaviour, but on a modern output it is quiet, so the gain is exposed rather
    than baked in.

    Measured worst cases at 0 dB (E. Organ 1, velocity 127, CC7 127, sustained cluster):

        1 note   -15.8 dBFS      8 notes   -4.5 dBFS
        2 notes  -10.9 dBFS     12 notes   -1.6 dBFS
        4 notes   -7.1 dBFS     16 notes   -0.9 dBFS

    so 0 dB is as loud as this can be without a dense cluster clipping the 16-bit output.
    Ordinary playing has headroom to spare; +6 dB is comfortable up to about eight notes.
***************************************************************************************/

constexpr int GAIN_STEPS   = 16;
constexpr int GAIN_MIN_DB  = -9;
constexpr int GAIN_STEP_DB = 3;
constexpr int GAIN_DEFAULT = 3;                 // index of 0 dB
constexpr int gain_db_for(int idx) { return GAIN_MIN_DB + GAIN_STEP_DB * idx; }


/***************************************************************************************
    MIDI

    31250 baud, 8N1.  MAME's midi_port is bit-level and this CPU core takes whole bytes,
    so the driver deserialises; a plugin host hands over bytes already and has to clock
    them in at this rate for the firmware's timing to be right.
***************************************************************************************/

constexpr int MIDI_BAUD = 31250;


/***************************************************************************************
    Firmware RAM addresses the host reads directly

    Reading these is version-independent.  Matching byte patterns against tables in
    program ROM would not be -- they sit at different addresses in v2.00 and v2.03.
***************************************************************************************/

constexpr uint16_t RAM_PATCH_EDIT_BUFFER = 0x2800;  // active patch, 128 bytes
constexpr uint16_t RAM_OUTPUT_MODE_INDEX = 0x280E;  // mode - 1
constexpr uint16_t RAM_PART_RECORDS      = 0x2814;
constexpr uint16_t RAM_CARD_IDS          = 0x2743;

} // namespace roland_u110

#endif // MAME_ROLAND_ROLAND_U110_DATA_H
