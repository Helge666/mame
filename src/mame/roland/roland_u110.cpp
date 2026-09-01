// license:BSD-3-Clause
// copyright-holders:Elliott H. Liggett
/*************************************************************************************************

    Roland U-110 PCM Sound Module (1989)

    The U-110 is built on the same PCM engine as the Roland CM-32P (see roland_cm32p.cpp,
    driver by Valley Bell): an MCS-96 CPU driving the MB87419 / MB87420 gate array pair.
    This driver is derived from that one.  Differences that matter:

                            U-110                       CM-32P
      CPU                   N8097BH, 16-bit bus         P8098, 8-bit bus
      Sound chip regs       0x1400 + 2*reg              0x1400 + reg
      Sound chip crystal    34.816 MHz  -> 34 kHz       32.768 MHz -> 32 kHz
      0xE000-0xFFFF         bank switched on P2.7       absent
      PCM card slots        four                        one
      DSP at 0x1080-0x10FF  absent                      present
      Panel                 16x2 LCD, own switch matrix different

    The wave ROM address/data line scrambling is identical to the CM-32P and to the
    SN-U110 cards; see the tables below.


    Register window addressing
    --------------------------
    The U-110's 8097BH reaches the sound chip over a 16-bit bus, so the chip's register
    select lines see CPU A1-A6 rather than A0-A5.  Every chip register therefore appears at
    twice its offset, and 0x1400-0x143F covers exactly the 32 registers of the device.

    Because MAME's mb87419_mb87420_device is byte-addressed (it was written for the
    CM-32P's 8-bit bus), the mapping falls out very cleanly if the window is handled with
    16-bit handlers: the word offset *is* the device register index for the low byte, and
    offset+1 for the high byte.

        CPU addr    word offset   device reg(s)   function
        0x1402      0x01          01              ROM / card data read port
        0x1404      0x02          02,03           ROM bank + loop mode
        0x1408      0x04          04,05           pitch, 2.14 fixed point
        0x140C      0x06          06,07           volume
        0x1410      0x08          08,09           sample start address, fraction
        0x1414      0x0A          0A,0B           sample start address, high word
        0x1418      0x0C          0C,0D           sample end address
        0x141C      0x0E          0E,0F           sample loop address
        0x1422      0x11          11              voice enable mask, voices 0-7
        0x142A      0x15          15              voice enable mask, voices 16-23
        0x143E      0x1F          1F              voice select (0-31)


    PCM address space
    -----------------
    The bank selector the firmware writes to 0x1404 has bits 12-13 = slot and bit 11 = card.
    Bit 10 is also live and is wave address bit 18, i.e. it selects the upper half of a
    512 KB ROM -- easy to miss, and the reason the CPU read port must wrap at 18 bits (the
    phase accumulator supplies bits 0-17 only).  The device forms its ROM address as
    (addr >> 14) | ((bank & 0x3C00) << 8), putting bank bits 10-13 on address bits 18-21.
    So the 4 MB region is laid out:

        0x000000  waverom0   internal bank 0        0x080000  card slot 1
        0x100000  waverom1   internal bank 1        0x180000  card slot 2
        0x200000  waverom2   internal bank 2        0x280000  card slot 3
        0x300000  waverom3   internal bank 3        0x380000  card slot 4

    Four internal banks plus four cards fill the region exactly, and this is the same decode
    the CM-32P uses.  No change to the sound device is needed to support four slots.


    Status
    ------
    Boots, runs and plays.  Verified against a real U-110 (MIDI driven, audio captured):

      - pitch tracks hardware within +/-2.5 cents across five octaves and four patches
      - panel navigation, LEDs, NVRAM persistence, all four card slots
      - MIDI in via a bit-level UART receiver; -min file.mid works

    Sample format: 8-BIT DELTAS, INTEGRATED BY THE CHIP.  Selected with
    set_pcm_mode(PCM_FLOAT8_DELTA) and set_delta_dc_hz(0.0); the device keeps its
    historical behaviour for the CM-32P.  The reasoning is at the set_pcm_mode() call in
    u110() -- integration explains the 6 dB/octave the ROM sits above the U-110's output,
    it welds a ping-pong loop (what reverses is the SLOPE, so the amplitude never jumps),
    and it removes any need for a separate de-emphasis filter.

    `[I]` An earlier reading of this file said "LINEAR 8-BIT TWO'S COMPLEMENT PCM ...
    selected here with set_pcm_mode(PCM_FLOAT8)", from correlating decodes against a
    hardware recording (sample 89: direct PCM r=0.76 against 0.26 differential).  That
    conclusion was superseded and the code has said PCM_FLOAT8_DELTA for some time; the
    correlation test could not tell the two apart, because a sine's derivative is a sine.
    The stale text is recorded here only because it cost an investigation: read the
    set_pcm_mode() call, not this summary, when the decode is in question.

    Engine rate: 34.816 MHz / 1088 = 32000 Hz.  The crystal is confirmed on the schematic
    (p.13, IC15 pins 79/80); it is the divisor that differs from the CM-32P, not the
    oscillator.  With MAME's /1024 the emulator ran +104 cents sharp, and
    1200*log2(34000/32000) = +104.96.  Set with set_rate_divider(1088).

    Four fixes were needed outside this driver:
      1. i8x9x.cpp gated the HSI.0 interrupt on IOC1 bit 1.  This firmware sets IOC1 = 0x21
         (bit 1 clear, EXTINT on the pin for the sound chip) while enabling INT_MASK bit 4
         for HSI.0 from IC8, so the two were mutually exclusive, the LCD text ring at 0x2700
         filled and the firmware spun at 0xD2F6.  Very likely the same wall the CM-32P driver
         works around with a ROM patch.
      2. roland_lp.cpp gained set_pcm_mode() and set_rate_divider(), both defaulting to the
         CM-32P behaviour.
      3. 34.816 MHz added to emu/xtal.cpp.
      4. emu/sound.cpp and emu/resampler.cpp ROUND and TPDF-DITHER the float-to-s16
         conversions instead of truncating them.  MAME mixes in float and meets 16-bit
         integers at every hand-off to the OSD and at -wavwrite; `int(x * 32768)` truncates
         toward zero, which doubles the dead zone around silence, leaves a DC bias and makes
         the quantisation error correlated with the signal.  Measured on a U-110 fade 75 dB
         down, that is 16 dB of quantisation junk that dither removes, for 4.7 dB of added
         broadband noise.  Digital black is passed through undithered, so silence stays
         silent.  The switch is `audio_dither_enabled()` in emu/sound.h; it defaults ON.
         This one is core MAME, not a device, so it affects every driver and it does NOT
         reach the plugin -- the plugin hands float to its host and has no s16 stage at all.

    KNOWN DEFECT
    ------------
    Excess harmonic distortion.  Against hardware on the same patch and note the emulator
    produces 8-19 dB more noise+distortion, with upper harmonics 7-27 dB too strong:

        VIB 1 note 60   harmonic:   2    3    4    5    6   10   11
                        hardware:  -8  -15  -20  -22  -33  -45  -46
                        emulator:  -1   -4   -5   -6  -17  -23  -19

    Ruled out: companding curves, missing reconstruction filter, chorus, keymap selection,
    stepped volume.  Leading suspect is voice allocation/release -- a single-part patch was
    seen with six voices enabled, and roland_lp.cpp has a standing TODO about how the
    firmware frees a voice.

    TODO
    ----
    - Voice release (see above); the most likely cause of the distortion defect.
    - Output demultiplexing.  One DAC is time-multiplexed across six outputs (IC47A~F analog
      switches, IC30-35 filters) and summed by IC38 to L/R.  The device currently sums all
      voices and writes both channels identically, so routing and panning do not exist.  The
      plan is six output streams in the device and a 6->2 fold-down here using the BU3905
      registers.  Blocked on finding the per-voice output assignment register: 0x00/0x01 and
      0x1A are ruled out; 0x10, 0x19, 0x1B, 0x1D remain.
    - Chorus/tremolo (patch header +0x0E block) is not modelled; hardware shows a clear
      detune doublet on chorused patches that the emulator does not reproduce.
    - The LCD controller part is unidentified and its CGROM undumped; the base MSM6222B is
      used with a synthesised font (see tools/make_lcd_cgrom.py), flagged BAD_DUMP.
    - MIDI OUT is serialised to a midi_port; MIDI THRU is passed through from MIDI IN,
      as the hardware does.
    - PORT1 bit 4 unexplained.

    References:
      analysis/ROM-ANALYSIS.md   - firmware structures, register protocol, card format
      analysis/SYSTEM-DESIGN.md  - board topology, address decode, wave subsystem
      Roland U-110 service notes - p.5 block diagram, p.6 IC8 decode, p.13-14 schematic

*************************************************************************************************/

#include "emu.h"

#include "bus/generic/carts.h"
#include "bus/midi/midi.h"
#include "bus/generic/slot.h"
#include "cpu/mcs96/i8x9x.h"
#include "machine/nvram.h"
#include "sound/roland_lp.h"
#include "video/msm6222b.h"

#include "roland_u110_data.h"

#include "emupal.h"
#include "screen.h"
#include "softlist_dev.h"
#include "machine/rescap.h"
#include "sound/flt_biquad.h"
#include "sound/flt_rc.h"
#include "speaker.h"
#include "sound.h"        // audio_dither_enabled(), for the null test

#include <array>
#include <bit>


namespace {

// The wave-ROM permutations, the fifty output modes, the measured pan positions, the
// reconstruction filter's component values and the fitted output EQ all live in
// roland_u110_data.h, because the Voltaire 110 plugin core needs exactly the same numbers
// and two copies would drift.  See PLUGIN-PLAN.md section 3.
using namespace roland_u110;



class roland_u110_state : public driver_device
{
public:
	roland_u110_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_pcm(*this, "pcm")
		, m_lpf_sk(*this, "lpf_sk%u", 1U)
		, m_lpf_rc(*this, "lpf_rc%u", 1U)
		, m_eq(*this, "eq%u", 1U)
		, m_lcd(*this, "lcd")
		, m_cards(*this, "cardslot%u", 1U)
		, m_mdout(*this, "mdout")
		, m_mdthru(*this, "mdthru")
		, m_pcmrom(*this, "pcm")
		, m_patchview(*this, "patchview")
		, m_sw(*this, "SW")
		, m_boot_keys(*this, "BOOTKEYS")
		, m_autotest(*this, "AUTOTEST")
		, m_eq_enable(*this, "EQ")
		, m_gain(*this, "GAIN")
		, m_led(*this, "led%u", 0U)
	{
	}

	void u110(machine_config &config) ATTR_COLD;
	void init_u110() ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	void mem_map(address_map &map) ATTR_COLD;

	void lcd_palette(palette_device &palette) const;
	u32 screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	// panel / display
	void lcd_ctrl_w(u8 data);
	void lcd_data_w(u8 data);
	void led_w(offs_t offset, u16 data, u16 mem_mask);
	u8 switch_r(offs_t offset);
	TIMER_CALLBACK_MEMBER(lcd_int_cb);

	// CPU ports
	u8 port1_r();
	void port1_w(u8 data);
	void port2_w(u8 data);
	u16 battery_r();

	// sound chip window
	// `[I]` The tone generator's envelope ramp generator and its interrupt.  See the
	// machine config for why this is false; flipping it switches snd_r between the device's
	// real level readback and the stand-in below, so the two stay consistent.
	static constexpr bool ENV_ENGINE = true;

	u16 voice_status() const;
	u16 snd_r(offs_t offset, u16 mem_mask);
	void snd_w(offs_t offset, u16 data, u16 mem_mask);
	u8 pcm_data_r();

	// output routing (BU3905)
	void out_ctrl_w(offs_t offset, u16 data, u16 mem_mask);
	void update_voice_routing();

	// MIDI
	void midi_tx_w(u16 data);
	void midi_tx_start();
	TIMER_CALLBACK_MEMBER(midi_tx_cb);
	void midi_rxd_w(int state);
	TIMER_CALLBACK_MEMBER(midi_rx_cb);

	template <unsigned Slot> DECLARE_DEVICE_IMAGE_LOAD_MEMBER(card_load);
	template <unsigned Slot> DECLARE_DEVICE_IMAGE_UNLOAD_MEMBER(card_unload);


	// Measured output EQ correction (see u110(), "Output EQ correction").
	void update_eq();

	void update_gain(bool announce);

public:
	DECLARE_INPUT_CHANGED_MEMBER(eq_changed);
	DECLARE_INPUT_CHANGED_MEMBER(gain_changed);
	DECLARE_INPUT_CHANGED_MEMBER(gain_key);

private:

	required_device<i8x9x_device> m_maincpu;
	required_device<mb87419_mb87420_device> m_pcm;
	required_device_array<filter_biquad_device, 4> m_lpf_sk;
	required_device_array<filter_rc_device, 2> m_lpf_rc;
	required_device_array<filter_biquad_device, 2> m_eq;
	required_device<msm6222b_device> m_lcd;
	required_device_array<generic_slot_device, 4> m_cards;
	required_device<midi_port_device> m_mdout;
	required_device<midi_port_device> m_mdthru;
	required_region_ptr<u8> m_pcmrom;
	memory_view m_patchview;
	required_ioport m_sw;
	optional_ioport m_boot_keys;
	optional_ioport m_autotest;
	optional_ioport m_eq_enable;
	optional_ioport m_gain;
	output_finder<2> m_led;

	emu_timer *m_lcd_timer = nullptr;
	emu_timer *m_midi_rx_timer = nullptr;
	int m_rxd = 1;
	int m_rx_bit = -1;
	u8 m_rx_data = 0;

	// MIDI OUT.  The CPU core hands over whole bytes, so the 31250 baud 8N1 stream is
	// generated here -- the mirror image of the receiver above.  m_tx_bit is -1 when the
	// line is idle, 0-7 while a data bit is pending, 8 for the stop bit.
	emu_timer *m_midi_tx_timer = nullptr;
	int m_tx_bit = -1;
	u8 m_tx_data = 0;
	u8 m_tx_fifo[16]{};
	u8 m_tx_head = 0, m_tx_tail = 0;
	bool m_trace_midi_tx = true;

	u8  m_snd_regs[0x20]{};     // shadow of what the CPU wrote, for the read port and tracing
	u8  m_stat_idx = 0;     // voice index written to sound-chip register 0x10 / 0x12
	attotime m_voice_quiet[32];
	u8  m_card_present = 0x0f;  // bit per slot, 1 = empty (PORT1 is active low)
	u8  m_port2 = 0;
	u16 m_leds = 0;
	bool m_trace_tg = true;
	bool m_trace_midi_rx = true;
	u32 m_scan = 0;                        // firmware panel scans since power-on (100 Hz)
	bool m_mode_announced = false;
};


/**************************************************************************************
    Display
**************************************************************************************/

void roland_u110_state::lcd_palette(palette_device &palette) const
{
	palette.set_pen_color(0, rgb_t(  0,   0,   0));
	palette.set_pen_color(1, rgb_t(  0, 255,   0));
}

// 16x2 character LCD.  render() hands back 80 character cells; in two-line mode the first
// line is cells 0..39 and the second 40..79, so line 2 starts 40*16 = 640 bytes in.
u32 roland_u110_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	u16 sy = 0;
	u8 const *const data = m_lcd->render();
	bitmap.fill(0);

	for (u8 y = 0; y < 2; y++)
	{
		for (u8 ra = 0; ra < 9; ra++)
		{
			u16 *p = &bitmap.pix(sy++);

			for (u16 x = 0; x < 16; x++)
			{
				u8 gfx = 0;
				if (ra < 8)
					gfx = data[x*16 + y*640 + ra];

				*p++ = BIT(gfx, 4);
				*p++ = BIT(gfx, 3);
				*p++ = BIT(gfx, 2);
				*p++ = BIT(gfx, 1);
				*p++ = BIT(gfx, 0);
				*p++ = 0;
			}
		}
	}
	return 0;
}

// IC8 raises "LCD INT" when an LCD transaction completes; the firmware's HSI.0 handler
// (vector 0x2008 -> 0x4032) uses it to drain its text queue.  MAME's LCD model is
// instantaneous and never reports busy, so the interrupt is synthesised here a short time
// after each access.  The delay is a guess in the right order of magnitude for an
// HD44780-class controller.
void roland_u110_state::lcd_ctrl_w(u8 data)
{
	m_lcd->control_w(data);
	logerror("LCD %f cmd %02X\n", machine().time().as_double(), data);
	m_lcd_timer->adjust(attotime::from_usec(40));
}

void roland_u110_state::lcd_data_w(u8 data)
{
	m_lcd->data_w(data);
	// The single most useful Milestone 1 diagnostic: what the firmware is trying to print.
	logerror("LCD %f chr %02X '%c'\n", machine().time().as_double(), data,
			(data >= 0x20 && data < 0x7f) ? char(data) : '.');
	m_lcd_timer->adjust(attotime::from_usec(40));
}

TIMER_CALLBACK_MEMBER(roland_u110_state::lcd_int_cb)
{
	m_maincpu->set_input_line(i8x9x_device::HSI0_LINE, ASSERT_LINE);
	m_maincpu->set_input_line(i8x9x_device::HSI0_LINE, CLEAR_LINE);
}

// Panel LED write.  IC5 (HC74 dual flip-flop) latches D0 and D1 on IC8's LED WR strobe
// (IC8 pin 72): D0 = PART/JUMP lamp, D1 = EDIT/EXIT lamp.  The firmware builds the value
// and inverts it before storing ("NOT RW30" then "ST RW30,0x1200"), so a clear bit lights
// the lamp.
void roland_u110_state::led_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_leds);
	m_led[0] = BIT(~m_leds, 0);     // PART / JUMP
	m_led[1] = BIT(~m_leds, 1);     // EDIT / EXIT
	logerror("%s: LED part=%d edit=%d (raw %04X)\n", machine().describe_context(),
			BIT(~m_leds, 0), BIT(~m_leds, 1), m_leds);
}

// Panel key masks, as the firmware sees them at 0x1300 (active low on the pin).
static constexpr u8 KEY_PART = 0x01, KEY_EDIT = 0x02, KEY_LEFT = 0x04,
                    KEY_RIGHT = 0x08, KEY_DEC = 0x10, KEY_INC = 0x20;
static constexpr u8 KEY_NEXT = KEY_DEC | KEY_INC;    // "[DEC]+[INC] = Next"

// The firmware scans the panel from its timer ISR at a steady 100 Hz, and does not read
// the port at all until about 3.5 s into boot.  Scripting keys in scans rather than in
// video frames therefore removes the race that makes frame-aligned presses unreliable:
// a two-key combination is only recognised when both keys are present in the SAME scan
// (see 0x8F69, which tests one key's press event against the other's held state).
static constexpr u32 SCAN_BOOT_HOLD = 40;    // hold the power-on combination this long
static constexpr u32 SCAN_LEAD_IN   = 150;   // let the banner settle
static constexpr u32 SCAN_PRESS     = 15;    // 150 ms - comfortably several scans
static constexpr u32 SCAN_GAP       = 400;   // 4 s: the S-RAM and wave-ROM tests take over a second

// Calibration note: "auto-navigate to test N" issues N [DEC]+[INC] presses, which reaches
// tests 1-8 reliably.  It does NOT get past 8 (DAC OFFSET ADJ) however many presses are
// issued, so tests 9-11 -- including 10 SOUND CHECK -- are not reachable this way yet.
// Test 3 (KEY&LED CHECK) also absorbs presses as key-test input rather than advancing,
// which is why the press count and the test number are not one-to-one above 3.

u8 roland_u110_state::switch_r(offs_t offset)
{
	// Six panel switches wired to IC8's READ0-READ5 (pins 64-69), presented as one byte.
	// There is no matrix: the firmware reads 0x1300 exactly once, at 0x4118.
	u8 forced = 0;

	// Announce the boot mode on the console, once, on the first scan.  BOOTKEYS and
	// AUTOTEST are CONFIG ports, so MAME writes whatever they were set to into
	// cfg/u110.cfg on exit and silently restores it next time.  A single run that
	// enabled a service test therefore leaves every later run stuck in the test menu,
	// where no MIDI note ever sounds -- easy to lose an hour to.  (This cannot go in
	// machine_start: input ports cannot be read at init time.)
	if (!m_mode_announced)
	{
		m_mode_announced = true;
		u8 const at = m_autotest.read_safe(0) & 0x0f;
		u8 const bk = m_boot_keys.read_safe(0) & 0x33;
		if (at != 0)
			osd_printf_info("U-110: *** SERVICE TEST MODE *** auto-navigating to test %d%s."
					"  No MIDI notes will sound.\n", at,
					BIT(m_autotest.read_safe(0), 4) ? ", sweeping its steps" : "");
		else if (bk != 0)
			osd_printf_info("U-110: *** power-on key combination 0x%02X held *** (%s)\n", bk,
					bk == 0x30 ? "service test menu" :
					bk == 0x03 ? "initialise memory" : "unknown");
		else
			osd_printf_info("U-110: normal boot (no service test).\n");
	}

	u8 const autotest = m_autotest.read_safe(0) & 0x0f;
	bool const sweep = BIT(m_autotest.read_safe(0), 4);

	if (autotest != 0)
	{
		// Auto-navigation implies the power-on combination that reaches the test menu.
		if (m_scan < SCAN_BOOT_HOLD)
		{
			forced = KEY_NEXT;
		}
		else
		{
			u32 const t = m_scan - SCAN_BOOT_HOLD;
			u32 const cycle = SCAN_PRESS + SCAN_GAP;
			if (t >= SCAN_LEAD_IN)
			{
				u32 const step = (t - SCAN_LEAD_IN) / cycle;
				bool const pressing = ((t - SCAN_LEAD_IN) % cycle) < SCAN_PRESS;
				if (pressing)
				{
					if (step < autotest)
						forced = KEY_NEXT;                  // advance one test
					else if (sweep && step < autotest + 40u)
						forced = KEY_RIGHT;                 // step within the test
				}
			}
		}
	}
	else if (m_boot_keys.read_safe(0) && m_scan < 500)
	{
		// Power-on key combinations.  The firmware samples the held-key mask once, and
		// branches on it: 0x30 (DEC+INC) enters the service test menu, 0x03 (PART+EDIT)
		// runs the memory initialise.  It does not read the port until roughly 3.5 s into
		// boot, so this is counted in scans rather than in wall-clock time.
		forced = m_boot_keys->read() & 0x33;
	}

	m_scan++;
	return m_sw->read() & ~forced;
}


/**************************************************************************************
    CPU ports
**************************************************************************************/

// PORT1 bits 0-3: cartridge presence, ACTIVE LOW.  The firmware writes 0xFF to PORT1 at
// 0x4378 (the quasi-bidirectional input idiom), so unpopulated lines read high.
u8 roland_u110_state::port1_r()
{
	return 0xf0 | m_card_present;
}

void roland_u110_state::port1_w(u8 data)
{
	// bit 4 is driven around card operations; purpose not established (open question #4)
	logerror("%s: PORT1 = %02X\n", machine().describe_context(), data);
}

// PORT2 bit 7 = BANK SELECT (IC8 pin 74), bit 6 = MIDI activity LED
void roland_u110_state::port2_w(u8 data)
{
	if (BIT(data ^ m_port2, 7))
	{
		// 1 = EPROM factory default patches at 0xE000, 0 = battery-backed user patch SRAM
		m_patchview.select(BIT(data, 7));
		logerror("%s: BANK SELECT = %d\n", machine().describe_context(), BIT(data, 7));
	}
	m_port2 = data;
}

// A/D channel 0 reads the backup battery, polled.  The check at 0x44D7 accepts
// AD_RESULT_HI strictly between 0x84 and 0xCC:
//
//      44E0  ldb  30, ad_result_hi
//      44E3  cmpb 30, #cb  /  jh 44ef      -> too high, flag bad
//      44E8  cmpb 30, #84  /  jnh 44ef     -> too low,  flag bad
//      44EF  orb  b8, #04                  -> "Check Battery!"
//
// The core builds ad_result as (callback() << 6), so AD_RESULT_HI is callback() >> 2.
// 0x2a0 >> 2 = 0xa8, comfortably mid-window.
u16 roland_u110_state::battery_r()
{
	return 0x2a0;
}


/**************************************************************************************
    Sound chip window at 0x1400-0x143F
**************************************************************************************/

// Reading 0x1404 (device registers 02/03) does NOT return the ROM bank that was written
// there: it is the selected voice's current envelope LEVEL, and which field of it comes
// back depends on whether 0x10, 0x12 or 0x16 named the voice.  Three places depend on it:
//
//   0x641A  note-on: read a voice's level; zero means the voice is FREE
//   0x7655  the envelope handler: all three fields, normalised to a log level
//   0xCC0F  power-up: set a voice's volume low, then spin until its level reaches zero
//
// The device answers all three when its envelope ramp generator is on, and snd_r forwards
// to it. With it off there is no level to report, so this stands in: a voice reads busy
// while it is enabled and goes idle a fixed 8 ms after a low volume is written, which is
// exactly what the release routine does before it starts waiting. It is enough to keep the
// firmware allocating and freeing voices, and no use at all for driving an envelope.
static constexpr attoseconds_t VOICE_FADE = ATTOSECONDS_PER_MILLISECOND * 8;

u16 roland_u110_state::voice_status() const
{
	u32 const on = u32(m_snd_regs[0x11]) | (u32(m_snd_regs[0x12]) << 8)
				 | (u32(m_snd_regs[0x15]) << 16) | (u32(m_snd_regs[0x16]) << 24);
	if (!BIT(on, m_stat_idx))
		return 0;                                   // not enabled: idle
	if (m_voice_quiet[m_stat_idx] != attotime::zero && machine().time() >= m_voice_quiet[m_stat_idx])
		return 0;                                   // release finished
	return 0x03ff;                                  // still sounding
}

u16 roland_u110_state::snd_r(offs_t offset, u16 mem_mask)
{
	u16 result = 0;

	// Registers 00 and 02/03 come from the tone generator itself once the envelope ramp
	// generator is running: 00 names the voice asking for its next envelope segment, and
	// 02/03 hand back that voice's current level.  Both are what the firmware's EXTINT
	// handler at 0x41C4 runs on -- see the note above snd_r.
	if (ACCESSING_BITS_0_7)
	{
		if (offset == 0x01)
			result |= pcm_data_r();
		else if (offset == 0x00 && ENV_ENGINE)
		{
			u8 const v = m_pcm->read(0x00);
			// Pairs with the ENVACK/ENVARR trace in roland_lp.cpp -- same U110_ENVTRACE
			// switch, but from this side the CPU's PC comes with it, which is how the
			// spurious third interrupt was pinned to the handler entry at 0x41D5.
			static bool const env_trace = getenv("U110_ENVTRACE") != nullptr;
			if (env_trace)
				logerror("ENVRD %f -> v%02X  @%s\n", machine().time().as_double(), v,
						machine().describe_context());
			return v;   // NOT offset+1: register 01 is the ROM read port
		}
		else if (offset == 0x02)
			return ENV_ENGINE ? (u16(m_pcm->read(0x02)) | (u16(m_pcm->read(0x03)) << 8))
							  : voice_status();
		else
			result |= m_snd_regs[offset];
	}
	if (ACCESSING_BITS_8_15 && offset < 0x1f)
		result |= u16(m_snd_regs[offset + 1]) << 8;

	return result;
}

void roland_u110_state::snd_w(offs_t offset, u16 data, u16 mem_mask)
{
	if (ACCESSING_BITS_0_7)
	{
		m_snd_regs[offset] = data & 0xff;
		if (offset == 0x10 || offset == 0x12)
			m_stat_idx = data & 0x1f;           // which voice voice_status() refers to
		m_pcm->write(offset, data & 0xff);
		if (m_trace_tg)
			logerror("TG %f v%02X reg %02X = %02X  @%s\n",
					machine().time().as_double(), m_snd_regs[0x1f] & 0x1f, offset, data & 0xff,
					machine().describe_context());
	}
	// a low volume written to the selected voice starts its release (voice_status() only)
	if (offset == 0x06 && ACCESSING_BITS_8_15 && (data & 0xff00) == 0 && (data & 0xff) < 0x40)
		m_voice_quiet[m_snd_regs[0x1f] & 0x1f] = machine().time() + attotime(0, VOICE_FADE);

	// The status selects are byte registers that happen to be written with a 16-bit store.
	// 0x142C in particular is `st 54, 142c` in the envelope handler, and forwarding its
	// high half by the rule below would put it in register 0x17 -- the voice-enable group
	// for voices 24-31 -- silently switching eight voices off on every status select.  That
	// is what made choir3_pingpong, shakuhachi, drums and the Wide Piano segments retrigger
	// their notes: only the segments whose voices landed in 24-31 were affected.
	if (offset == 0x10 || offset == 0x12 || offset == 0x16 || offset == 0x1a)
		return;

	if (ACCESSING_BITS_8_15 && offset < 0x1f)
	{
		// Voice enable is 16 bits wide on this bus: the firmware covers voices 0-15 with one
		// store at 0x1422 and 16-31 with one at 0x142A.  MAME's device decodes the four
		// 8-voice groups at byte offsets 0x11/0x13/0x15/0x17 (the CM-32P reaches them one
		// byte at a time over its 8-bit bus), and its own ((offset >> 1) & 3) grouping pairs
		// 0x12 with 0x13 and 0x16 with 0x17.  So the high half is routed to the odd offset.
		// Sending it to 0x12/0x16 instead would fall through the device's switch and voices
		// 8-15 and 24-31 would never sound.  Done here rather than by widening the device's
		// case list, because the CM-32P writes 0x12 for something else entirely.
		offs_t const hi = (offset == 0x11 || offset == 0x15) ? offset + 2 : offset + 1;
		m_snd_regs[offset + 1] = data >> 8;
		m_pcm->write(hi, data >> 8);
		if (m_trace_tg)
			logerror("TG %f v%02X reg %02X = %02X\n",
					machine().time().as_double(), m_snd_regs[0x1f] & 0x1f, offset + 1, data >> 8);
	}
}

// The CPU has no bus of its own to the wave ROMs or cards.  It borrows voice 0 of the tone
// generator as a read engine: park that voice's phase accumulator at the wanted address,
// busy-wait, then read the byte back from register 0x01.  See ROM-ANALYSIS.md 6.1.
u8 roland_u110_state::pcm_data_r()
{
	u32 const phase = m_snd_regs[0x08] | (m_snd_regs[0x09] << 8)
					| (m_snd_regs[0x0a] << 16) | (m_snd_regs[0x0b] << 24);
	u16 const bank  = m_snd_regs[0x02] | (m_snd_regs[0x03] << 8);

	// The address must wrap within the accumulator before the bank is applied.  The
	// firmware asks for logical address 0 by parking at 0 - 2, which sign-extends through
	// its SHRAL to phase 0xFFFF8000; only an 18-bit wrap brings (phase >> 14) + 2 back to
	// 0.  Without it the header read lands a bank away and every card reports
	// "  Illegal CARD".  The CM-32P driver masks identically.
	offs_t addr = (((phase >> 14) + PCM_READ_PREFETCH) & 0x3ffff) | ((bank & 0x3c00) << 8);

	return m_pcmrom[addr & 0x3fffff];
}



/**************************************************************************************
    Output Modes - Owner's Manual p.27

    A mode partitions the 31 voices into Voice Groups; a Part's Output Assign (patch part
    byte +0x0B >> 5) picks a group, and group N drives Multi Output N.

    The table and the partitioning rule are in roland_u110_data.h, along with how they
    were recovered.  Only the part that touches the emulated machine is here.
**************************************************************************************/

/**************************************************************************************
    Output routing - IC26/IC6 BU3905S at 0x1F00-0x1F08
**************************************************************************************/

void roland_u110_state::out_ctrl_w(offs_t offset, u16 data, u16 mem_mask)
{
	// Byte-wide device on a 16-bit bus, so both lanes are live and the byte address is
	// offset*2 / offset*2+1.
	if (ACCESSING_BITS_0_7)
		logerror("%s: OUT reg %X = %02X\n", machine().describe_context(), offset * 2, data & 0xff);
	if (ACCESSING_BITS_8_15)
		logerror("%s: OUT reg %X = %02X\n", machine().describe_context(), offset * 2 + 1, data >> 8);
	update_voice_routing();
}


// Turn the current Output Mode into a per-voice output mask for the sound device.
//
// The firmware keeps the mode index (mode - 1) at RAM 0x280E and uses it to block-copy
// eight routing bytes into 0x1F00-0x1F07 (routine at 0xB721).  Reading that index is
// version-independent; matching the eight bytes against a table in program ROM would not
// be, since the table sits at a different address in v2.00 and v2.03.
void roland_u110_state::update_voice_routing()
{
	u8 const idx = m_maincpu->space(AS_PROGRAM).read_byte(RAM_OUTPUT_MODE_INDEX);

	u8 mask[NUM_VOICE_SLOTS];
	voice_masks_for_mode(idx, mask);
	for (int v = 0; v < NUM_VOICE_SLOTS; v ++)
		m_pcm->set_voice_mask(v, mask[v]);

	if (m_trace_tg && idx < NUM_OUTPUT_MODES)
	{
		output_mode const &m = OUTPUT_MODES[idx];
		logerror("ROUTING idx=%u kind=%u sizes=%u,%u,%u,%u,%u,%u masks v1=%02X v8=%02X v16=%02X v28=%02X\n",
			idx, m.kind, m.sizes[0], m.sizes[1], m.sizes[2], m.sizes[3], m.sizes[4], m.sizes[5],
			mask[1], mask[8], mask[16], mask[28]);
	}
}

/**************************************************************************************
    MIDI
**************************************************************************************/

// The CPU core delivers whole bytes through serial_tx_cb, so the 31250 baud 8N1 stream
// has to be generated here -- the mirror image of the receiver below.
//
// A FIFO sits between the two because the CPU's transmit model and the real bit clock are
// independent: i8x9x_device::serial_send() arms its "sent" callback 9600 cycles after the
// write, which at this clock is slower than the 320 us a byte actually takes on the wire,
// so in practice the queue never holds more than one byte.  It is here so that if the
// firmware ever does write SBUF back-to-back the bytes queue instead of being silently
// overwritten mid-shift, which would corrupt a bulk dump in a way that is very hard to see.
void roland_u110_state::midi_tx_w(u16 data)
{
	u8 const byte = u8(data & 0xff);
	if (m_trace_midi_tx)
		logerror("MIDI OUT %02X\n", byte);

	u8 const next = (m_tx_head + 1) % u8(std::size(m_tx_fifo));
	if (next == m_tx_tail)
	{
		logerror("MIDI OUT overrun: byte %02X dropped\n", byte);
		return;
	}
	m_tx_fifo[m_tx_head] = byte;
	m_tx_head = next;

	if (m_tx_bit < 0)
		midi_tx_start();
}

// Begin the next queued byte, or leave the line idle high if there is none.
void roland_u110_state::midi_tx_start()
{
	if (m_tx_head == m_tx_tail)
		return;

	m_tx_data = m_tx_fifo[m_tx_tail];
	m_tx_tail = (m_tx_tail + 1) % u8(std::size(m_tx_fifo));
	m_tx_bit = 0;
	m_mdout->write_txd(0);                  // start bit
	m_midi_tx_timer->adjust(attotime::from_ticks(1, MIDI_BAUD));
}

TIMER_CALLBACK_MEMBER(roland_u110_state::midi_tx_cb)
{
	if (m_tx_bit < 8)
	{
		m_mdout->write_txd(BIT(m_tx_data, m_tx_bit));   // LSB first, as the receiver expects
		m_tx_bit ++;
		m_midi_tx_timer->adjust(attotime::from_ticks(1, MIDI_BAUD));
	}
	else if (m_tx_bit == 8)
	{
		m_mdout->write_txd(1);              // stop bit
		m_tx_bit ++;
		m_midi_tx_timer->adjust(attotime::from_ticks(1, MIDI_BAUD));
	}
	else
	{
		m_tx_bit = -1;                      // idle high; start the next byte if one is queued
		midi_tx_start();
	}
}

// MAME's midi_port is bit-level, but this CPU core takes whole bytes through serial_w(),
// so the 31250 baud 8N1 stream is deserialised here.  A falling edge while idle is a start
// bit; sample 1.5 bit times later, then once per bit time.
void roland_u110_state::midi_rxd_w(int state)
{
	// MIDI THRU is a wire, not a feature: the service manual shows JK3 driven straight from
	// IC2's output, the same node that feeds the CPU's RXD.  No CPU involvement, no
	// deserialising, no buffering -- so the line state is simply passed along, bit for bit,
	// and THRU keeps working even while the firmware is busy or halted.  (Owner's manual:
	// "the MIDI messages fed into the MIDI IN connector are output through the MIDI THRU
	// connector".)
	m_mdthru->write_txd(state);

	if (m_rx_bit < 0 && m_rxd && !state)
	{
		m_rx_bit = 0;
		m_rx_data = 0;
		m_midi_rx_timer->adjust(attotime::from_ticks(3, 2 * MIDI_BAUD));
	}
	m_rxd = state;
}

TIMER_CALLBACK_MEMBER(roland_u110_state::midi_rx_cb)
{
	if (m_rx_bit < 8)
	{
		m_rx_data |= m_rxd << m_rx_bit;
		m_rx_bit++;
		m_midi_rx_timer->adjust(attotime::from_ticks(1, MIDI_BAUD));
	}
	else
	{
		m_maincpu->serial_w(m_rx_data);     // stop bit reached; hand the byte over
		if (m_trace_midi_rx)
			// The TIME matters as much as the byte: the null test replays these into the
			// plugin core at the same emulated instants, so that a difference in the
			// rendered audio is the emulation and not the MIDI transport.
			logerror("MIDI IN %f %02X\n", machine().time().as_double(), m_rx_data);
		m_rx_bit = -1;
	}
}


/**************************************************************************************
    PCM cards
**************************************************************************************/

template <unsigned Slot>
DEVICE_IMAGE_LOAD_MEMBER(roland_u110_state::card_load)
{
	u32 const size = m_cards[Slot]->common_get_size("rom");
	if (size > 0x80000)
		return std::make_pair(image_error::INVALIDLENGTH, "Invalid size (maximum supported is 512K)");

	m_cards[Slot]->rom_alloc(0x80000, GENERIC_ROM8_WIDTH, ENDIANNESS_LITTLE);
	m_cards[Slot]->common_load_rom(m_cards[Slot]->get_rom_base(), size, "rom");

	u8 *const base = m_cards[Slot]->get_rom_base();
	if (size < 0x80000)
	{
		// Undersized dumps are mirrored up.  Address descrambling means this can only be
		// done in 128K pages.
		u32 mirror = std::bit_floor(size);
		if (mirror < 0x20000)
			mirror = 0x20000;
		for (u32 ofs = mirror; ofs < 0x80000; ofs += mirror)
			memcpy(base + ofs, base, mirror);
	}

	// card N lives at N*0x100000 + 0x080000 in the descrambled PCM space
	descramble_pcm(&m_pcmrom[Slot * CARD_STRIDE + CARD_OFFSET], base, CARD_SIZE);
	m_card_present &= ~(1 << Slot);

	return std::make_pair(std::error_condition(), std::string());
}

template <unsigned Slot>
DEVICE_IMAGE_UNLOAD_MEMBER(roland_u110_state::card_unload)
{
	memset(&m_pcmrom[Slot * 0x100000 + 0x80000], 0xff, 0x80000);
	m_card_present |= 1 << Slot;
}


/**************************************************************************************
    Address map
**************************************************************************************/

void roland_u110_state::mem_map(address_map &map)
{
	// The 8097BH has a 16-bit bus but every memory device in the machine is byte-wide;
	// IC8 generates BUSWIDTH per cycle so memory cycles run 8-bit and sound chip cycles
	// run 16-bit.  MAME reads a mapped region linearly regardless of space width, so the
	// byte-wide devices need no special handling here - only the timing differs.
	map(0x0000, 0x0fff).rom().region("maincpu", 0x0000);    // decoded, blank in both images
	map(0x1100, 0x1101).w(FUNC(roland_u110_state::lcd_ctrl_w)).umask16(0x00ff);
	map(0x1102, 0x1103).w(FUNC(roland_u110_state::lcd_data_w)).umask16(0x00ff);
	map(0x1200, 0x1201).w(FUNC(roland_u110_state::led_w));
	map(0x1300, 0x1301).r(FUNC(roland_u110_state::switch_r)).umask16(0x00ff);
	map(0x1400, 0x143f).rw(FUNC(roland_u110_state::snd_r), FUNC(roland_u110_state::snd_w));
	map(0x1f00, 0x1f0f).w(FUNC(roland_u110_state::out_ctrl_w));
	map(0x2000, 0x20ff).rom().region("maincpu", 0x2000);    // vectors, CCB, reset stub @ 0x2080
	// IC10.  The service notes list both SRAMs as battery-backed, and the firmware
	// keeps its global/system settings at 0x3C00 there, so this has to persist too or
	// every boot re-runs "Mem Initialized".
	map(0x2100, 0x3fff).ram().share("workram");             // IC10 work / back-up RAM
	map(0x4000, 0xdfff).rom().region("maincpu", 0x4000);    // program code

	// 0xE000-0xFFFF is bank switched on P2.7 (SYSTEM-DESIGN.md 3).  View 0 is the normal
	// case - IC11, the battery-backed user patch store.  View 1 exposes the EPROM's factory
	// default patch set, and the firmware selects it in exactly one place: the
	// "Mem Initialized" copy loop at 0x8475.
	map(0xe000, 0xffff).view(m_patchview);
	m_patchview[0](0xe000, 0xffff).ram().share("patchram");
	m_patchview[1](0xe000, 0xffff).rom().region("maincpu", 0xe000);
}


/**************************************************************************************
    Inputs
**************************************************************************************/

static INPUT_PORTS_START(u110)
	// Panel switches on IC8 READ0-READ5 (pins 64, 65, 66, 67, 68, 69).  Each is pulled up
	// and grounded when pressed, so ACTIVE_LOW.  The firmware's edge detector at 0x4118
	// stores ~raw as "previously pressed", which confirms that sense.
	PORT_START("SW")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_BUTTON1) PORT_NAME("Part / Jump")  PORT_CODE(KEYCODE_1)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_BUTTON2) PORT_NAME("Edit / Exit")  PORT_CODE(KEYCODE_2)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_BUTTON3) PORT_NAME("Left")         PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_BUTTON4) PORT_NAME("Right")        PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_BUTTON5) PORT_NAME("Dec")          PORT_CODE(KEYCODE_DOWN)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_BUTTON6) PORT_NAME("Inc / Enter")  PORT_CODE(KEYCODE_UP)
	PORT_BIT(0xc0, IP_ACTIVE_LOW, IPT_UNUSED)

	// The firmware reads the held-key mask once, a few milliseconds into reset, and uses it
	// to select a service mode.  Too early to press by hand, so it is a configuration here.
	PORT_START("BOOTKEYS")
	PORT_CONFNAME(0x33, 0x00, "Power-on key combination")
	PORT_CONFSETTING(   0x00, DEF_STR(None))
	PORT_CONFSETTING(   0x30, "DEC+INC (service test menu)")
	PORT_CONFSETTING(   0x03, "PART+EDIT (initialise memory)")

	// Drives the panel from switch_r itself, synchronised to the firmware's 100 Hz scan.
	// Frame-aligned input cannot reliably produce a two-key combination, because the key
	// routine at 0x8F69 only reports one when both keys appear in the same scan.
	PORT_START("AUTOTEST")
	PORT_CONFNAME(0x0f, 0x00, "Auto-navigate to service test")
	PORT_CONFSETTING(   0x00, DEF_STR(Off))
	PORT_CONFSETTING(   0x01, "1  S-RAM check")
	PORT_CONFSETTING(   0x02, "2  LCD check")
	PORT_CONFSETTING(   0x03, "3  Key & LED check")
	PORT_CONFSETTING(   0x04, "4  Battery check")
	PORT_CONFSETTING(   0x05, "5  MIDI check")
	PORT_CONFSETTING(   0x06, "6  Wave ROM check")
	PORT_CONFSETTING(   0x07, "7  ROM card check")
	PORT_CONFSETTING(   0x08, "8  DAC offset adjust")
	PORT_CONFSETTING(   0x09, "9  DAC MSB check")
	PORT_CONFSETTING(   0x0a, "10 Sound check")
	PORT_CONFSETTING(   0x0b, "11 Output check")
	PORT_CONFNAME(0x10, 0x00, "Sweep steps within that test")
	PORT_CONFSETTING(   0x00, DEF_STR(Off))
	PORT_CONFSETTING(   0x10, DEF_STR(On))

	// Switchable while the driver runs, from Tab -> Machine Configuration.  A gain of 1.0
	// makes MAME's PEAK biquad an exact pass-through (its numerator and denominator become
	// identical), so "Off" is a true bypass rather than a filter tuned flat.
	PORT_START("EQ")
	PORT_CONFNAME(0x01, 0x01, "Output EQ correction")
			PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_u110_state::eq_changed), 0)
	PORT_CONFSETTING(   0x00, DEF_STR(Off))
	PORT_CONFSETTING(   0x01, DEF_STR(On))

	// Output gain.  Adjustable three ways, all of which move the SAME setting, so the
	// menu always shows what the keys did and MAME persists it to cfg/u110.cfg:
	//
	//   * Tab -> Machine Configuration -> "Output gain";
	//   * the "Volume Down" / "Volume Up" inputs, `-` and `=` by default and remappable
	//     under Tab -> Input Settings, which is the one that works while live MIDI is
	//     playing;
	//   * mb87419_mb87420_device::set_mix_gain_db(), for a host that has no MAME UI.
	//
	// The gain is applied inside the sound device at the point a voice joins the mix, so
	// nothing downstream is being asked to amplify an already-quiet signal.
	PORT_START("GAIN")
	PORT_CONFNAME(0x0f, GAIN_DEFAULT, "Output gain")
			PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_u110_state::gain_changed), 0)
	PORT_CONFSETTING(   0x00, "-9 dB")
	PORT_CONFSETTING(   0x01, "-6 dB")
	PORT_CONFSETTING(   0x02, "-3 dB")
	PORT_CONFSETTING(   0x03, "+0 dB")
	PORT_CONFSETTING(   0x04, "+3 dB")
	PORT_CONFSETTING(   0x05, "+6 dB")
	PORT_CONFSETTING(   0x06, "+9 dB")
	PORT_CONFSETTING(   0x07, "+12 dB")
	PORT_CONFSETTING(   0x08, "+15 dB")
	PORT_CONFSETTING(   0x09, "+18 dB")
	PORT_CONFSETTING(   0x0a, "+21 dB")
	PORT_CONFSETTING(   0x0b, "+24 dB")
	PORT_CONFSETTING(   0x0c, "+27 dB")
	PORT_CONFSETTING(   0x0d, "+30 dB")
	PORT_CONFSETTING(   0x0e, "+33 dB")
	PORT_CONFSETTING(   0x0f, "+36 dB")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_VOLUME_DOWN) PORT_NAME("Output gain -3 dB")
			PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_u110_state::gain_key), 0)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_VOLUME_UP)   PORT_NAME("Output gain +3 dB")
			PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_u110_state::gain_key), 1)
INPUT_PORTS_END


/**************************************************************************************
    Machine
**************************************************************************************/

void roland_u110_state::machine_start()
{
	// U110_DITHER=0 turns off the TPDF dither at the 16-bit output.  The null test against
	// the plugin core needs it off at BOTH ends: dither is deterministic here (a seeded
	// xorshift32, which is why two MAME runs match), but the core cannot reproduce MAME's
	// exact call sequence into it, and there is no reason it should have to.  See
	// emu/sound.h and PLUGIN-PLAN.md section 4.
	if (char const *const d = getenv("U110_DITHER"))
		audio_dither_enabled() = (atoi(d) != 0);

	m_lcd_timer = timer_alloc(FUNC(roland_u110_state::lcd_int_cb), this);
	m_midi_rx_timer = timer_alloc(FUNC(roland_u110_state::midi_rx_cb), this);
	m_midi_tx_timer = timer_alloc(FUNC(roland_u110_state::midi_tx_cb), this);

	save_item(NAME(m_snd_regs));
	save_item(NAME(m_card_present));
	save_item(NAME(m_port2));
	save_item(NAME(m_leds));
	save_item(NAME(m_tx_bit));
	save_item(NAME(m_tx_data));
	save_item(NAME(m_tx_fifo));
	save_item(NAME(m_tx_head));
	save_item(NAME(m_tx_tail));
}

void roland_u110_state::update_eq()
{
	// U110_EQ=0 forces the correction off without touching cfg, for A/B renders.  The
	// front-panel switch under Machine Configuration still works when it is unset.
	static char const *const env_eq = getenv("U110_EQ");
	bool const on = env_eq ? (atoi(env_eq) != 0)
						   : (!m_eq_enable || BIT(m_eq_enable->read(), 0));
	for (int ch = 0; ch < 2; ch ++)
		m_eq[ch]->modify(filter_biquad_device::biquad_type::PEAK,
				EQ_FC, EQ_Q, on ? EQ_GAIN : 1.0);
}

INPUT_CHANGED_MEMBER(roland_u110_state::eq_changed)
{
	update_eq();
}

void roland_u110_state::update_gain(bool announce)
{
	int const idx = m_gain ? int(m_gain->read() & 0x0f) : GAIN_DEFAULT;
	int const db  = gain_db_for(idx);
	m_pcm->set_mix_gain_db(double(db));
	if (announce)
		popmessage("Output gain %+d dB", db);
}

INPUT_CHANGED_MEMBER(roland_u110_state::gain_changed)
{
	update_gain(false);       // the menu already shows the new setting
}

// One step of the same configuration field the menu edits, so the two never disagree and
// the choice is saved with the rest of the configuration.  select_next_setting() would be
// shorter but it WRAPS, which on a gain control means one keypress too many drops you from
// +36 dB to -9 dB; this clamps instead.
INPUT_CHANGED_MEMBER(roland_u110_state::gain_key)
{
	if (!newval || !m_gain)
		return;
	ioport_field *const f = m_gain->field(0x0f);
	if (f == nullptr)
		return;

	ioport_field::user_settings settings;
	f->get_user_settings(settings);
	int const idx = std::clamp(int(settings.value) + (param ? +1 : -1), 0, GAIN_STEPS - 1);
	if (ioport_value(idx) != settings.value)
	{
		settings.value = ioport_value(idx);
		f->set_user_settings(settings);
	}
	update_gain(true);
}

void roland_u110_state::machine_reset()
{
	// MIDI OUT idles high; nothing has been sent yet.
	m_mdout->write_txd(1);
	m_tx_bit = -1;
	m_tx_head = m_tx_tail = 0;

	update_eq();       // a saved configuration may have switched it off
	update_gain(false);   // ...and may carry a saved output gain
	m_patchview.select(0);
	m_port2 = 0;
	std::fill(std::begin(m_snd_regs), std::end(m_snd_regs), 0);
}

void roland_u110_state::init_u110()
{
	// Descramble the four internal wave ROMs into the banks the sound chip addresses.
	// Cards are descrambled on load into the gaps between them.
	u8 const *const src = memregion("pcmorg")->base();
	for (int bank = 0; bank < 4; bank++)
		descramble_pcm(&m_pcmrom[bank * CARD_STRIDE], &src[bank * CARD_SIZE], CARD_SIZE);
}

void roland_u110_state::u110(machine_config &config)
{
	N8097BH(config, m_maincpu, 12_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &roland_u110_state::mem_map);
	m_maincpu->in_p1_cb().set(FUNC(roland_u110_state::port1_r));
	m_maincpu->out_p1_cb().set(FUNC(roland_u110_state::port1_w));
	m_maincpu->out_p2_cb().set(FUNC(roland_u110_state::port2_w));
	m_maincpu->ach0_cb().set(FUNC(roland_u110_state::battery_r));
	m_maincpu->serial_tx_cb().set(FUNC(roland_u110_state::midi_tx_w));

	// Battery-backed SRAM: IC10 (work/back-up) and IC11 (user patches).
	// nvram_device adopts the memory share with a matching tag.
	NVRAM(config, "workram", nvram_device::DEFAULT_ALL_0);
	NVRAM(config, "patchram", nvram_device::DEFAULT_ALL_0);

	// 16x2 LCD.  MSM6222B is what the CM-32P and D-110 use; if the U-110's character set
	// or busy behaviour turns out to differ, hd44780 is the fallback.
	screen_device &screen(SCREEN(config, "screen").set_lcd());
	screen.set_refresh_hz(50);
	screen.set_screen_update(FUNC(roland_u110_state::screen_update));
	screen.set_size(16*6-1, (16*6-1)*3/4);
	screen.set_visarea(0, 16*6-2, 0, (16*6-1)*3/4-1);
	screen.set_palette("palette");
	PALETTE(config, "palette", FUNC(roland_u110_state::lcd_palette), 2);
	// The U-110's actual LCD controller part is not established, and the character
	// generator ROM of the MSM6222B-01 is not dumped here, so the base MSM6222B is used:
	// it takes an optional CGROM and renders blank glyphs without one.  The firmware's
	// character stream is logged by lcd_data_w regardless, which is what Milestone 1 needs.
	MSM6222B(config, m_lcd, 0);

	SPEAKER(config, "speaker", 2).front();

	// IC15's own crystal, X2 = 34.816 MHz (confirmed on the schematic, p.13, at IC15
	// pins 79/80).  The engine rate is 34.816 MHz / 1088 = 32000 Hz exactly, measured
	// against real hardware: at MAME's default /1024 the emulator ran +105 cents sharp,
	// and 1200*log2(34000/32000) = +104.96.  This is the divisor the original analysis
	// derived before it was 'corrected' to MAME's CM-32P formula.
	MB87419_MB87420(config, m_pcm, 34.816_MHz_XTAL);
	m_pcm->int_callback().set_inputline(m_maincpu, i8x9x_device::EXTINT_LINE);
	// The U-110's wave data is plain two's complement 8-bit PCM, established by comparing
	// decodes against the circulating ROM-dump WAVs: sample 89 matches direct PCM at
	// r=0.76 versus 0.26 for the differential reading MAME uses for the CM-32P.
	// The wave ROM byte is a DELTA and the chip integrates it.  A sine's derivative is a
	// sine, so the Sound Check tone could never have revealed this -- but it explains the
	// 6 dB/octave the ROM sits above the U-110's output, and it makes a ping-pong loop
	// weld itself: on the reverse pass the delta's sign flips, the accumulator retraces
	// the waveform exactly, and the amplitude never jumps because what reverses is the
	// SLOPE.  No de-emphasis filter is needed -- the integration is the de-emphasis.
	m_pcm->set_pcm_mode(mb87419_mb87420_device::PCM_FLOAT8_DELTA);
	m_pcm->set_delta_dc_hz(0.0);            // a true integrator; the loops are DC balanced
	m_pcm->set_rate_divider(1088);
	m_pcm->set_output_count(6);
	// The envelope is the firmware's, not ours: the chip ramps a level and interrupts when
	// it arrives, and the handler at 0x41C4 writes the next segment.  Everything the
	// U-110's own parameters reach -- ENV ATTACK RATE, ENV RELEASE RATE, key scaling, the
	// per-tone decay shapes -- comes along with it, because it is the real code running.
	// Calibrated against listen/hardware/env and listen/hardware/env2; see listen/hardware/env/ANALYSIS.md.
	// ON.  The chip ramps a level and interrupts when it arrives; the firmware's handler at
	// 0x41C4 writes the next segment.  Everything the instrument's own parameters reach --
	// ENV ATTACK RATE, ENV RELEASE RATE, key scaling, the per-tone decay shapes -- comes
	// along with it, because it is the real code running.
	//
	// Measured against listen/hardware/env and listen/hardware/env2 (tools/capture_env.py):
	//   * attack, Vib 1: 2.162 / 0.516 / 0.122 / 0.018 s against 2.164 / 0.540 / 0.122 /
	//     0.018 on hardware;
	//   * release, E. Organ 1 and Choir 3: a constant 0.92-0.94x across 125x of rate, both
	//     doubling step for step with ENV RELEASE;
	//   * decay, all twelve tones of decay_hold, timed above the recording's noise floor --
	//     marimba 0.23/0.48/1.01 s to fall 10/20/40 dB against 0.24/0.48/0.96 measured,
	//     piano 0.63/1.17/5.71 against 0.64/1.14/5.07, fbass 1.07/2.16/4.36 against
	//     1.05/2.10/4.18.
	//
	// And across the whole listen/hardware/3 reference session: sixteen of seventeen segments
	// allocate voices identically to the old stand-in, and the dB envelope correlates
	// 0.79-0.95 with the hardware recording segment for segment.
	m_pcm->set_env_engine(ENV_ENGINE);
	m_pcm->set_env_max_int_hz(32000);
	// The old stand-in, reachable only by setting ENV_ENGINE false: a fixed exponential fade
	// with no interrupt and no level readback.  Kept one flag away for now; it and
	// voice_status() should go once this has had some mileage.
	m_pcm->set_env_release_db_per_s(127.0);
	m_pcm->set_env_decay_db_per_s(0.0);
	// Six Multi Outputs, summed to the MIX pair.  PAN_L/PAN_R and the filter component
	// values below are in roland_u110_data.h, measured and documented there.


	for (int ch = 0; ch < 2; ch ++)
	{
		FILTER_BIQUAD(config, m_lpf_sk[ch * 2 + 0]).opamp_sk_lowpass_setup(
				SK1_R1, SK1_R2, SK_R3, SK_R4, SK1_C1, SK1_C2);
		FILTER_BIQUAD(config, m_lpf_sk[ch * 2 + 1]).opamp_sk_lowpass_setup(
				SK2_R1, SK2_R2, SK_R3, SK_R4, SK2_C1, SK2_C2);
		FILTER_RC(config, m_lpf_rc[ch]).set_lowpass(RC_R, RC_C);

		for (int out = 0; out < 6; out ++)
			m_pcm->add_route(out, m_lpf_sk[ch * 2 + 0], ch ? PAN_R[out] : PAN_L[out]);

		m_lpf_sk[ch * 2 + 0]->add_route(0, m_lpf_sk[ch * 2 + 1], 1.0);
		m_lpf_sk[ch * 2 + 1]->add_route(0, m_lpf_rc[ch], 1.0);

		// Output EQ correction -- MEASURED, not modelled, and switchable at runtime under
		// Tab -> Machine Configuration -> "Output EQ correction".
		//
		// Against the hardware captures in listen/hardware/3, level-matched over 200 Hz - 2 kHz, the
		// emulator runs +7.0 dB hot at 6 kHz (analysis/hf_excess.pdf).  The excess is a
		// broad bell: flat below 2 kHz, peaking at 6-6.5 kHz, back to zero by 10 kHz.  Seven
		// patches agree within +5.9 to +8.9 dB, and rendering natively at 32 kHz reproduces
		// it, so it is neither patch-specific nor a resampling artefact.  It sits on real
		// signal rather than added noise: on a sustained note the harmonic peaks and the
		// valleys between them rise together, the comb contrast dropping only 24.3 -> 22.2 dB.
		//
		// One RBJ peaking cut cancels it to a weighted rms residual of 0.53 dB, and across
		// ten patches it takes the 300 Hz - 10 kHz error from 2.83 dB rms to 1.20 dB.  Bass
		// is deliberately untouched: below 200 Hz the two already agree to 0.1 dB, so the
		// thin bottom end one hears is the treble excess, not a missing octave.
		//
		// `[I]` This is a CORRECTION LAYER, not the physics.  The Sallen-Key chain above is
		// the more likely culprit: as modelled it peaks +4.29 dB at 6065 Hz, where the
		// service notes' own simulation of the same circuit says +2.17 dB max -- twice the
		// resonance, in the same place as the measured error.  Fixing that is the real
		// repair; this filter is what makes the emulator match the hardware in the meantime.
		// Switch it off to hear the raw chain, or when working on the chain itself.
		FILTER_BIQUAD(config, m_eq[ch]).setup(
				filter_biquad_device::biquad_type::PEAK, EQ_FC, EQ_Q, EQ_GAIN);
		m_lpf_rc[ch]->add_route(0, m_eq[ch], 1.0);
		m_eq[ch]->add_route(0, "speaker", 1.0, ch);
	}

	GENERIC_CARTSLOT(config, m_cards[0], generic_romram_plain_slot, "u110_card", "bin")
			.set_device_load(FUNC(roland_u110_state::card_load<0>));
	m_cards[0]->set_device_unload(FUNC(roland_u110_state::card_unload<0>));
	GENERIC_CARTSLOT(config, m_cards[1], generic_romram_plain_slot, "u110_card", "bin")
			.set_device_load(FUNC(roland_u110_state::card_load<1>));
	m_cards[1]->set_device_unload(FUNC(roland_u110_state::card_unload<1>));
	GENERIC_CARTSLOT(config, m_cards[2], generic_romram_plain_slot, "u110_card", "bin")
			.set_device_load(FUNC(roland_u110_state::card_load<2>));
	m_cards[2]->set_device_unload(FUNC(roland_u110_state::card_unload<2>));
	GENERIC_CARTSLOT(config, m_cards[3], generic_romram_plain_slot, "u110_card", "bin")
			.set_device_load(FUNC(roland_u110_state::card_load<3>));
	m_cards[3]->set_device_unload(FUNC(roland_u110_state::card_unload<3>));

	SOFTWARE_LIST(config, "card_list").set_original("u110_card");

	// MIDI IN (JK1 -> IC2 opto-isolator -> CPU RXD, pin 18)
	midi_port_device &mdin(MIDI_PORT(config, "mdin", midiin_slot, "midiin"));
	mdin.rxd_handler().set(FUNC(roland_u110_state::midi_rxd_w));

	// MIDI OUT (CPU TXD, pin 17).  The firmware sends nothing unprompted -- there is no
	// active sensing and no keyboard -- so this carries bulk dumps and SysEx replies only.
	//
	MIDI_PORT(config, m_mdout, midiout_slot, "midiout");

	// MIDI THRU (JK3), buffered off the IC2 opto-isolator output -- see midi_rxd_w().
	MIDI_PORT(config, m_mdthru, midiout_slot, "midiout");
}


/**************************************************************************************
    ROM definitions
**************************************************************************************/

ROM_START( u110 )
	ROM_REGION( 0x10000, "maincpu", 0 )
	ROM_SYSTEM_BIOS(0, "v203", "Version 2.03")
	ROMX_LOAD( "roland_u110_pgm_15179960.bin", 0x00000, 0x10000, CRC(73e70eae) SHA1(ec0d76d325f27eddd7d8ac5530221137dbde6db1), ROM_BIOS(0) ) // MBM27C512, DIP-28
	ROM_SYSTEM_BIOS(1, "v200", "Version 2.00")
	ROMX_LOAD( "u110_v200.bin",                0x00000, 0x10000, CRC(88e1aa9f) SHA1(68cc648470d133a2d7268bd24b6346600366ffb1), ROM_BIOS(1) )

	// NOT A DUMP.  The U-110's LCD controller part is unidentified and its character
	// generator has never been dumped; without a font the display renders blank glyphs.
	// This is a synthesised ASCII font so the machine is legible - see
	// tools/make_lcd_cgrom.py in the project root.  Marked BAD_DUMP deliberately.
	ROM_REGION( 0x1000, "lcd", 0 )
	ROM_LOAD( "u110_lcd_cgrom.bin", 0x0000, 0x1000, BAD_DUMP CRC(c78231d7) SHA1(1dd8a086aec6d3d9ee0c0ac08dc5bad5192d2aff) )

	ROM_REGION( 0x200000, "pcmorg", 0 ) // wave ROMs before descrambling
	ROM_LOAD( "roland_t110_u110_u220_waverom0.bin", 0x000000, 0x80000, CRC(d5475560) SHA1(55d36199ea489969a729782afe4cbff5b1cede0e) )
	ROM_LOAD( "roland_t110_u110_u220_waverom1.bin", 0x080000, 0x80000, CRC(43a5b26c) SHA1(8b8067fbc6fa7b2e8694a3785f123627ea458d1f) )
	ROM_LOAD( "roland_t110_u110_u220_waverom2.bin", 0x100000, 0x80000, CRC(e79ee88a) SHA1(8dd9826d7d3e67b9a568eadeaaf7856a05f3e068) )
	ROM_LOAD( "roland_t110_u110_u220_waverom3.bin", 0x180000, 0x80000, CRC(da9fbf6d) SHA1(35b98a0fb5cf643fb7b761a74581486eee3bd8c3) )

	ROM_REGION( 0x400000, "pcm", ROMREGION_ERASEFF )    // after descrambling; cards fill the gaps
ROM_END

} // anonymous namespace


SYST( 1989, u110, 0, 0, u110, u110, roland_u110_state, init_u110, "Roland", "U-110 PCM Sound Module", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
