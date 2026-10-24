// license:BSD-3-Clause
// copyright-holders:Helge Thorsten Kautz
/*************************************************************************************************

    Roland U-220 RS-PCM Sound Module (1989)

    Work in progress, Phase 1 of Ostrich (github: none yet): get the firmware to boot.
    This revision is INSTRUMENTED: it logs what the firmware does to the parts of the
    machine that are not yet understood, so that the memory map can be read off a run.

    Hardware (service notes, main board 79574130):

      IC6   P8098 CPU, 12 MHz (8-bit bus)
      IC8   HN27C101 program EPROM, 128 KB
      IC10  HM62256 SRAM, 32 KB, battery backed
      IC12  MB623157 I/O gate array: address latch, chip selects (ROM1, ROM2-5, RAM, LP,
            EFFECT, CARD, KEY, 8251), extended address lines A14-A16, the LCD bus
            (D0-D7, E, R/W, RS), key matrix (SC0-SC3 out, SD0-SD7 in), LED0, INT
      IC17  MB87419 \ PCM ("Roland LP"), as in the U-110 and CM-32P
      IC18  MB87420 /
      IC25  TC23SC140AF-007 effect processor, with IC26/IC27 M5M4464 DRAM
      LCD   DM047Z-7BL7, 24 x 2 characters, backlit
      ACH7  battery voltage (via IC4 TL062)
      PWM   LCD contrast (via IC9)
      HSI0  from IC12 INT through IC13A: the LCD ready interrupt (firmware 0x2130 drains
            a 64-byte character queue at 0x1100 into 0x0800/0x0801)

    Firmware (v1.02, 90/02/16), from the disassembly:

      0x2000-0x2013  vectors -> stubs at 0x2104...; live: HSI0 0x2130 (LCD),
                     software timer 0x218B, serial 0x224E (MIDI), EXTINT 0x22DD
      0x2080         reset -> 0x2100 -> 0x27E4 -> init at 0x25BC
      0x25BC         sp = 0x2000; 0x1000 <- 0x30; 0x1001 <- 0; clears 0x1002-0x11FF and
                     0x1F80-0x1FFF; then writes 1..4 to 0x1000 calling one routine each

    So 0x1000/0x1001 are IC12 registers, 0x1002-0x1FFF is RAM, and 0x1000 selects what
    the window at 0x0800-0x0FFF shows -- a device, or a 2 KB page of ROM or RAM:

      0x1000      window 0x0800-0x0FFF
      00-0F       SRAM, page (bank & 0F) * 2 KB              banks 1-4, CD=00, D4=0D cleared at
                                                            boot through the window
      10          LCD: 0800 command, 0801 data              the HSI0 handler at 0x2130
      30          key matrix: 080D, 080E (active low)       the key scan at 0xE1A2
      40          effect processor (IC25)?                  table entry DE
      60          PCM chip, CM-32P register layout          table entry DD, used by the chip
                                                            init at 0x2309 and EXTINT
      80-9F       EPROM upper half, 0x10000 + (bank & 1F)   DA=97 -> 0x1B800, the message page;
                  * 2 KB                                    0x8DEC turns a 16-bit data address
                                                            into bank 80|(a >> 11)

    The firmware keeps these selectors in registers CD-DE, loaded at 0x8DBF from bank 96
    offset 0x256 -- which is EPROM 0x1B256, so the table documents itself:

      CD=00 CE=01 CF=02 D0=03 D1=04 D2=05 D3=0C D4=0D D5=91 D6=92 D7=93 D8=95 D9=96
      DA=97 DB=98 DC=99 DD=60 DE=40

    SRAM pages 0E and 0F appear nowhere in it: they are the fixed 0x1000-0x1FFF (with
    IC12's two registers over the first two bytes), as the 4 KB arithmetic suggests.

    There is no code banking: 0x2000-0xFFFF is the lower 64 KB of the EPROM, fixed.  The
    upper 64 KB is data, read through the window.

*************************************************************************************************/

#include "emu.h"

#include "bus/midi/midi.h"
#include "cpu/mcs96/i8x9x.h"
#include "machine/nvram.h"
#include "speaker.h"

#include <set>
#include <tuple>


namespace {

class roland_u220_state : public driver_device
{
public:
	roland_u220_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_progrom(*this, "progrom")
		, m_nvram(*this, "nvram")
		, m_keys(*this, "KEYS")
		, m_mdout(*this, "mdout")
		, m_mdthru(*this, "mdthru")
	{
	}

	void u220(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	void mem_map(address_map &map) ATTR_COLD;

	u8 low_r(offs_t offset);
	void low_w(offs_t offset, u8 data);
	u8 window_r(offs_t offset);
	u8 fixed_r(offs_t offset);
	void fixed_w(offs_t offset, u8 data);
	void window_w(offs_t offset, u8 data);
	u8 bank_r();
	void bank_w(u8 data);
	u8 reg1001_r();
	void reg1001_w(u8 data);

	u16 battery_r() { return 0x2a0; }

	// MIDI: the CPU core takes and gives whole bytes; MAME's ports are bit level
	void midi_rxd_w(int state);
	TIMER_CALLBACK_MEMBER(midi_rx_bit);
	void midi_tx_w(u16 data);
	void midi_tx_start();
	TIMER_CALLBACK_MEMBER(midi_tx_bit);

	TIMER_CALLBACK_MEMBER(lcd_ready);
	TIMER_CALLBACK_MEMBER(lcd_dump);
	void lcd_shadow_cmd(u8 data);
	void lcd_shadow_data(u8 data);

	// Logs one line per distinct (what, pc, address, bank) -- a boot touches the same
	// places thousands of times, and the first time is the one worth reading.
	void note(char what, offs_t addr, u8 data);

	required_device<i8x9x_device> m_maincpu;
	required_region_ptr<u8> m_progrom;

	u8 m_bank = 0;
	u8 m_reg1001 = 0;
	u8 m_pcmreg[0x20] = {};             // PCM chip registers, until the device is wired
	u8 m_effreg[0x80] = {};
	u8 m_ram70[0x800] = {};             // bank 70: what is it?             // effect processor, until it is understood
	required_device<nvram_device> m_nvram;
	required_ioport m_keys;
	required_device<midi_port_device> m_mdout;
	required_device<midi_port_device> m_mdthru;

	static constexpr u32 MIDI_BAUD = 31250;
	emu_timer *m_midi_rx_timer = nullptr;
	emu_timer *m_midi_tx_timer = nullptr;
	int m_rxd = 1;
	int m_rx_bit = -1;
	u8 m_rx_data = 0;
	u8 m_tx_fifo[16] = {};
	u8 m_tx_head = 0, m_tx_tail = 0;
	int m_tx_bit = -1;
	u8 m_tx_data = 0;
	std::unique_ptr<u8[]> m_ram;        // IC10, 32 KB, battery backed
	std::unique_ptr<u8[]> m_low;        // 0x0100-0x07FF, in case it is RAM after all

	std::set<std::tuple<char, offs_t, offs_t, u8>> m_seen;

	emu_timer *m_lcd_ready_timer = nullptr;
	emu_timer *m_lcd_dump_timer = nullptr;
	u8 m_ddram[0x80] = {};
	u8 m_lcd_addr = 0;
	bool m_lcd_cgram = false;
	std::string m_lcd_last;
};


void roland_u220_state::machine_start()
{
	m_low = std::make_unique<u8[]>(0x800);
	m_ram = std::make_unique<u8[]>(0x8000);
	m_nvram->set_base(m_ram.get(), 0x8000);
	save_pointer(NAME(m_ram), 0x8000);
	save_pointer(NAME(m_low), 0x800);
	save_item(NAME(m_bank));
	save_item(NAME(m_reg1001));
	save_item(NAME(m_pcmreg));
	save_item(NAME(m_effreg));

	std::fill(std::begin(m_ddram), std::end(m_ddram), ' ');
	m_midi_rx_timer = timer_alloc(FUNC(roland_u220_state::midi_rx_bit), this);
	m_midi_tx_timer = timer_alloc(FUNC(roland_u220_state::midi_tx_bit), this);
	save_item(NAME(m_rxd));
	save_item(NAME(m_rx_bit));
	save_item(NAME(m_rx_data));
	save_item(NAME(m_tx_fifo));
	save_item(NAME(m_tx_head));
	save_item(NAME(m_tx_tail));
	save_item(NAME(m_tx_bit));
	save_item(NAME(m_tx_data));

	m_lcd_ready_timer = timer_alloc(FUNC(roland_u220_state::lcd_ready), this);
	m_lcd_dump_timer = timer_alloc(FUNC(roland_u220_state::lcd_dump), this);
	m_lcd_dump_timer->adjust(attotime::from_msec(100), 0, attotime::from_msec(100));
}

void roland_u220_state::machine_reset()
{
	m_bank = 0;
	m_reg1001 = 0;
	m_rx_bit = -1;
	m_tx_bit = -1;
	m_tx_head = m_tx_tail = 0;
	m_mdout->write_txd(1);                  // MIDI OUT idles high
}


/**************************************************************************************
    MIDI, as in roland_u110.cpp: IN is deserialised here (31250 baud 8N1) and handed to
    the CPU a byte at a time; OUT is serialised from the CPU's bytes; THRU is a wire.
**************************************************************************************/

void roland_u220_state::midi_rxd_w(int state)
{
	m_mdthru->write_txd(state);

	if (m_rx_bit < 0 && m_rxd && !state)
	{
		m_rx_bit = 0;
		m_rx_data = 0;
		m_midi_rx_timer->adjust(attotime::from_ticks(3, 2 * MIDI_BAUD));   // mid bit 0
	}
	m_rxd = state;
}

TIMER_CALLBACK_MEMBER(roland_u220_state::midi_rx_bit)
{
	if (m_rx_bit < 8)
	{
		m_rx_data |= m_rxd << m_rx_bit;
		m_rx_bit++;
		m_midi_rx_timer->adjust(attotime::from_ticks(1, MIDI_BAUD));
	}
	else
	{
		m_maincpu->serial_w(m_rx_data);
		logerror("MIDI IN %f %02X\n", machine().time().as_double(), m_rx_data);
		m_rx_bit = -1;
	}
}

void roland_u220_state::midi_tx_w(u16 data)
{
	const u8 next = (m_tx_head + 1) % u8(std::size(m_tx_fifo));
	if (next == m_tx_tail)
	{
		logerror("MIDI OUT overrun: %02X dropped\n", data & 0xff);
		return;
	}
	m_tx_fifo[m_tx_head] = u8(data);
	m_tx_head = next;
	logerror("MIDI OUT %f %02X\n", machine().time().as_double(), data & 0xff);
	if (m_tx_bit < 0)
		midi_tx_start();
}

void roland_u220_state::midi_tx_start()
{
	if (m_tx_head == m_tx_tail)
		return;
	m_tx_data = m_tx_fifo[m_tx_tail];
	m_tx_tail = (m_tx_tail + 1) % u8(std::size(m_tx_fifo));
	m_tx_bit = 0;
	m_mdout->write_txd(0);                  // start bit
	m_midi_tx_timer->adjust(attotime::from_ticks(1, MIDI_BAUD));
}

TIMER_CALLBACK_MEMBER(roland_u220_state::midi_tx_bit)
{
	if (m_tx_bit < 8)
	{
		m_mdout->write_txd(BIT(m_tx_data, m_tx_bit));
		m_tx_bit++;
		m_midi_tx_timer->adjust(attotime::from_ticks(1, MIDI_BAUD));
	}
	else if (m_tx_bit == 8)
	{
		m_mdout->write_txd(1);              // stop bit
		m_tx_bit++;
		m_midi_tx_timer->adjust(attotime::from_ticks(1, MIDI_BAUD));
	}
	else
	{
		m_tx_bit = -1;
		midi_tx_start();
	}
}

void roland_u220_state::note(char what, offs_t addr, u8 data)
{
	const offs_t pc = m_maincpu->pc();
	if (m_seen.emplace(what, pc, addr, m_bank).second)
		logerror("%c pc=%04X addr=%04X bank=%02X data=%02X\n", what, pc, addr, m_bank, data);
}


/**************************************************************************************
    Everything below 0x2000 that is not yet understood
**************************************************************************************/

u8 roland_u220_state::low_r(offs_t offset)
{
	note('r', 0x100 + offset, m_low[offset]);
	return m_low[offset];
}

void roland_u220_state::low_w(offs_t offset, u8 data)
{
	note('w', 0x100 + offset, data);
	m_low[offset] = data;
}

u8 roland_u220_state::window_r(offs_t offset)
{
	u8 data = 0xff;
	switch (m_bank >> 4)
	{
	case 0x0:                                           // SRAM pages
		data = m_ram[((m_bank & 0x0f) << 11) | offset];
		break;
	case 0x1:                                           // LCD
		data = (offset & 1) ? m_ddram[m_lcd_addr & 0x7f] : (m_lcd_addr & 0x7f);
		note('l', 0x800 + offset, data);
		break;
	case 0x3:                                           // key matrix
		// Active low, 14 bits: 080E is bits 0-7, 080D bits 8-13 (the scan at 0xE1A2 sets
		// the top two itself and inverts the word).  Bit n becomes key code
		// E194[n]: 6A-6F, 76, 77, 52-57.
		if (offset == 0x0e)
			data = m_keys->read() & 0xff;
		else if (offset == 0x0d)
			data = (m_keys->read() >> 8) | 0xc0;
		else
			note('k', 0x800 + offset, data);
		break;
	case 0x4:                                           // effect processor?
		data = m_effreg[offset & 0x7f];
		note('e', 0x800 + offset, data);
		break;
	case 0x7:                                           // unknown, written and read back
		data = m_ram70[offset];
		note('7', 0x800 + offset, data);
		break;
	case 0x6:                                           // PCM chip
		data = m_pcmreg[offset & 0x1f];
		note('p', 0x800 + offset, data);
		break;
	case 0x8: case 0x9:                                 // EPROM upper half
		data = m_progrom[0x10000 | ((m_bank & 0x1f) << 11) | offset];
		break;
	default:
		note('?', 0x800 + offset, data);
		break;
	}
	return data;
}

void roland_u220_state::window_w(offs_t offset, u8 data)
{
	switch (m_bank >> 4)
	{
	case 0x0:
		m_ram[((m_bank & 0x0f) << 11) | offset] = data;
		break;
	case 0x1:
		if (offset == 0)
			lcd_shadow_cmd(data);
		else if (offset == 1)
			lcd_shadow_data(data);
		else
			note('L', 0x800 + offset, data);
		m_lcd_ready_timer->adjust(attotime::from_usec(40));
		break;
	case 0x3:
		note('K', 0x800 + offset, data);
		break;
	case 0x4:
		m_effreg[offset & 0x7f] = data;
		note('E', 0x800 + offset, data);
		break;
	case 0x7:
		m_ram70[offset] = data;
		note('8', 0x800 + offset, data);
		break;
	case 0x6:
		m_pcmreg[offset & 0x1f] = data;
		note('P', 0x800 + offset, data);
		break;
	default:
		note('!', 0x800 + offset, data);
		break;
	}
}

u8 roland_u220_state::fixed_r(offs_t offset)
{
	return m_ram[0x7002 + offset];
}

void roland_u220_state::fixed_w(offs_t offset, u8 data)
{
	m_ram[0x7002 + offset] = data;
}

u8 roland_u220_state::bank_r()
{
	return m_bank;
}

void roland_u220_state::bank_w(u8 data)
{
	if (m_seen.emplace('B', m_maincpu->pc(), 0x1000, data).second)
		logerror("B pc=%04X bank %02X -> %02X\n", m_maincpu->pc(), m_bank, data);
	m_bank = data;
}

u8 roland_u220_state::reg1001_r()
{
	note('r', 0x1001, m_reg1001);
	return m_reg1001;
}

void roland_u220_state::reg1001_w(u8 data)
{
	note('w', 0x1001, data);
	m_reg1001 = data;
}


/**************************************************************************************
    LCD.  IC12 drives the module's bus and raises INT (-> HSI0) when it is ready for
    the next byte; the firmware feeds it from that interrupt.
**************************************************************************************/

TIMER_CALLBACK_MEMBER(roland_u220_state::lcd_ready)
{
	m_maincpu->set_input_line(i8x9x_device::HSI0_LINE, ASSERT_LINE);
	m_maincpu->set_input_line(i8x9x_device::HSI0_LINE, CLEAR_LINE);
}

// A text copy of the display, which is all a headless run needs.
void roland_u220_state::lcd_shadow_cmd(u8 data)
{
	if (data == 0x01)
	{
		std::fill(std::begin(m_ddram), std::end(m_ddram), ' ');
		m_lcd_addr = 0;
		m_lcd_cgram = false;
	}
	else if ((data & 0xfe) == 0x02)
	{
		m_lcd_addr = 0;
		m_lcd_cgram = false;
	}
	else if (data & 0x80)
	{
		m_lcd_addr = data & 0x7f;
		m_lcd_cgram = false;
	}
	else if (data & 0x40)
		m_lcd_cgram = true;
}

void roland_u220_state::lcd_shadow_data(u8 data)
{
	if (m_lcd_cgram)
		return;
	m_ddram[m_lcd_addr & 0x7f] = data;
	m_lcd_addr = (m_lcd_addr + 1) & 0x7f;
}

TIMER_CALLBACK_MEMBER(roland_u220_state::lcd_dump)
{
	std::string text;
	for (int row = 0; row < 2; row++)
	{
		text += '[';
		for (int col = 0; col < 24; col++)
		{
			const u8 c = m_ddram[row * 0x40 + col];
			text += (c >= 0x20 && c < 0x7f) ? char(c) : (c < 0x10 ? '#' : '.');
		}
		text += ']';
	}
	if (text != m_lcd_last)
	{
		m_lcd_last = text;
		logerror("LCD %8.3f %s\n", machine().time().as_double(), text);
	}
}

/**************************************************************************************
    Memory map
**************************************************************************************/

void roland_u220_state::mem_map(address_map &map)
{
	map(0x0100, 0x07ff).rw(FUNC(roland_u220_state::low_r), FUNC(roland_u220_state::low_w));
	map(0x0800, 0x0fff).rw(FUNC(roland_u220_state::window_r), FUNC(roland_u220_state::window_w));
	map(0x1000, 0x1000).rw(FUNC(roland_u220_state::bank_r), FUNC(roland_u220_state::bank_w));
	map(0x1001, 0x1001).rw(FUNC(roland_u220_state::reg1001_r), FUNC(roland_u220_state::reg1001_w));
	map(0x1002, 0x1fff).rw(FUNC(roland_u220_state::fixed_r), FUNC(roland_u220_state::fixed_w));
	map(0x2000, 0x7fff).rom().region("progrom", 0x2000);
	map(0x8000, 0xffff).rom().region("progrom", 0x8000);
}

static INPUT_PORTS_START(u220)
	// Bit n is key code E194[n].  Worked out by pressing each bit on the play screen
	// (tools/u220_keys.lua) and confirmed by round trips (EDIT then EXIT, and so on).
	// Bits 6 and 7 do nothing on the U-220; the matrix has room for 14, the panel has 12.
	PORT_START("KEYS")
	PORT_BIT(0x0001, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Edit")            PORT_CODE(KEYCODE_E)
	PORT_BIT(0x0002, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Data")            PORT_CODE(KEYCODE_D)
	PORT_BIT(0x0004, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Exit")            PORT_CODE(KEYCODE_BACKSPACE)
	PORT_BIT(0x0008, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Enter")           PORT_CODE(KEYCODE_ENTER)
	PORT_BIT(0x0010, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Part/Inst <<")    PORT_CODE(KEYCODE_PGUP)
	PORT_BIT(0x0020, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Part/Inst >>")    PORT_CODE(KEYCODE_PGDN)
	PORT_BIT(0x00c0, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x0100, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Mark")            PORT_CODE(KEYCODE_M)
	PORT_BIT(0x0200, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Jump")            PORT_CODE(KEYCODE_J)
	PORT_BIT(0x0400, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Cursor <")        PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0x0800, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Cursor >")        PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT(0x1000, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Value -")         PORT_CODE(KEYCODE_DOWN)
	PORT_BIT(0x2000, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Value +")         PORT_CODE(KEYCODE_UP)
	PORT_BIT(0xc000, IP_ACTIVE_LOW, IPT_UNUSED)
INPUT_PORTS_END

void roland_u220_state::u220(machine_config &config)
{
	P8098(config, m_maincpu, 12_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &roland_u220_state::mem_map);
	m_maincpu->ach7_cb().set(FUNC(roland_u220_state::battery_r));
	m_maincpu->serial_tx_cb().set(FUNC(roland_u220_state::midi_tx_w));

	// IC10, reached through the window (pages 00-0D) and at 0x1000-0x1FFF (0E-0F).
	NVRAM(config, m_nvram, nvram_device::DEFAULT_ALL_0);

	// No LCD device yet: MAME's HD44780 needs a character ROM nobody has dumped, and the
	// boot work needs the text, not the pixels -- which the shadow above provides.

	SPEAKER(config, "speaker", 2).front();

	// MIDI IN (JK1, opto-isolated to RXD), OUT (TXD), THRU (from the same opto output)
	midi_port_device &mdin(MIDI_PORT(config, "mdin", midiin_slot, "midiin"));
	mdin.rxd_handler().set(FUNC(roland_u220_state::midi_rxd_w));
	MIDI_PORT(config, m_mdout, midiout_slot, "midiout");
	MIDI_PORT(config, m_mdthru, midiout_slot, "midiout");
}


ROM_START(u220)
	ROM_REGION(0x20000, "progrom", 0)
	ROM_SYSTEM_BIOS(0, "v102", "Version 1.02")
	ROMX_LOAD("u-220_roland_1-0-2.ic8", 0x00000, 0x20000, CRC(d5492e9b) SHA1(9d72b1688a173505b5e1c5ddac01efa78ee76c7f), ROM_BIOS(0))
	ROM_SYSTEM_BIOS(1, "v101", "Version 1.01")
	ROMX_LOAD("u-220_roland_1-0-1.ic8", 0x00000, 0x20000, CRC(893afa9c) SHA1(d0b66c0ea0e3af284a1806226aed79d8da2f3dd4), ROM_BIOS(1)) // HN27C101G-20
	ROM_SYSTEM_BIOS(2, "v100", "Version 1.00")
	ROMX_LOAD("u-220_roland_1-0-0.ic8", 0x00000, 0x20000, CRC(82abd055) SHA1(e1c6f1ee29cebe8663cb0c6989b8faebb3d841fd), ROM_BIOS(2))

	// Interleaved as in current MAME (roland_u20.cpp): A-D where the U-110 has its four
	// internal ROMs, E and F (= cards SN-U110-08/09) where its first two card slots are.
	ROM_REGION(0x400000, "waverom", ROMREGION_ERASE00)
	ROM_LOAD("roland-a_r15179892f_mb834000a-20_226-aa.bin", 0x000000, 0x080000, CRC(d5475560) SHA1(55d36199ea489969a729782afe4cbff5b1cede0e)) // IC19
	ROM_LOAD("roland-e_r15179947_mb834000a-20_3a1-aa.bin",  0x080000, 0x080000, CRC(104f3974) SHA1(71b6f97df6dd4e573db5743c10c77e86d67965a3)) // IC23
	ROM_LOAD("roland-b_r15179893f_mb834000a-20_227-aa.bin", 0x100000, 0x080000, CRC(43a5b26c) SHA1(8b8067fbc6fa7b2e8694a3785f123627ea458d1f)) // IC20
	ROM_LOAD("roland-f_r15179948_mb834000a-20_3a2-aa.bin",  0x180000, 0x080000, CRC(af1c2778) SHA1(aea715a0a554c071a2e912894805a8d819802e1d)) // IC24
	ROM_LOAD("roland-c_r15179894f_mb834000a-20_228-aa.bin", 0x200000, 0x080000, CRC(e79ee88a) SHA1(8dd9826d7d3e67b9a568eadeaaf7856a05f3e068)) // IC21
	ROM_LOAD("roland-d_r15179895f_mb834000a-20_229-aa.bin", 0x300000, 0x080000, CRC(da9fbf6d) SHA1(35b98a0fb5cf643fb7b761a74581486eee3bd8c3)) // IC22
ROM_END

} // anonymous namespace


SYST(1989, u220, 0, 0, u220, u220, roland_u220_state, empty_init, "Roland", "U-220 RS-PCM Sound Module", MACHINE_NO_SOUND | MACHINE_NOT_WORKING)
