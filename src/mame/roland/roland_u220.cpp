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
    the window at 0x0800-0x0FFF shows.  THAT window is what this revision instruments.

*************************************************************************************************/

#include "emu.h"

#include "cpu/mcs96/i8x9x.h"
#include "video/hd44780.h"

#include "emupal.h"
#include "screen.h"
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
		, m_lcd(*this, "lcd")
		, m_progrom(*this, "progrom")
	{
	}

	void u220(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	void mem_map(address_map &map) ATTR_COLD;

	u8 rom_hi_r(offs_t offset);

	u8 low_r(offs_t offset);
	void low_w(offs_t offset, u8 data);
	u8 ic12_r(offs_t offset);
	void ic12_w(offs_t offset, u8 data);
	u8 window_r(offs_t offset);
	void window_w(offs_t offset, u8 data);
	u8 bank_r();
	void bank_w(u8 data);
	u8 reg1001_r();
	void reg1001_w(u8 data);

	u16 battery_r() { return 0x2a0; }

	void lcd_palette(palette_device &palette) const ATTR_COLD;
	TIMER_CALLBACK_MEMBER(lcd_ready);
	TIMER_CALLBACK_MEMBER(lcd_dump);
	void lcd_shadow_cmd(u8 data);
	void lcd_shadow_data(u8 data);

	// Logs one line per distinct (what, pc, address, bank) -- a boot touches the same
	// places thousands of times, and the first time is the one worth reading.
	void note(char what, offs_t addr, u8 data);

	required_device<i8x9x_device> m_maincpu;
	required_device<hd44780_device> m_lcd;
	required_region_ptr<u8> m_progrom;

	u8 m_bank = 0;
	u8 m_reg1001 = 0;
	u8 m_ic12[0x20] = {};
	std::unique_ptr<u8[]> m_window;     // 256 banks x 2 KB, until the real map is known
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
	m_window = std::make_unique<u8[]>(0x100 * 0x800);
	m_low = std::make_unique<u8[]>(0x800);
	save_pointer(NAME(m_window), 0x100 * 0x800);
	save_pointer(NAME(m_low), 0x800);
	save_item(NAME(m_bank));
	save_item(NAME(m_reg1001));
	save_item(NAME(m_ic12));

	m_lcd_ready_timer = timer_alloc(FUNC(roland_u220_state::lcd_ready), this);
	m_lcd_dump_timer = timer_alloc(FUNC(roland_u220_state::lcd_dump), this);
	m_lcd_dump_timer->adjust(attotime::from_msec(100), 0, attotime::from_msec(100));
}

void roland_u220_state::machine_reset()
{
	m_bank = 0;
	m_reg1001 = 0;
}

void roland_u220_state::note(char what, offs_t addr, u8 data)
{
	const offs_t pc = m_maincpu->pc();
	if (m_seen.emplace(what, pc, addr, m_bank).second)
		logerror("%c pc=%04X addr=%04X bank=%02X data=%02X\n", what, pc, addr, m_bank, data);
}


/**************************************************************************************
    Program ROM.  0x2000-0x7FFF is fixed; how the other 64 KB reaches the CPU is not
    known yet, so the upper window shows the lower half's 0x8000-0xFFFF for now.
**************************************************************************************/

u8 roland_u220_state::rom_hi_r(offs_t offset)
{
	return m_progrom[0x8000 + offset];
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

u8 roland_u220_state::ic12_r(offs_t offset)
{
	u8 data = m_ic12[offset];
	if (offset == 0)
		data = m_lcd->control_r();
	else if (offset == 1)
		data = m_lcd->data_r();
	note('R', 0x800 + offset, data);
	return data;
}

void roland_u220_state::ic12_w(offs_t offset, u8 data)
{
	m_ic12[offset] = data;
	if (offset == 0)
	{
		m_lcd->control_w(data);
		lcd_shadow_cmd(data);
		m_lcd_ready_timer->adjust(attotime::from_usec(40));
		return;
	}
	if (offset == 1)
	{
		m_lcd->data_w(data);
		lcd_shadow_data(data);
		m_lcd_ready_timer->adjust(attotime::from_usec(40));
		return;
	}
	note('W', 0x800 + offset, data);
}

u8 roland_u220_state::window_r(offs_t offset)
{
	const u8 data = m_window[(m_bank << 11) | offset];
	note('<', 0x820 + offset, data);
	return data;
}

void roland_u220_state::window_w(offs_t offset, u8 data)
{
	note('>', 0x820 + offset, data);
	m_window[(m_bank << 11) | offset] = data;
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

// A text copy of the display for headless runs: the HD44780 keeps its DDRAM to itself.
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

void roland_u220_state::lcd_palette(palette_device &palette) const
{
	palette.set_pen_color(0, rgb_t(0x20, 0x48, 0x10));
	palette.set_pen_color(1, rgb_t(0xb0, 0xf0, 0x60));
}


/**************************************************************************************
    Memory map
**************************************************************************************/

void roland_u220_state::mem_map(address_map &map)
{
	map(0x0100, 0x07ff).rw(FUNC(roland_u220_state::low_r), FUNC(roland_u220_state::low_w));
	map(0x0800, 0x081f).rw(FUNC(roland_u220_state::ic12_r), FUNC(roland_u220_state::ic12_w));
	map(0x0820, 0x0fff).rw(FUNC(roland_u220_state::window_r), FUNC(roland_u220_state::window_w));
	map(0x1000, 0x1000).rw(FUNC(roland_u220_state::bank_r), FUNC(roland_u220_state::bank_w));
	map(0x1001, 0x1001).rw(FUNC(roland_u220_state::reg1001_r), FUNC(roland_u220_state::reg1001_w));
	map(0x1002, 0x1fff).ram().share("ram");
	map(0x2000, 0x7fff).rom().region("progrom", 0x2000);
	map(0x8000, 0xffff).r(FUNC(roland_u220_state::rom_hi_r));
}

static INPUT_PORTS_START(u220)
INPUT_PORTS_END

void roland_u220_state::u220(machine_config &config)
{
	P8098(config, m_maincpu, 12_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &roland_u220_state::mem_map);
	m_maincpu->ach7_cb().set(FUNC(roland_u220_state::battery_r));

	HD44780(config, m_lcd, 270'000);
	m_lcd->set_lcd_size(2, 24);

	screen_device &screen(SCREEN(config, "screen").set_lcd());
	screen.set_refresh_hz(60);
	screen.set_size(24 * 6 - 1, 2 * 9 - 1);
	screen.set_visarea(0, 24 * 6 - 2, 0, 2 * 9 - 2);
	screen.set_screen_update("lcd", FUNC(hd44780_device::screen_update));
	screen.set_palette("palette");
	PALETTE(config, "palette", FUNC(roland_u220_state::lcd_palette), 2);

	SPEAKER(config, "speaker", 2).front();
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
