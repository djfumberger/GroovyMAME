// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/***************************************************************************

    Hitachi HD44780 LCD controller

    TODO:
    - dump internal CGROM
    - emulate osc pin, determine video timings and busy flag duration from it,
      and if possible, remove m_busy_factor

***************************************************************************/

#include "emu.h"
#include "video/hd44780.h"

//#define VERBOSE 1
#include "logmacro.h"


//**************************************************************************
//  DEVICE DEFINITIONS
//**************************************************************************

DEFINE_DEVICE_TYPE(HD44780,  hd44780_device,  "hd44780",  "Hitachi HD44780 LCD Controller")
DEFINE_DEVICE_TYPE(HD44780U, hd44780u_device, "hd44780u", "Hitachi HD44780U LCD Controller")
DEFINE_DEVICE_TYPE(SED1278,  sed1278_device,  "sed1278",  "Epson SED1278 LCD Controller") // packaged as either SED1278F0B or SED1278D0B
DEFINE_DEVICE_TYPE(KS0066,   ks0066_device,   "ks0066",   "Samsung KS0066 LCD Controller")


//-------------------------------------------------
//  ROM( hd44780 )
//-------------------------------------------------

ROM_START( hd44780 )
	ROM_DEFAULT_BIOS("a00")
	ROM_SYSTEM_BIOS(0, "a00", "A00")

	ROM_REGION( 0x1000, "cgrom", 0 )
	ROMX_LOAD( "hd44780_a00.bin",    0x0000, 0x1000,  BAD_DUMP CRC(e459877c) SHA1(65cf075a988cdcbb316b9afdd0529b374a1a65ec), ROM_BIOS(0)) // from page 97 of the 1985 HD44780 datasheet from crystalfontz
ROM_END

ROM_START( hd44780u )
	ROM_DEFAULT_BIOS("a00")
	ROM_SYSTEM_BIOS(0, "a00", "A00")
	ROM_SYSTEM_BIOS(1, "a02", "A02")
	// Note the HD44780UA01 font does exist in the 1994 M24T026 Hitachi LCD Controller Driver LSI Data Book on bitsavers as well

	ROM_REGION( 0x1000, "cgrom", 0 )
	ROMX_LOAD( "hd44780u_a00.bin",    0x0000, 0x1000,  BAD_DUMP CRC(8494cb6b) SHA1(2d4f9cf5ff81f20d2f4e4640f5b8a697a3781eef), ROM_BIOS(0)) // from page 17 of the 1999 HD44780U datasheet
	// this has 6 characters different from the HD44780A00
	// this differs slightly (see the '7') from the HD44780UA00 font in the 1994 M24T026 Hitachi LCD Controller Driver LSI Data Book on bitsavers
	// and needs confirmation from a real device as to which is correct; confirmation on the horizontal offset of the '[' would also be helpful
	ROMX_LOAD( "hd44780u_a02.bin",    0x0000, 0x1000,  BAD_DUMP CRC(6d522b42) SHA1(db8f59573c81933cfc9d3232d419406f0896e60b), ROM_BIOS(1)) // from page 18 of the 1999 HD44780U datasheet
	// this differs slightly (see the '7') from the HD44780UA02 font in the 1994 M24T026 Hitachi LCD Controller Driver LSI Data Book on bitsavers
	// and needs confirmation from a real device as to which is correct
ROM_END


ROM_START( sed1278 )
	ROM_DEFAULT_BIOS("0a")
	ROM_SYSTEM_BIOS(0, "0a", "0A")
	ROM_SYSTEM_BIOS(1, "0b", "0B")

	ROM_REGION( 0x1000, "cgrom", 0 )
	ROMX_LOAD( "sed1278_0a.bin",    0x0000, 0x1000,  BAD_DUMP CRC(e459877c) SHA1(65cf075a988cdcbb316b9afdd0529b374a1a65ec), ROM_BIOS(0)) // from page 9-32 of the SED1278 datasheet (100% identical to hd44780a00)
	ROMX_LOAD( "sed1278_0b.bin",    0x0000, 0x1000,  BAD_DUMP CRC(962498b7) SHA1(41866836ab4ed7bd4c3539bc8df492ba7d7ff42a), ROM_BIOS(1)) // from page 9-33 of the SED1278 datasheet
ROM_END

// TODO: many other SED1278 variants, documented in the datasheet

// ks0066_f00 is 100% identical to hd44780a00, see page 61 of the KS0066 non-U datasheet
// later ks0066 masks may be different, see page 48 of the other KS0066 datasheet
ROM_START( ks0066 )
	ROM_DEFAULT_BIOS("f00")
	ROM_SYSTEM_BIOS(0, "f00", "F00")
	//ROM_SYSTEM_BIOS(?, "f03", "F03")
	//ROM_SYSTEM_BIOS(?, "f04", "F04")
	ROM_SYSTEM_BIOS(1, "f05", "F05")
	//ROM_SYSTEM_BIOS(?, "f06", "F06")
	//ROM_SYSTEM_BIOS(?, "f59", "F59")

	ROM_REGION( 0x1000, "cgrom", 0 )
	ROMX_LOAD( "ks0066_f00.bin",    0x0000, 0x1000,  BAD_DUMP CRC(e459877c) SHA1(65cf075a988cdcbb316b9afdd0529b374a1a65ec), ROM_BIOS(0)) // from page 61 of the KS0066 non-U datasheet
	ROMX_LOAD( "ks0066_f05.bin",    0x0000, 0x1000,  BAD_DUMP CRC(af9e7bd6) SHA1(0196e871584ee5d370856e7307c0f9d1466e3e51), ROM_BIOS(1)) // from page 51 of the KS0066 datasheet
ROM_END


//**************************************************************************
//  live device
//**************************************************************************

//-------------------------------------------------
//  hd44780_device - constructor
//-------------------------------------------------

hd44780_device::hd44780_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: hd44780_device(mconfig, HD44780, tag, owner, clock)
{
}

hd44780_device::hd44780_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, m_pixel_update_cb(*this)
	, m_busy_factor(1.0)
	, m_cgrom(nullptr)
	, m_cgrom_region(*this, DEVICE_SELF)
	, m_rs_input(0)
	, m_rw_input(0)
	, m_db_input(0)
	, m_enabled(false)
	, m_function_set_at_any_time(false)
{
}

hd44780u_device::hd44780u_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	hd44780_device(mconfig, HD44780U, tag, owner, clock)
{
}

sed1278_device::sed1278_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	hd44780_device(mconfig, SED1278, tag, owner, clock)
{
}

ks0066_device::ks0066_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	hd44780_device(mconfig, KS0066, tag, owner, clock)
{
}


//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *hd44780_device::device_rom_region() const
{
	return ROM_NAME(hd44780);
}

const tiny_rom_entry *hd44780u_device::device_rom_region() const
{
	return ROM_NAME(hd44780u);
}

const tiny_rom_entry *sed1278_device::device_rom_region() const
{
	return ROM_NAME(sed1278);
}

const tiny_rom_entry *ks0066_device::device_rom_region() const
{
	return ROM_NAME(ks0066);
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void hd44780_device::device_start()
{
	m_cgrom = m_cgrom_region.found() ? m_cgrom_region : memregion("cgrom")->base();

	m_pixel_update_cb.resolve();

	m_busy_timer = timer_alloc(FUNC(hd44780_device::clear_busy_flag), this);
	m_blink_timer = timer_alloc(FUNC(hd44780_device::blink_tick), this);
	m_blink_timer->adjust(attotime::from_msec(409), 0, attotime::from_msec(409));

	// state saving
	save_item(NAME(m_busy_factor));
	save_item(NAME(m_busy_flag));
	save_item(NAME(m_ac));
	save_item(NAME(m_dr));
	save_item(NAME(m_ir));
	save_item(NAME(m_active_ram));
	save_item(NAME(m_display_on));
	save_item(NAME(m_cursor_on));
	save_item(NAME(m_shift_on));
	save_item(NAME(m_blink_on));
	save_item(NAME(m_direction));
	save_item(NAME(m_data_len));
	save_item(NAME(m_num_line));
	save_item(NAME(m_char_size));
	save_item(NAME(m_disp_shift));
	save_item(NAME(m_blink));
	save_item(NAME(m_ddram));
	save_item(NAME(m_cgram));
	save_item(NAME(m_nibble));
	save_item(NAME(m_rs_input));
	save_item(NAME(m_rw_input));
	save_item(NAME(m_db_input));
	save_item(NAME(m_enabled));
	save_item(NAME(m_rs_state));
	save_item(NAME(m_rw_state));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void hd44780_device::device_reset()
{
	memset(m_ddram, 0x20, sizeof(m_ddram)); // filled with SPACE char
	memset(m_cgram, 0, sizeof(m_cgram));

	m_ac         = 0;
	m_dr         = 0;
	m_ir         = 0;
	m_active_ram = DDRAM;
	m_display_on = false;
	m_cursor_on  = false;
	m_blink_on   = false;
	m_shift_on   = false;
	m_direction  = 1;
	m_data_len   = 8;
	m_num_line   = 1;
	m_char_size  = 8;
	m_disp_shift = 0;
	m_blink      = false;
	m_nibble     = false;
	m_first_cmd  = true;
	m_rs_state   = 0;
	m_rw_state   = 0;

	set_busy_flag(1520);
}

//-------------------------------------------------
//  device validity check
//-------------------------------------------------
void hd44780_device::device_validity_check(validity_checker &valid) const
{
	if (clock() == 0)
		osd_printf_error("LCDC clock cannot be zero!\n");
}

//-------------------------------------------------
//  timer events
//-------------------------------------------------

TIMER_CALLBACK_MEMBER(hd44780_device::clear_busy_flag)
{
	m_busy_flag = false;
}

TIMER_CALLBACK_MEMBER(hd44780_device::blink_tick)
{
	m_blink = !m_blink;
}


//**************************************************************************
//  HELPERS
//**************************************************************************

void hd44780_device::set_busy_flag(uint16_t usec)
{
	m_busy_flag = true;

	usec = float(usec) * m_busy_factor + 0.5;
	m_busy_timer->adjust(attotime::from_usec(usec));
}

void hd44780_device::correct_ac()
{
	if (m_active_ram == DDRAM)
	{
		int max_ac = (m_num_line == 1) ? 0x4f : 0x67;

		if (m_ac > max_ac)
			m_ac -= max_ac + 1;
		else if (m_ac < 0)
			m_ac = max_ac;
		else if (m_num_line == 2 && m_ac > 0x27 && m_ac < 0x40)
			m_ac = 0x40 + (m_ac - 0x28);
	}
	else
		m_ac &= 0x3f;
}

void hd44780_device::update_ac(int direction)
{
	if (m_active_ram == DDRAM && m_num_line == 2 && direction == -1 && m_ac == 0x40)
		m_ac = 0x27;
	else
		m_ac += direction;

	correct_ac();
}

void hd44780_device::shift_display(int direction)
{
	m_disp_shift += direction;

	if (m_disp_shift == 0x50)
		m_disp_shift = 0;
	else if (m_disp_shift == -1)
		m_disp_shift = 0x4f;
}

void hd44780_device::update_nibble(int rs, int rw)
{
	if (m_rs_state != rs || m_rw_state != rw)
	{
		m_rs_state = rs;
		m_rw_state = rw;
		m_nibble = false;
	}

	m_nibble = !m_nibble;
}

inline void hd44780_device::pixel_update(bitmap_ind16 &bitmap, u8 line, u8 pos, u8 y, u8 x, int state)
{
	if (!m_pixel_update_cb.isnull())
	{
		m_pixel_update_cb(bitmap, line, pos, y, x, state);
	}
	else
	{
		u8 line_height = (m_char_size == 8) ? m_char_size : m_char_size + 1;

		if (m_lines <= 2)
		{
			if (pos < m_chars)
				bitmap.pix(line * (line_height + 1) + y, pos * 6 + x) = state;
		}
		else if (m_lines <= 4)
		{
			if (pos < m_chars*2)
			{
				if (pos >= m_chars)
				{
					line += 2;
					pos -= m_chars;
				}

				if (line < m_lines)
					bitmap.pix(line * (line_height + 1) + y, pos * 6 + x) = state;
			}
		}
		else
		{
			fatalerror("%s: use a custom callback for this LCD configuration (%d x %d)\n", tag(), m_lines, m_chars);
		}
	}
}


//**************************************************************************
//  device interface
//**************************************************************************

const u8 *hd44780_device::render()
{
	memset(m_render_buf, 0, sizeof(m_render_buf));

	if (m_display_on)
	{
		u8 line_size = 80 / m_num_line;

		for (int line = 0; line < m_num_line; line++)
		{
			for (int pos = 0; pos < line_size; pos++)
			{
				uint16_t char_pos = line * 0x40 + ((pos + m_disp_shift) % line_size);

				int char_base;
				if (m_ddram[char_pos] < 0x10)
				{
					// draw CGRAM characters
					if (m_char_size == 8)
						char_base = (m_ddram[char_pos] & 0x07) * 8;
					else
						char_base = ((m_ddram[char_pos] >> 1) & 0x03) * 16;
				}
				else
				{
					// draw CGROM characters
					char_base = m_ddram[char_pos] * 0x10;
				}

				const u8 *charset = (m_ddram[char_pos] < 0x10) ? m_cgram : m_cgrom;
				u8 *dest = m_render_buf + 16 * (line * line_size + pos);
				memcpy (dest, charset + char_base, m_char_size);

				if (char_pos == m_ac)
				{
					// draw the cursor
					if (m_cursor_on)
						dest[m_char_size - 1] = 0x1f;

					if (!m_blink && m_blink_on)
						memset(dest, 0x1f, m_char_size);
				}
			}
		}
	}

	return m_render_buf;
}

uint32_t hd44780_device::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	bitmap.fill(0, cliprect);
	const u8 *img = render();

	u8 line_size = 80 / m_num_line;

	for (int line = 0; line < m_num_line; line++)
	{
		for (int pos = 0; pos < line_size; pos++)
		{
			const u8 *src = img + 16 * (line * line_size + pos);
			for (int y = 0; y < m_char_size; y++)
				for (int x = 0; x < 5; x++)
					pixel_update(bitmap, line, pos, y, x, BIT(src[y], 4 - x));
		}
	}

	return 0;
}

u8 hd44780_device::read(offs_t offset)
{
	if (m_data_len == 4 && !machine().side_effects_disabled())
		update_nibble(offset & 0x01, 1);

	switch (offset & 0x01)
	{
		case 0: return control_read();
		case 1: return data_read();
	}

	return 0;
}

void hd44780_device::write(offs_t offset, u8 data)
{
	if (m_data_len == 4 && !machine().side_effects_disabled())
		update_nibble(offset & 0x01, 0);

	switch (offset & 0x01)
	{
		case 0: control_write(data);  break;
		case 1: data_write(data);     break;
	}
}

u8 hd44780_device::db_r()
{
	if (m_enabled && m_rw_input == 1)
	{
		switch (m_rs_input)
		{
			case 0: return control_read();
			case 1: return data_read();
		}
	}

	return 0xff;
}

void hd44780_device::db_w(u8 data)
{
	m_db_input = data;
}

void hd44780_device::rs_w(int state)
{
	m_rs_input = state;
}

void hd44780_device::rw_w(int state)
{
	m_rw_input = state;
}

void hd44780_device::e_w(int state)
{
	if (m_data_len == 4 && state && !m_enabled && !machine().side_effects_disabled())
		update_nibble(m_rs_input, m_rw_input);

	if (!state && m_enabled && m_rw_input == 0)
	{
		switch (m_rs_input)
		{
			case 0: control_write(m_db_input);  break;
			case 1: data_write(m_db_input);     break;
		}
	}

	m_enabled = state;
}

void hd44780_device::control_write(u8 data)
{
	if (m_data_len == 4)
	{
		if (m_nibble)
		{
			m_ir = data & 0xf0;
			return;
		}
		else
		{
			m_ir |= ((data >> 4) & 0x0f);
		}
	}
	else
	{
		m_ir = data;
	}

	if (BIT(m_ir, 7))
	{
		// set DDRAM address
		m_active_ram = DDRAM;
		m_ac = m_ir & 0x7f;
		correct_ac();
		set_busy_flag(37);

		LOG("HD44780: set DDRAM address %x\n", m_ac);
		return;
	}
	else if (BIT(m_ir, 6))
	{
		// set CGRAM address
		m_active_ram = CGRAM;
		m_ac = m_ir & 0x3f;
		set_busy_flag(37);

		LOG("HD44780: set CGRAM address %x\n", m_ac);
		return;
	}
	else if (BIT(m_ir, 5))
	{
		// function set
		if (!m_function_set_at_any_time && !m_first_cmd && m_data_len == (BIT(m_ir, 4) ? 8 : 4) && (m_char_size != (BIT(m_ir, 2) ? 10 : 8) || m_num_line != (BIT(m_ir, 3) + 1)))
		{
			logerror("HD44780: function set cannot be executed after other instructions unless the interface data length is changed\n");
			return;
		}
		m_first_cmd = true;

		m_char_size = BIT(m_ir, 2) ? 10 : 8;
		m_data_len  = BIT(m_ir, 4) ? 8 : 4;
		m_num_line  = BIT(m_ir, 3) + 1;
		correct_ac();
		set_busy_flag(37);

		LOG("HD44780: char size 5x%d, data len %d, lines %d\n", m_char_size, m_data_len, m_num_line);
		return;
	}
	else if (BIT(m_ir, 4))
	{
		// cursor or display shift
		int direction = (BIT(m_ir, 2)) ? +1 : -1;

		LOG("HD44780: %s shift %d\n", BIT(m_ir, 3) ? "display" : "cursor", direction);

		if (BIT(m_ir, 3))
			shift_display(direction);
		else
			update_ac(direction);

		set_busy_flag(37);
	}
	else if (BIT(m_ir, 3))
	{
		// display on/off control
		m_display_on = BIT(m_ir, 2);
		m_cursor_on  = BIT(m_ir, 1);
		m_blink_on   = BIT(m_ir, 0);
		set_busy_flag(37);

		LOG("HD44780: display %d, cursor %d, blink %d\n", m_display_on, m_cursor_on, m_blink_on);
	}
	else if (BIT(m_ir, 2))
	{
		// entry mode set
		m_direction = (BIT(m_ir, 1)) ? +1 : -1;
		m_shift_on  = BIT(m_ir, 0);
		set_busy_flag(37);

		LOG("HD44780: entry mode set: direction %d, shift %d\n", m_direction, m_shift_on);
	}
	else if (BIT(m_ir, 1))
	{
		// return home
		LOG("HD44780: return home\n");

		m_ac         = 0;
		m_active_ram = DDRAM;
		m_direction  = 1;
		m_disp_shift = 0;
		set_busy_flag(1520);
	}
	else if (BIT(m_ir, 0))
	{
		// clear display
		LOG("HD44780: clear display\n");

		m_ac         = 0;
		m_active_ram = DDRAM;
		m_direction  = 1;
		m_disp_shift = 0;
		memset(m_ddram, 0x20, sizeof(m_ddram));
		set_busy_flag(1520);

		// Some machines do a "clear display" first, even though the datasheet insists "function set" must come before all else
		return;
	}

	m_first_cmd = false;
}

u8 hd44780_device::control_read()
{
	if (m_data_len == 4)
	{
		if (m_nibble)
			return (m_busy_flag ? 0x80 : 0) | (m_ac & 0x70);
		else
			return (m_ac << 4) & 0xf0;
	}
	else
	{
		return (m_busy_flag ? 0x80 : 0) | (m_ac & 0x7f);
	}
}

void hd44780_device::data_write(u8 data)
{
	if (m_busy_flag)
	{
		logerror("HD44780: Ignoring data write %02x due of busy flag\n", data);
		return;
	}

	if (m_data_len == 4)
	{
		if (m_nibble)
		{
			m_dr = data & 0xf0;
			return;
		}
		else
		{
			m_dr |= ((data >> 4) & 0x0f);
		}
	}
	else
	{
		m_dr = data;
	}

	LOG("HD44780: %sRAM write %x %x '%c'\n", m_active_ram == DDRAM ? "DD" : "CG", m_ac, m_dr, isprint(m_dr) ? m_dr : '.');

	if (m_active_ram == DDRAM)
		m_ddram[m_ac] = m_dr;
	else
		m_cgram[m_ac] = m_dr;

	update_ac(m_direction);
	if (m_shift_on)
		shift_display(m_direction);
	set_busy_flag(41);
}

u8 hd44780_device::data_read()
{
	u8 data = (m_active_ram == DDRAM) ? m_ddram[m_ac] : m_cgram[m_ac];

	LOG("HD44780: %sRAM read %x %c\n", m_active_ram == DDRAM ? "DD" : "CG", m_ac, data);

	if (m_data_len == 4)
	{
		if (m_nibble)
			return data & 0xf0;
		else
			data = (data << 4) & 0xf0;
	}

	if (!machine().side_effects_disabled())
	{
		update_ac(m_direction);
		set_busy_flag(41);
	}

	return data;
}
