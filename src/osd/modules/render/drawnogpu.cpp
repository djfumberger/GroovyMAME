// license:BSD-3-Clause
// copyright-holders:Aaron Giles, Antonio Giner, Sergi Clara
//============================================================
//
//  drawgnopu.cpp - nogpu drawing
//
//============================================================

#include "render_module.h"

#include "modules/osdmodule.h"

#include "window.h"

// emu
#include "emu.h"
#include "rendersw.hxx"


#include "render_module.h"

#ifdef _WIN32
#include <windows.h>
#include <winsock2.h>
#define socklen_t int
#else
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

// MAMEOS headers
#if defined(OSD_WINDOWS)
#include "winmain.h"
#elif defined(OSD_SDL)
#include "osdsdl.h"
#endif

#include <lz4/lz4.h>
#include <lz4/lz4hc.h>

#include <switchres/switchres.h>

#define MAX_BUFFER_WIDTH 768
#define MAX_BUFFER_HEIGHT 576
#define VRAM_BUFFER_SIZE 65536
#define SEND_BIFFER_SIZE 2097152 //2 * 1024 * 1024
#define MAX_LZ4_BLOCK   61440
#define MAX_SAMPLE_RATE 48000
#define STREAMS_UPDATE_FREQUENCY 50 // sound.h

// nogpu UDP server
#define UDP_PORT 32100

// Server commands
#define CMD_CLOSE 1
#define CMD_INIT 2
#define CMD_SWITCHRES 3
#define CMD_AUDIO 4
#define CMD_GET_STATUS 5
#define CMD_BLIT_VSYNC 6

// Status bits
#define VRAM_READY       1 << 0
#define VRAM_END_FRAME   1 << 1
#define VRAM_SYNCED      1 << 2
#define VRAM_FRAMESKIP   1 << 3
#define VGA_VBLANK       1 << 4
#define VGA_FIELD        1 << 5
#define FPGA_AUDIO       1 << 6
#define VGA_QUEUE        1 << 7

#pragma pack(1)

typedef struct nogpu_modeline
{
	double    pclock;
	uint16_t  hactive;
	uint16_t  hbegin;
	uint16_t  hend;
	uint16_t  htotal;
	uint16_t  vactive;
	uint16_t  vbegin;
	uint16_t  vend;
	uint16_t  vtotal;
	uint8_t  interlace;
} nogpu_modeline;

typedef struct nogpu_status
{
	uint16_t vcount;
	uint32_t frame_num;
} nogpu_status;

typedef struct nogpu_blit_status
{
	uint32_t frame_req;
	uint16_t vcount_req;
	uint32_t frame_gpu;
	uint16_t vcount_gpu;
	uint8_t bits;
} nogpu_blit_status;

typedef struct cmd_init
{
	const uint8_t cmd = CMD_INIT;
	uint8_t	compression;
	uint8_t sound_rate;
	uint8_t sound_channels;
} cmd_init;

typedef struct cmd_close
{
	const uint8_t cmd = CMD_CLOSE;
} cmd_close;

typedef struct cmd_switchres
{
	const uint8_t cmd = CMD_SWITCHRES;
	nogpu_modeline mode;
} cmd_switchres;

typedef struct cmd_audio
{
	const uint8_t cmd = CMD_AUDIO;
	uint16_t sample_bytes;
} cmd_blit;

typedef struct cmd_blit_vsync
{
	const uint8_t cmd = CMD_BLIT_VSYNC;
	uint32_t frame;
	uint16_t vsync;
	uint16_t block_size;
} cmd_blit_vsync;


typedef struct cmd_get_status
{
	const uint8_t cmd = CMD_GET_STATUS;
} cmd_get_status;

#pragma pack(0)


namespace osd {

namespace {

// renderer_nogpu is the information for the current screen
class renderer_nogpu : public osd_renderer
{
public:
	renderer_nogpu(osd_window &window)
		: osd_renderer(window)
		, m_bmdata(nullptr)
		, m_bmsize(0)
	{
	}

	virtual ~renderer_nogpu();
	virtual int create() override;
	virtual render_primitive_list *get_primitives() override;
	virtual int draw(const int update) override;
	virtual void save() override {}
	virtual void record() override {}
	virtual void toggle_fsfx() override {}
	virtual void add_audio_to_recording(const int16_t *buffer, int samples_this_frame) override;

private:
#if defined(OSD_WINDOWS)
	BITMAPINFO                  m_bminfo;
#endif
	std::unique_ptr<uint8_t []> m_bmdata;
	size_t                      m_bmsize;

	// npgpu private members
	bool m_initialized = false;
	bool m_first_blit = true;
	int m_compression = 0;
	bool m_show_window = false;
	bool m_is_internal_fe = false;
	bool m_autofilter = false;
	bool m_bilinear = false;
	int m_frame = 0;
	int m_field = 0;
	int m_width = 0;
	int m_height = 0;
	int m_vtotal = 0;
	int m_vsync_scanline = 0;
	bool m_sleep_allowed = false;
	double m_period = 16.666667;
	double m_line_period = 0.064;
	double m_frame_delay = 0.0;
	double m_fd_margin = 1.5;
	float m_aspect = 4.0 / 3.0;
	float m_pixel_aspect = 1.0;
	int m_sample_rate = MAX_SAMPLE_RATE;
	nogpu_status m_status;
	nogpu_blit_status m_blit_status;
	modeline m_current_mode;

	osd_ticks_t time_start = 0;
	osd_ticks_t time_entry = 0;
	osd_ticks_t time_blit = 0;
	osd_ticks_t time_exit = 0;
	osd_ticks_t time_frame[16];
	osd_ticks_t time_frame_avg = 0;
	osd_ticks_t time_frame_dm = 0;
	osd_ticks_t time_sleep = 1 * osd_ticks_per_second() / 1000.0; // 1 ms

	int m_sockfd = -1; //INVALID_SOCKET;
	sockaddr_in m_server_addr;

	char m_fb[MAX_BUFFER_HEIGHT * MAX_BUFFER_WIDTH * 3];
	char m_fb_compressed[MAX_BUFFER_HEIGHT * MAX_BUFFER_WIDTH * 3];
	char inp_buf[2][MAX_LZ4_BLOCK + 1];
	char m_ab[MAX_SAMPLE_RATE / STREAMS_UPDATE_FREQUENCY * 2 * 2];

	bool nogpu_init();
	bool nogpu_send_command(void *command, int command_size);
	bool nogpu_switch_video_mode();
	void nogpu_blit(uint32_t frame, uint16_t vsync, uint16_t line_width);
	void nogpu_send_mtu(char *buffer, int bytes_to_send, int chunk_max_size);
	void nogpu_send_lz4(char *buffer, int bytes_to_send, int block_size);
	int nogpu_compress(int id_compress, char *buffer_comp, const char *buffer_rgb, uint32_t buffer_size);
	bool nogpu_wait_ack(double timeout);
	bool nogpu_wait_status(nogpu_blit_status *status, double timeout);
	void nogpu_register_frametime(osd_ticks_t frametime);
};

inline double get_ms(osd_ticks_t ticks) { return (double) ticks / osd_ticks_per_second() * 1000; };

//============================================================
//  renderer_nogpu::create
//============================================================

int renderer_nogpu::create()
{
	#if defined(OSD_WINDOWS)
		// fill in the bitmap info header
		m_bminfo.bmiHeader.biSize            = sizeof(m_bminfo.bmiHeader);
		m_bminfo.bmiHeader.biPlanes          = 1;
		m_bminfo.bmiHeader.biBitCount        = 32;
		m_bminfo.bmiHeader.biCompression     = BI_RGB;
		m_bminfo.bmiHeader.biSizeImage       = 0;
		m_bminfo.bmiHeader.biXPelsPerMeter   = 0;
		m_bminfo.bmiHeader.biYPelsPerMeter   = 0;
		m_bminfo.bmiHeader.biClrUsed         = 0;
		m_bminfo.bmiHeader.biClrImportant    = 0;
	#endif

	return 0;
}

//============================================================
//  renderer_nogpu::~renderer_nogpu
//============================================================

renderer_nogpu::~renderer_nogpu()
{
	osd_printf_verbose("nogpu: Sending CMD_CLOSE...");
	cmd_close command;

	nogpu_send_command(&command, sizeof(command));
	osd_printf_verbose("done.\n");

#ifdef WIN32
	closesocket(m_sockfd);
	WSACleanup();
#else
	close(m_sockfd);
#endif
}

//============================================================
//  renderer_nogpu::get_primitives
//============================================================

render_primitive_list *renderer_nogpu::get_primitives()
{
	if (m_width == 0 || m_height == 0)
	{
		osd_dim const dimensions = window().get_size();
		if ((dimensions.width() <= 0) || (dimensions.height() <= 0))
			return nullptr;

		m_width = std::min(dimensions.width(), MAX_BUFFER_WIDTH);
		m_height = std::min(dimensions.height(), MAX_BUFFER_HEIGHT);
	}

	window().target()->set_bounds(m_width, m_height, m_pixel_aspect);
	return &window().target()->get_primitives();
}

//============================================================
//  renderer_nogpu::draw
//============================================================

int renderer_nogpu::draw(const int update)
{
	#if defined(OSD_WINDOWS)
		auto &win = dynamic_cast<win_window_info &>(window());
	#elif defined(OSD_SDL)
		auto &win = dynamic_cast<sdl_window_info &>(window());
	#endif

	#if defined(OSD_WINDOWS)
	// we don't have any special resize behaviors
	if (win.m_resize_state == win_window_info::RESIZE_STATE_PENDING)
		win.m_resize_state = win_window_info::RESIZE_STATE_NORMAL;
	#endif

	// resize window if required
	static int old_width = 0;
	static int old_height = 0;
	if (old_width != m_width || old_height != m_height)
	{
		old_width = m_width;
		old_height = m_height;

		#if defined(OSD_WINDOWS)
		if (m_show_window)
		{
			RECT client_rect;
			RECT window_rect;
			GetClientRect(win.platform_window(), &client_rect);
			GetWindowRect(win.platform_window(), &window_rect);

			int extra_width = (window_rect.right - window_rect.left) - (client_rect.right - client_rect.left);
			int extra_height = (window_rect.bottom - window_rect.top) - (client_rect.bottom - client_rect.top);

			SetWindowPos(win.platform_window(), nullptr, 0, 0, m_width + extra_width, m_height + extra_height, SWP_NOMOVE | SWP_NOZORDER);
		}
		#endif
	}

	// compute pitch of target
	int const pitch = (m_width + 3) & ~3;

	// make sure our temporary bitmap is big enough
	if ((pitch * m_height * 4) > m_bmsize)
	{
		m_bmsize = pitch * m_height * 4 * 2;
		m_bmdata.reset();
		m_bmdata = std::make_unique<uint8_t []>(m_bmsize);
	}

	// draw the primitives to the bitmap
	win.m_primlist->acquire_lock();
	if (m_bilinear)
		software_renderer<uint32_t, 0,0,0, 16,8,0,0, 1>::draw_primitives(*win.m_primlist, m_bmdata.get(), m_width, m_height, pitch);
	else
		software_renderer<uint32_t, 0,0,0, 16,8,0,0, 0>::draw_primitives(*win.m_primlist, m_bmdata.get(), m_width, m_height, pitch);
	win.m_primlist->release_lock();

	// blit to the screen
	#if defined(OSD_WINDOWS)
	if (m_show_window)
	{
		// fill in bitmap-specific info
		m_bminfo.bmiHeader.biWidth = pitch;
		m_bminfo.bmiHeader.biHeight = -m_height;

		StretchDIBits(
				win.m_dc, 0, 0, m_width, m_height,
				0, 0, m_width, m_height,
				m_bmdata.get(), &m_bminfo, DIB_RGB_COLORS, SRCCOPY);
	}
	#endif

	// initialize nogpu right before first blit
	if (m_first_blit && !m_initialized)
	{
		m_initialized = nogpu_init();
		if (m_initialized)
			osd_printf_verbose("done.\n");
		else
		{
			osd_printf_verbose("failed.\n");
			window().machine().video().set_throttled(true);
			window().machine().video().set_sync_refresh(false);
			m_first_blit = false;
		}
	}

	// only send frame if nogpu is initialized
	if (!m_initialized)
		return 0;

	// get current field for interlaced mode
	if (m_current_mode.interlace)
		m_field = (m_blit_status.bits & VGA_FIELD? 1 : 0) ^ ((m_frame - m_blit_status.frame_gpu) % 2);

	// convert RGBA buffer to RGB
	int i = 0, j = 0, k = 0;
	int lstart = pitch * 4 * m_field;
	int lend = (m_height - 1) * pitch * 4;
	int lstep = pitch * 4 * (m_current_mode.interlace? 2 : 1);

	for (i = lstart; i <= lend ; i += lstep)
	{
		for (j = 0; j < pitch * 4; j += 4)
		{
			m_fb[k] = (char)m_bmdata[i+j];
			m_fb[k+1] = (char)m_bmdata[i+j+1];
			m_fb[k+2] = (char)m_bmdata[i+j+2];
			k += 3;
		}
	}

	// change video mode right before the blit
	if (m_initialized) nogpu_switch_video_mode();

	bool valid_status = false;

	time_entry = osd_ticks();

	if (video_config.syncrefresh && m_first_blit)
	{
		time_start = time_entry;
		time_blit = time_entry;
		time_exit = time_entry;

		m_first_blit = false;
		m_frame = 1;
	}

	// Blit now
	nogpu_blit(m_frame, m_width, m_height / (m_current_mode.interlace? 2 : 1));

	time_blit = osd_ticks();
	osd_printf_verbose("[%.3f] frame: %d emulation_time: %.3f blit_time: %.3f \n",
			get_ms(time_entry - time_start), m_frame,get_ms(time_entry - time_exit), get_ms(time_blit - time_entry));

	nogpu_register_frametime(time_entry - time_exit);

	// Wait raster position
	if (video_config.syncrefresh)
		valid_status = nogpu_wait_status(&m_blit_status, std::max(0.0, m_period - get_ms(time_blit - time_exit)));

	if (video_config.syncrefresh && valid_status)
		m_frame = m_blit_status.frame_req + 1;
	else
		m_frame ++;

	time_exit = osd_ticks();
	osd_printf_verbose("[%.3f] ft_avg %.3f Dm: %.3f fd: %.3f wait_time: %.3f\n\n",
		get_ms(time_exit - time_start), get_ms(time_frame_avg), get_ms(time_frame_dm), m_frame_delay * 10.0, get_ms(time_exit - time_blit));

	return 0;
}

//============================================================
//  renderer_nogpu::nogpu_init
//============================================================

bool renderer_nogpu::nogpu_init()
{
	int result;

	#if defined(OSD_WINDOWS)
		windows_options &options = downcast<windows_options &>(window().machine().options());
	#elif defined(OSD_SDL)
		sdl_options &options = downcast<sdl_options &>(window().machine().options());
	#endif

	#ifdef _WIN32
		osd_printf_verbose("nogpu: Initializing Winsock...");
		WSADATA wsa;
		result = WSAStartup(MAKEWORD(2, 2), &wsa);
		if (result != NO_ERROR)
		{
			osd_printf_verbose("Failed. Error code : %d", WSAGetLastError());
			return false;
		}
		osd_printf_verbose("done.\n");
	#endif

	osd_printf_verbose("nogpu: Initializing socket... ");
	m_sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	if (m_sockfd < 0)
	{
		osd_printf_verbose("Could not create socket\n");
		return false;
	}
	else
		osd_printf_verbose(" done.\n");

	short port = UDP_PORT;
	const char* local_host = options.mister_ip();

	m_server_addr = {};
	m_server_addr.sin_family = AF_INET;
	m_server_addr.sin_port = htons(port);
	m_server_addr.sin_addr.s_addr = inet_addr(local_host);

	osd_printf_verbose("nogpu: Setting socket async...\n");

	#ifdef _WIN32
		u_long opt = 1;
		if (ioctlsocket(m_sockfd, FIONBIO, &opt) < 0)
			osd_printf_verbose("Could not set nonblocking.\n");
	#else
		int flags;
		flags = fcntl(m_sockfd, F_GETFD, 0);
		if (flags < 0)
			osd_printf_verbose("Could not get socket flags.\n");
		else
		{
			flags |= O_NONBLOCK;
			if (fcntl(m_sockfd, F_SETFL, flags) < 0)
				osd_printf_verbose("Could not set nonblocking.\n");
		}
	#endif

	osd_printf_verbose("nogpu: Setting send buffer to %d bytes...\n", SEND_BIFFER_SIZE);
	int opt_val = SEND_BIFFER_SIZE;
	result = setsockopt(m_sockfd, SOL_SOCKET, SO_SNDBUF, (char*)&opt_val, sizeof(opt_val));
	if (result < 0)
	{
		osd_printf_verbose("Unable to set send buffer: %d\n", result);
		return false;
	}

	m_compression = 0; // raw
	const char* compression = options.mister_compression();
	if (!strcmp(compression, "lz4"))
	{
		m_compression = 0x01;
		osd_printf_verbose("nogpu: compression algorithm %s\n", compression);
	}
	else if (strcmp(compression, "none"))
		osd_printf_verbose("nogpu: compression algorithm %s not supported\n", compression);

	switch(options.sample_rate())
	{
		case 22050:
			m_sample_rate = 1;
			break;
		case 44100:
			m_sample_rate = 2;
			break;
		case 48000:
			m_sample_rate = 3;
			break;
		default:
			m_sample_rate = 0;
			osd_printf_verbose("nogpu: not valid sample rate %d\n", options.sample_rate());
	}

	osd_printf_verbose("nogpu: Sending CMD_INIT...");
	cmd_init command;
	command.compression = m_compression;
	command.sound_rate = m_sample_rate;
	command.sound_channels = 2;

	// Reset current mode
	m_current_mode = {};

	// Hide window optionally
	m_show_window = options.mister_window();
	#if defined(OSD_WINDOWS)
		if (!m_show_window)
		{
			auto &win = dynamic_cast<win_window_info &>(window());
			SetWindowLong(win.platform_window(), GWL_EXSTYLE, WS_EX_LAYERED);
			SetLayeredWindowAttributes(win.platform_window(), 0, 0, LWA_ALPHA);
		}
	#endif

	m_fd_margin = (double)options.mister_fd_margin();
	m_sleep_allowed = options.sleep();
	m_autofilter = options.autofilter();
	m_is_internal_fe = strcmp(window().machine().system().name, "___empty") == 0;

	if (nogpu_send_command(&command, sizeof(command)))
		return nogpu_wait_ack(1000);

	return false;
}

//============================================================
//  renderer_nogpu::nogpu_switch_video_mode()
//============================================================

bool renderer_nogpu::nogpu_switch_video_mode()
{
	#if defined(OSD_WINDOWS)
		switchres_manager *m_switchres = &downcast<windows_osd_interface&>(window().machine().osd()).switchres()->switchres();
	#elif defined(OSD_SDL)
		switchres_manager *m_switchres = &downcast<sdl_osd_interface&>(window().machine().osd()).switchres()->switchres();
	#endif
	display_manager *display = m_switchres->display(window().index());

	// Check if we have a pending mode change
	if (display == nullptr)
		return false;

	modeline *mode = display->selected_mode();
	if (mode == nullptr)
		return false;

	// Make sure SR's mode isn't bigger than our buffer
	if (mode->hactive > MAX_BUFFER_WIDTH || mode->vactive > MAX_BUFFER_HEIGHT)
	{
		// Force 640x480 otherwise
		display->get_mode(640, 480, mode->vfreq, 0);
		if (!display->got_mode())
			return false;

		mode = display->selected_mode();
	}

	// If not, we're done
	if (!modeline_is_different(mode, &m_current_mode))
		return true;

	m_current_mode = *mode;

	// Send new modeline to nogpu
	osd_printf_verbose("nogpu: Sending CMD_SWITCHRES...\n");

	cmd_switchres command;
	nogpu_modeline *m = &command.mode;

	m->pclock    = double(mode->pclock) / 1000000.0;
	m->hactive   = mode->hactive;
	m->hbegin    = mode->hbegin;
	m->hend      = mode->hend;
	m->htotal    = mode->htotal;
	m->vactive   = mode->vactive;
	m->vbegin    = mode->vbegin;
	m->vend      = mode->vend;
	m->vtotal    = mode->vtotal;
	m->interlace = mode->interlace;

	m_width = mode->hactive;
	m_height = mode->vactive;
	m_vtotal = mode->vtotal;
	m_period = 1000.0 / (double(mode->pclock) / (mode->htotal * mode->vtotal)) / (mode->interlace? 2: 1);
	m_line_period = 1000.0 / mode->hfreq;
	m_field = 0;
	m_bilinear = m_autofilter & display->is_stretched();

	m_aspect = display->monitor_aspect();
	m_pixel_aspect = m_aspect / ((float)m_width / m_height);

	return nogpu_send_command(&command, sizeof(command));
}

//============================================================
//  renderer_nogpu::nogpu_wait_ack
//============================================================

bool renderer_nogpu::nogpu_wait_ack(double timeout)
{
	osd_ticks_t time_1 = osd_ticks();
	socklen_t server_addr_size = sizeof(m_server_addr);

	// Poll server for ack
	do
	{
		int bytes_recv = recvfrom(m_sockfd, (char *)&m_status, sizeof(nogpu_blit_status), 0, (sockaddr*)&m_server_addr, &server_addr_size);

		if (bytes_recv == sizeof(nogpu_blit_status))
			break;

		osd_ticks_t time_2 = osd_ticks();
		if (get_ms(time_2 - time_1) > timeout)
		{
			osd_printf_verbose("timeout: %.3f ms\n", timeout);
			return false;
		}

		osd_sleep(time_sleep);

	} while (true);

	return true;
}

//============================================================
//  renderer_nogpu::nogpu_wait_status
//============================================================

bool renderer_nogpu::nogpu_wait_status(nogpu_blit_status *status, double timeout)
{
	int retries = 0;
	osd_ticks_t time_1 = 0;
	osd_ticks_t time_2 = 0;
	socklen_t server_addr_size = sizeof(m_server_addr);
	int bytes_recv = 0;

	osd_printf_verbose("[%.3f] wait_status[%.3f]: ", get_ms(osd_ticks() - time_start), timeout);
	time_1 = osd_ticks();

	// Poll server for blit line timestamp
	do
	{
		retries++;
		bytes_recv = recvfrom(m_sockfd, (char *)status, sizeof(nogpu_blit_status), 0, (sockaddr*)&m_server_addr, &server_addr_size);

		if (bytes_recv > 0 && m_frame == status->frame_req)
			break;

		time_2 = osd_ticks();
		if (get_ms(time_2 - time_1) > timeout)
		{
			osd_printf_verbose("timeout[%d]: %.3f\n", retries, timeout);
			return false;
		}

		if (m_sleep_allowed) osd_sleep(time_sleep);

	} while (true);


	// We have a valid timestamp
	osd_printf_verbose("success[%d] ack[%d][%d]->[%d][%d] ", retries, status->frame_req, status->vcount_req, status->frame_gpu, status->vcount_gpu);

	// Compute line target for next blit, relative to last blit line timestamp
	int lines_to_wait = (status->frame_req - status->frame_gpu) * m_current_mode.vtotal + status->vcount_req - status->vcount_gpu;
	if (m_current_mode.interlace)
		lines_to_wait /= 2;

	// Compute time target for emulation of next frame, so that blit after it happens at desired line target
	osd_ticks_t time_target = time_entry + (osd_ticks_t)((double)lines_to_wait * m_line_period * osd_ticks_per_second() / 1000.0) - time_frame_avg;

	// Wait for target time
	if ((int)(time_target - osd_ticks()) > 0)
	{
		osd_printf_verbose("to wait[%d|%.3f]\n", lines_to_wait, get_ms(time_target - osd_ticks()));

		do
		{
			time_2 = osd_ticks();
			if (time_2 >= time_target)
				break;

			if (m_sleep_allowed && get_ms(time_target - time_2) > 2.0)
				osd_sleep(time_sleep);

		} while (true);
	}
	else
		osd_printf_verbose("delayed, exiting\n");

	// Make sure our frame counter hasn't fallen behind gpu's
	if (status->frame_gpu > status->frame_req)
		status->frame_req = status->frame_gpu + 1;

	return true;
}

//============================================================
//  renderer_nogpu::nogpu_register_frametime
//============================================================

void renderer_nogpu::nogpu_register_frametime(osd_ticks_t frametime)
{
	static int i = 0;
	static int regs = 0;
	const int max_regs = sizeof(time_frame) / sizeof(time_frame[0]);
	osd_ticks_t acum = 0;
	int diff = 0;

	// Discard invalid values
	if (frametime <= 0 || get_ms(frametime) > m_period)
		return;

	// Register value and compute current average
	time_frame[i] = frametime;
	i++;

	if (i > max_regs)
		i = 0;

	if (regs < max_regs)
		regs++;

	for (int k = 0; k < regs; k++)
		acum += time_frame[k];

	time_frame_avg = acum / regs;

	// Compute current max deviation
	osd_ticks_t max_diff = 0;

	for (int k = 1; k <= regs; k++)
	{
		diff = time_frame[k] - time_frame[k-1];

		if (diff > 0 && diff > max_diff)
			max_diff = diff;
	}

	time_frame_dm = max_diff;
}

//============================================================
//  renderer_nogpu::nogpu_send_mtu
//============================================================

void renderer_nogpu::nogpu_send_mtu(char *buffer, int bytes_to_send, int chunk_max_size)
{
	int bytes_this_chunk = 0;
	int chunk_size = 0;
	uint32_t offset = 0;

	do
	{
		chunk_size = bytes_to_send > chunk_max_size? chunk_max_size : bytes_to_send;
		bytes_to_send -= chunk_size;
		bytes_this_chunk = chunk_size;

		nogpu_send_command(buffer + offset, bytes_this_chunk);
		offset += chunk_size;

	} while (bytes_to_send > 0);
}

//============================================================
//  renderer_nogpu::nogpu_send_lz4
//============================================================

void renderer_nogpu::nogpu_send_lz4(char *buffer, int bytes_to_send, int block_size)
{
	LZ4_stream_t lz4_stream_body;
	LZ4_stream_t* lz4_stream = &lz4_stream_body;
	LZ4_initStream(lz4_stream, sizeof(*lz4_stream));

	int inp_buf_index = 0;
	int bytes_this_chunk = 0;
	int chunk_size = 0;
	uint32_t offset = 0;

	do
	{
		chunk_size = bytes_to_send > block_size? block_size : bytes_to_send;
		bytes_to_send -= chunk_size;
		bytes_this_chunk = chunk_size;

		char* const inp_ptr = inp_buf[inp_buf_index];
		memcpy((char *)&inp_ptr[0], buffer + offset, chunk_size);

		const uint16_t c_size = LZ4_compress_fast_continue(lz4_stream, inp_ptr, (char *)&m_fb_compressed[2], bytes_this_chunk, MAX_LZ4_BLOCK, 1);
		uint16_t *c_size_ptr = (uint16_t *)&m_fb_compressed[0];
		*c_size_ptr = c_size;

		nogpu_send_mtu((char *) &m_fb_compressed[0], c_size + 2, 1472);
		offset += chunk_size;
		inp_buf_index ^= 1;

	} while (bytes_to_send > 0);
}

//============================================================
//  renderer_nogpu::nogpu_blit
//============================================================

void renderer_nogpu::nogpu_blit(uint32_t frame, uint16_t width, uint16_t height)
{
	// Compressed blocks are 16 lines long
	int block_size = m_compression? (width << 4) * 3 : 0;

	int vsync_offset = 0;

	// Calculate frame delay factor
	if (m_is_internal_fe)
		// Internal frontend needs fd > 0
		m_frame_delay = .5;

	else if (video_config.framedelay == 0)
		// automatic
		m_frame_delay = std::max((double)(m_period - std::max(m_fd_margin, get_ms(time_frame_dm))) / m_period, 0.0);
	else
	{
		// user defined
		m_frame_delay = (double)(video_config.framedelay) / 10.0;
		vsync_offset = window().machine().video().vsync_offset();
	}

	// Update vsync scanline
	m_vsync_scanline = std::min<int>((m_current_mode.vtotal) * m_frame_delay + vsync_offset + 1, m_current_mode.vtotal);

	// Send CMD_BLIT
	cmd_blit_vsync command;
	command.frame = frame;
	command.vsync = video_config.syncrefresh? m_vsync_scanline : 0;
	command.block_size = block_size;
	nogpu_send_command(&command, sizeof(command));

	if (m_compression == 0)
		nogpu_send_mtu(&m_fb[0], width * height * 3, 1470);

	else
		nogpu_send_lz4(&m_fb[0], width * height * 3, block_size);
}

//============================================================
//  renderer_nogpu::add_audio_to_recording
//============================================================

void renderer_nogpu::add_audio_to_recording(const int16_t *buffer, int samples_this_frame)
{
	if (m_blit_status.bits & FPGA_AUDIO && m_sample_rate)
	{
		osd_printf_verbose("audio samples sent: %d\n", samples_this_frame);

		// Send CMD_AUDIO
		cmd_audio command;
		command.sample_bytes = samples_this_frame << 2;
		nogpu_send_command(&command, sizeof(command));
		nogpu_send_mtu((char*)buffer, command.sample_bytes, 1472);
	}
}

//============================================================
//  renderer_nogpu::nogpu_send_command
//============================================================

bool renderer_nogpu::nogpu_send_command(void *command, int command_size)
{
	int rc = sendto(m_sockfd, (char *)command, command_size, 0, (sockaddr*)&m_server_addr, sizeof(m_server_addr));

	if (rc < 0)
	{
		osd_printf_verbose("nogpu_send_command failed.\n");
		return false;
	}

	return true;
}

//

class video_nogpu : public osd_module, public render_module
{
public:
	video_nogpu() : osd_module(OSD_RENDERER_PROVIDER, "mister") { }

	virtual int init(osd_interface &osd, osd_options const &options) override { return 0; }
	virtual void exit() override { }

	virtual std::unique_ptr<osd_renderer> create(osd_window &window) override;

protected:
	virtual unsigned flags() const override { return FLAG_INTERACTIVE; }
};

std::unique_ptr<osd_renderer> video_nogpu::create(osd_window &window)
{
	return std::make_unique<renderer_nogpu>(window);
}

} // anonymous namespace

} // namespace osd

MODULE_DEFINITION(RENDERER_MISTER, osd::video_nogpu)

