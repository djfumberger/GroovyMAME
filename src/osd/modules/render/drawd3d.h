// license:BSD-3-Clause
// copyright-holders:Aaron Giles
//============================================================
//
//  drawd3d.h - Win32 Direct3D header
//
//============================================================
#ifndef MAME_OSD_MODULES_RENDER_DRAWD3D_H
#define MAME_OSD_MODULES_RENDER_DRAWD3D_H

#pragma once

#include "d3d/d3dcomm.h"

#include "modules/lib/osdlib.h"
#include "modules/osdwindow.h"

#include "sliderdirtynotifier.h"

#include <windows.h>
#include <tchar.h>
#include <mmsystem.h>
#include <d3d9.h>
#include <d3dx9.h>
#undef interface

#include <memory>
#include <vector>
#include <cmath>


//============================================================
//  CONSTANTS
//============================================================

#define VERTEX_BASE_FORMAT  (D3DFVF_DIFFUSE | D3DFVF_TEX1 | D3DFVF_TEX2)
#define VERTEX_BUFFER_SIZE  (40960*4+4)

//============================================================
//  TYPE DEFINITIONS
//============================================================

class shaders;
struct hlsl_options;

/* renderer_d3d9 is the information about Direct3D for the current screen */
class renderer_d3d9 : public osd_renderer, public slider_dirty_notifier
{
public:
	using IDirect3D9Ptr = Microsoft::WRL::ComPtr<IDirect3D9Ex>;

	renderer_d3d9(osd_window &window, const IDirect3D9Ptr &d3dobj);
	virtual ~renderer_d3d9();

	virtual int create() override;
	virtual render_primitive_list *get_primitives() override;
	virtual int draw(const int update) override;
	virtual void save() override;
	virtual void record() override;
	virtual void toggle_fsfx() override;
	virtual void add_audio_to_recording(const int16_t *buffer, int samples_this_frame) override;
	virtual std::vector<ui::menu_item> get_slider_list() override;
	virtual void set_sliders_dirty() override;
	virtual int restart() override;

	int                     initialize();

	int                     device_create(HWND device_HWND);
	int                     device_create_resources();
	void                    device_delete();
	void                    device_delete_resources();
	void                    device_flush();
	void                    update_break_scanlines();
	void                    update_presentation_parameters();
	void                    update_gamma_ramp();

	bool                    device_verify_caps();
	int                     device_test_cooperative();

	int                     config_adapter_mode();
	void                    pick_best_mode();
	int                     get_adapter_for_monitor();

	bool                    update_window_size();

	int                     pre_window_draw_check();
	void                    begin_frame();
	void                    end_frame();

	void                    draw_line(const render_primitive &prim);
	void                    draw_quad(const render_primitive &prim);
	void                    batch_vector(const render_primitive &prim);
	void                    batch_vectors(int vector_count);

	vertex *                mesh_alloc(int numverts);

	void                    process_primitives();
	void                    primitive_flush_pending();

	void                    set_texture(texture_info *texture);
	void                    set_filter(int filter);
	void                    set_wrap(unsigned int wrap);
	void                    set_modmode(int modmode);
	void                    set_blendmode(int blendmode);
	void                    reset_render_states();

	// Setters / getters
	int                     get_adapter() const { return m_adapter; }
	int                     get_width() const { return m_width; }
	vec2f                   get_dims() const { return vec2f(m_width, m_height); }
	int                     get_height() const { return m_height; }
	int                     get_refresh() const { return m_refresh; }
	bool                    post_fx_available() const { return m_post_fx_available; }
	void                    set_post_fx_unavailable() { m_post_fx_available = false; }

	IDirect3DDevice9Ex *    get_device() const { return m_device.Get(); }
	D3DPRESENT_PARAMETERS * get_presentation() { return &m_presentation; }

	IDirect3DVertexBuffer9 *get_vertex_buffer() const { return m_vertexbuf.Get(); }

	void                    set_toggle(bool toggle) { m_toggle = toggle; }

	D3DFORMAT               get_screen_format() const { return m_screen_format; }
	D3DFORMAT               get_pixel_format() const { return m_pixformat; }
	D3DDISPLAYMODEEX        get_origmode() const { return m_origmode; }

	uint32_t                get_last_texture_flags() const { return m_last_texture_flags; }

	d3d_texture_manager *   get_texture_manager() const { return m_texture_manager.get(); }
	texture_info *          get_default_texture();

	shaders *               get_shaders() const { return m_shaders.get(); }

private:
	using IDirect3DDevice9Ptr = Microsoft::WRL::ComPtr<IDirect3DDevice9Ex>;
	using IDirect3DVertexBuffer9Ptr = Microsoft::WRL::ComPtr<IDirect3DVertexBuffer9>;

	const IDirect3D9Ptr     m_d3dobj;                   // Direct3D 9 API object
	int                     m_adapter;                  // ordinal adapter number
	int                     m_vendor_id;                // adapter vendor id
	int                     m_width;                    // current width
	int                     m_height;                   // current height
	int                     m_refresh;                  // current refresh rate
	bool                    m_interlace;                // current interlace
	int                     m_frame_delay;              // current frame delay value
	int                     m_vsync_offset;             // current vsync_offset value
	int                     m_first_scanline;           // first scanline number (visible)
	int                     m_last_scanline;            // last scanline number (visible)
	int                     m_delay_scanline;           // scanline number supposed to be after frame delay
	int                     m_break_scanline;           // break scanline number, for vsync offset
	int                     m_create_error_count;       // number of consecutive create errors
	bool                    m_post_fx_available;

	IDirect3DDevice9Ptr     m_device;                   // pointer to the Direct3DDevice object
	int                     m_gamma_supported;          // is full screen gamma supported?
	D3DPRESENT_PARAMETERS   m_presentation;             // set of presentation parameters
	D3DDISPLAYMODEEX        m_origmode;                 // original display mode for the adapter
	D3DDISPLAYMODEEX        m_display_mode;             // full screen display mode
	D3DFORMAT               m_pixformat;                // pixel format we are using
	IDirect3DQuery9 *		m_query;
	IDirect3DSwapChain9 *   m_swap9;
	IDirect3DSwapChain9Ex * m_swap;
	D3DPRESENTSTATS         m_stats;
	D3DRASTER_STATUS        m_raster_status;
	int                     m_sync_count;
	int                     m_enter_line;
	int                     m_exit_line;

	IDirect3DVertexBuffer9Ptr m_vertexbuf;              // pointer to the vertex buffer object
	vertex *                m_lockedbuf;                // pointer to the locked vertex buffer
	int                     m_numverts;                 // number of accumulated vertices

	vertex *                m_vectorbatch;              // pointer to the vector batch buffer
	int                     m_batchindex;               // current index into the vector batch

	poly_info               m_poly[VERTEX_BUFFER_SIZE/3];// array to hold polygons as they are created
	int                     m_numpolys;                 // number of accumulated polygons

	bool                    m_toggle;                   // if we're toggle fsfx

	D3DFORMAT               m_screen_format;            // format to use for screen textures

	texture_info *          m_last_texture;             // previous texture
	uint32_t                m_last_texture_flags;       // previous texture flags
	int                     m_last_blendenable;         // previous blendmode
	int                     m_last_blendop;             // previous blendmode
	int                     m_last_blendsrc;            // previous blendmode
	int                     m_last_blenddst;            // previous blendmode
	int                     m_last_filter;              // previous texture filter
	uint32_t                m_last_wrap;                // previous wrap state
	int                     m_last_modmode;             // previous texture modulation

	std::unique_ptr<shaders> m_shaders;                 // HLSL interface

	std::unique_ptr<d3d_texture_manager> m_texture_manager;          // texture manager
};

#endif // MAME_OSD_MODULES_RENDER_DRAWD3D_H
