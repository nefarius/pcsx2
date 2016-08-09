/*
 *	Copyright (C) 2011-2011 Gregory hainaut
 *	Copyright (C) 2007-2009 Gabest
 *
 *  This Program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This Program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with GNU Make; see the file COPYING.  If not, write to
 *  the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA USA.
 *  http://www.gnu.org/copyleft/gpl.html
 *
 */

#include "stdafx.h"
#include "GSRendererOGL.h"
#include "GSRenderer.h"


GSRendererOGL::GSRendererOGL()
	: GSRendererHW(new GSTextureCacheOGL(this))
{
	m_accurate_date   = theApp.GetConfigB("accurate_date");

	m_sw_blending = theApp.GetConfigI("accurate_blending_unit");

	// Hope nothing requires too many draw calls.
	m_drawlist.reserve(2048);

	UserHacks_TCOffset       = theApp.GetConfigI("UserHacks_TCOffset");
	UserHacks_TCO_x          = (UserHacks_TCOffset & 0xFFFF) / -1000.0f;
	UserHacks_TCO_y          = ((UserHacks_TCOffset >> 16) & 0xFFFF) / -1000.0f;
	UserHacks_safe_fbmask    = theApp.GetConfigB("UserHacks_safe_fbmask");
	UserHacks_merge_sprite   = theApp.GetConfigB("UserHacks_merge_pp_sprite");
	UserHacks_HPO            = theApp.GetConfigI("UserHacks_HalfPixelOffset_New");

	m_prim_overlap = PRIM_OVERLAP_UNKNOW;
	ResetStates();

	if (!theApp.GetConfigB("UserHacks")) {
		UserHacks_TCOffset       = 0;
		UserHacks_TCO_x          = 0;
		UserHacks_TCO_y          = 0;
		UserHacks_safe_fbmask    = false;
		UserHacks_merge_sprite   = false;
		UserHacks_HPO            = 0;
	}
}

bool GSRendererOGL::CreateDevice(GSDevice* dev)
{
	if (!GSRenderer::CreateDevice(dev))
		return false;

	return true;
}

void GSRendererOGL::Lines2Sprites()
{
	if (m_vt.m_primclass != GS_SPRITE_CLASS) return;

	// each sprite converted to quad needs twice the space

	while(m_vertex.tail * 2 > m_vertex.maxcount)
	{
		GrowVertexBuffer();
	}

	// assume vertices are tightly packed and sequentially indexed (it should be the case)

	if (m_vertex.next >= 2)
	{
		size_t count = m_vertex.next;

		int i = (int)count * 2 - 4;
		GSVertex* s = &m_vertex.buff[count - 2];
		GSVertex* q = &m_vertex.buff[count * 2 - 4];
		uint32* RESTRICT index = &m_index.buff[count * 3 - 6];

		for(; i >= 0; i -= 4, s -= 2, q -= 4, index -= 6)
		{
			GSVertex v0 = s[0];
			GSVertex v1 = s[1];

			v0.RGBAQ = v1.RGBAQ;
			v0.XYZ.Z = v1.XYZ.Z;
			v0.FOG = v1.FOG;

			q[0] = v0;
			q[3] = v1;

			// swap x, s, u

			uint16 x = v0.XYZ.X;
			v0.XYZ.X = v1.XYZ.X;
			v1.XYZ.X = x;

			float s = v0.ST.S;
			v0.ST.S = v1.ST.S;
			v1.ST.S = s;

			uint16 u = v0.U;
			v0.U = v1.U;
			v1.U = u;

			q[1] = v0;
			q[2] = v1;

			index[0] = i + 0;
			index[1] = i + 1;
			index[2] = i + 2;
			index[3] = i + 1;
			index[4] = i + 2;
			index[5] = i + 3;
		}

		m_vertex.head = m_vertex.tail = m_vertex.next = count * 2;
		m_index.tail = count * 3;
	}
}

void GSRendererOGL::SetupIA()
{
	GL_PUSH("IA");

	GSDeviceOGL* dev = (GSDeviceOGL*)m_dev;


	if (UserHacks_WildHack && !isPackedUV_HackFlag && PRIM->TME && PRIM->FST) {
		for(unsigned int i = 0; i < m_vertex.next; i++)
			m_vertex.buff[i].UV &= 0x3FEF3FEF;
	}

	if (!GLLoader::found_geometry_shader)
		Lines2Sprites();

	dev->IASetVertexBuffer(m_vertex.buff, m_vertex.next);
	dev->IASetIndexBuffer(m_index.buff, m_index.tail);

	GLenum t = 0;

	switch(m_vt.m_primclass)
	{
	case GS_POINT_CLASS:
		t = GL_POINTS;
		break;
	case GS_LINE_CLASS:
		t = GL_LINES;
		break;
	case GS_SPRITE_CLASS:
		if (GLLoader::found_geometry_shader)
			t = GL_LINES;
		else
			t = GL_TRIANGLES;
		break;
	case GS_TRIANGLE_CLASS:
		t = GL_TRIANGLES;
		break;
	default:
		__assume(0);
	}

	dev->IASetPrimitiveTopology(t);
}

void GSRendererOGL::EmulateTextureShuffleAndFbmask()
{
	if (m_texture_shuffle) {
		m_ps_sel.shuffle = 1;
		m_ps_sel.dfmt = 0;

		const GIFRegXYOFFSET& o = m_context->XYOFFSET;
		GSVertex* v = &m_vertex.buff[0];
		size_t count = m_vertex.next;

		// vertex position is 8 to 16 pixels, therefore it is the 16-31 bits of the colors
		int  pos = (v[0].XYZ.X - o.OFX) & 0xFF;
		bool write_ba = (pos > 112 && pos < 136);
		// Read texture is 8 to 16 pixels (same as above)
		float tw = (float)(1u << m_context->TEX0.TW);
		int tex_pos = (PRIM->FST) ? v[0].U : (int)(tw * v[0].ST.S);
		tex_pos &= 0xFF;
		m_ps_sel.read_ba = (tex_pos > 112 && tex_pos < 144);

		// Convert the vertex info to a 32 bits color format equivalent
		if (PRIM->FST) {
			GL_INS("First vertex is  P: %d => %d    T: %d => %d", v[0].XYZ.X, v[1].XYZ.X, v[0].U, v[1].U);

			for(size_t i = 0; i < count; i += 2) {
				if (write_ba)
					v[i].XYZ.X   -= 128u;
				else
					v[i+1].XYZ.X += 128u;

				if (m_ps_sel.read_ba)
					v[i].U       -= 128u;
				else
					v[i+1].U     += 128u;

				// Height is too big (2x).
				int tex_offset = v[i].V & 0xF;
				GSVector4i offset(o.OFY, tex_offset, o.OFY, tex_offset);

				GSVector4i tmp(v[i].XYZ.Y, v[i].V, v[i+1].XYZ.Y, v[i+1].V);
				tmp = GSVector4i(tmp - offset).srl32(1) + offset;

				v[i].XYZ.Y   = (uint16)tmp.x;
				v[i].V       = (uint16)tmp.y;
				v[i+1].XYZ.Y = (uint16)tmp.z;
				v[i+1].V     = (uint16)tmp.w;
			}
		} else {
			const float offset_8pix = 8.0f / tw;
			GL_INS("First vertex is  P: %d => %d    T: %f => %f (offset %f)", v[0].XYZ.X, v[1].XYZ.X, v[0].ST.S, v[1].ST.S, offset_8pix);

			for(size_t i = 0; i < count; i += 2) {
				if (write_ba)
					v[i].XYZ.X   -= 128u;
				else
					v[i+1].XYZ.X += 128u;

				if (m_ps_sel.read_ba)
					v[i].ST.S    -= offset_8pix;
				else
					v[i+1].ST.S  += offset_8pix;

				// Height is too big (2x).
				GSVector4i offset(o.OFY, o.OFY);

				GSVector4i tmp(v[i].XYZ.Y, v[i+1].XYZ.Y);
				tmp = GSVector4i(tmp - offset).srl32(1) + offset;

				//fprintf(stderr, "Before %d, After %d\n", v[i+1].XYZ.Y, tmp.y);
				v[i].XYZ.Y   = (uint16)tmp.x;
				v[i].ST.T   /= 2.0f;
				v[i+1].XYZ.Y = (uint16)tmp.y;
				v[i+1].ST.T /= 2.0f;
			}
		}

		// If date is enabled you need to test the green channel instead of the
		// alpha channel. Only enable this code in DATE mode to reduce the number
		// of shader.
		m_ps_sel.write_rg = !write_ba && m_context->TEST.DATE;

		// Please bang my head against the wall!
		// 1/ Reduce the frame mask to a 16 bit format
		const uint32& m = m_context->FRAME.FBMSK;
		uint32 fbmask = ((m >> 3) & 0x1F) | ((m >> 6) & 0x3E0) | ((m >> 9) & 0x7C00) | ((m >> 16) & 0x8000);
		// FIXME GSVector will be nice here
		uint8 rg_mask = fbmask & 0xFF;
		uint8 ba_mask = (fbmask >> 8) & 0xFF;
		m_om_csel.wrgba = 0;

		// 2 Select the new mask (Please someone put SSE here)
		if (rg_mask != 0xFF) {
			if (write_ba) {
				GL_INS("Color shuffle %s => B", m_ps_sel.read_ba ? "B" : "R");
				m_om_csel.wb = 1;
			} else {
				GL_INS("Color shuffle %s => R", m_ps_sel.read_ba ? "B" : "R");
				m_om_csel.wr = 1;
			}
			if (rg_mask)
				m_ps_sel.fbmask = 1;
		}

		if (ba_mask != 0xFF) {
			if (write_ba) {
				GL_INS("Color shuffle %s => A", m_ps_sel.read_ba ? "A" : "G");
				m_om_csel.wa = 1;
			} else {
				GL_INS("Color shuffle %s => G", m_ps_sel.read_ba ? "A" : "G");
				m_om_csel.wg = 1;
			}
			if (ba_mask)
				m_ps_sel.fbmask = 1;
		}

		if (m_ps_sel.fbmask && m_sw_blending) {
			GL_INS("FBMASK SW emulated fb_mask:%x on tex shuffle", fbmask);
			ps_cb.FbMask.r = rg_mask;
			ps_cb.FbMask.g = rg_mask;
			ps_cb.FbMask.b = ba_mask;
			ps_cb.FbMask.a = ba_mask;
			m_require_full_barrier = true;
		} else {
			m_ps_sel.fbmask = 0;
		}

	} else {
		m_ps_sel.dfmt = GSLocalMemory::m_psm[m_context->FRAME.PSM].fmt;

		GSVector4i fbmask_v = GSVector4i::load((int)m_context->FRAME.FBMSK);
		int ff_fbmask = fbmask_v.eq8(GSVector4i::xffffffff()).mask();
		int zero_fbmask = fbmask_v.eq8(GSVector4i::zero()).mask();

		m_om_csel.wrgba = ~ff_fbmask; // Enable channel if at least 1 bit is 0

		m_ps_sel.fbmask = m_sw_blending && (~ff_fbmask & ~zero_fbmask & 0xF);

		if (m_ps_sel.fbmask) {
			ps_cb.FbMask = fbmask_v.u8to32();
			// Only alpha is special here, I think we can take a very unsafe shortcut
			// Alpha isn't blended on the GS but directly copyied into the RT.
			//
			// Behavior is clearly undefined however there is a high probability that
			// it will work. Masked bit will be constant and normally the same everywhere
			// RT/FS output/Cached value.
			//
			// Just to be sure let's add a new safe hack for unsafe access :)
			//
			// Here the GL spec quote to emphasize the unexpected behavior.
			/*
			   - If a texel has been written, then in order to safely read the result
			   a texel fetch must be in a subsequent Draw separated by the command

			   void TextureBarrier(void);

			   TextureBarrier() will guarantee that writes have completed and caches
			   have been invalidated before subsequent Draws are executed.
			 */
			if (!(~ff_fbmask & ~zero_fbmask & 0x7) && !UserHacks_safe_fbmask) {
				GL_INS("FBMASK Unsafe SW emulated fb_mask:%x on %d bits format", m_context->FRAME.FBMSK,
						(GSLocalMemory::m_psm[m_context->FRAME.PSM].fmt == 2) ? 16 : 32);
				m_require_one_barrier = true;
			} else {
				// The safe and accurate path (but slow)
				GL_INS("FBMASK SW emulated fb_mask:%x on %d bits format", m_context->FRAME.FBMSK,
						(GSLocalMemory::m_psm[m_context->FRAME.PSM].fmt == 2) ? 16 : 32);
				m_require_full_barrier = true;;
			}
		}
	}
}

void GSRendererOGL::EmulateChannelShuffle(GSTexture** rt, const GSTextureCache::Source* tex)
{
	GSDeviceOGL* dev         = (GSDeviceOGL*)m_dev;

	// Uncomment to disable HLE emulation (allow to trace the draw call)
	// m_channel_shuffle = false;

	// First let's check we really have a channel shuffle effect
	if (m_channel_shuffle) {
		if (m_game.title == CRC::GT4 || m_game.title == CRC::GT3 || m_game.title == CRC::GTConcept || m_game.title == CRC::TouristTrophy) {
			GL_INS("Gran Turismo RGB Channel");
			m_ps_sel.channel = 7;
			m_context->TEX0.TFX = TFX_DECAL;
			*rt = tex->m_from_target;
		} else if (m_game.title == CRC::Tekken5) {
			GL_INS("Tekken5 RGB Channel");
			m_ps_sel.channel = 7;
			m_context->FRAME.FBMSK = 0xFF000000;
			// 12 pages: 2 calls by channel, 3 channels, 1 blit
			// Minus current draw call
			m_skip = 12 * (3 + 3 + 1) - 1;
			*rt = tex->m_from_target;
		} else if ((tex->m_texture->GetType() == GSTexture::DepthStencil) && !(tex->m_32_bits_fmt)) {
			// So far 2 games hit this code path. Urban Chaos and Tales of Abyss
			// UC: will copy depth to green channel
			// ToA: will copy depth to alpha channel
			if ((m_context->FRAME.FBMSK & 0xFF0000) == 0xFF0000) {
				// Green channel is masked
				GL_INS("Tales Of Abyss Crazyness (MSB 16b depth to Alpha)");
				m_ps_sel.tales_of_abyss_hle = 1;
			} else {
				GL_INS("Urban Chaos Crazyness (Green extraction)");
				m_ps_sel.urban_chaos_hle = 1;
			}
		} else if (m_index.tail <= 64 && m_context->CLAMP.WMT == 3) {
			// Blood will tell. I think it is channel effect too but again
			// implemented in a different way. I don't want to add more CRC stuff. So
			// let's disable channel when the signature is different
			//
			// Note: Tales Of Abyss and Tekken5 could hit this path too. Those games are
			// handled above.
			GL_INS("maybe not a channel!");
			m_channel_shuffle = false;
		} else if (m_context->CLAMP.WMS == 3 && ((m_context->CLAMP.MAXU & 0x8) == 8)) {
			// Read either blue or Alpha. Let's go for Blue ;)
			// MGS3/Kill Zone
			GL_INS("Blue channel");
			m_ps_sel.channel = 3;
		} else if (m_context->CLAMP.WMS == 3 && ((m_context->CLAMP.MINU & 0x8) == 0)) {
			// Read either Red or Green. Let's check the V coordinate. 0-1 is likely top so
			// red. 2-3 is likely bottom so green (actually depends on texture base pointer offset)
			bool green = PRIM->FST && (m_vertex.buff[0].V & 32);
			if (green && (m_context->FRAME.FBMSK & 0x00FFFFFF) == 0x00FFFFFF) {
				// Typically used in Terminator 3
				int blue_mask  = m_context->FRAME.FBMSK >> 24;
				int green_mask = ~blue_mask & 0xFF;
				int blue_shift = -1;

				// Note: potentially we could also check the value of the clut
				switch (m_context->FRAME.FBMSK >> 24) {
					case 0xFF: ASSERT(0);      break;
					case 0xFE: blue_shift = 1; break;
					case 0xFC: blue_shift = 2; break;
					case 0xF8: blue_shift = 3; break;
					case 0xF0: blue_shift = 4; break;
					case 0xE0: blue_shift = 5; break;
					case 0xC0: blue_shift = 6; break;
					case 0x80: blue_shift = 7; break;
					default:   ASSERT(0);      break;
				}

				int green_shift = 8 - blue_shift;
				dev->SetupCBMisc(GSVector4i(blue_mask, blue_shift, green_mask, green_shift));

				if (blue_shift >= 0) {
					GL_INS("Green/Blue channel (%d, %d)", blue_shift, green_shift);
					m_ps_sel.channel = 6;
					m_context->FRAME.FBMSK = 0x00FFFFFF;
				} else {
					GL_INS("Green channel (wrong mask) (fbmask %x)", m_context->FRAME.FBMSK >> 24);
					m_ps_sel.channel = 2;
				}

			} else if (green) {
				GL_INS("Green channel");
				m_ps_sel.channel = 2;
			} else {
				// Pop
				GL_INS("Red channel");
				m_ps_sel.channel = 1;
			}
		} else {
			GL_INS("channel not supported");
			m_channel_shuffle = false;
			ASSERT(0);
		}
	}

	// Effect is really a channel shuffle effect so let's cheat a little
	if (m_channel_shuffle) {
		dev->PSSetShaderResource(4, tex->m_from_target);
		m_require_one_barrier = true;

		// Replace current draw with a fullscreen sprite
		//
		// Performance GPU note: it could be wise to reduce the size to
		// the rendered size of the framebuffer

		GSVertex* s = &m_vertex.buff[0];
		s[0].XYZ.X = (uint16)(m_context->XYOFFSET.OFX + 0);
		s[1].XYZ.X = (uint16)(m_context->XYOFFSET.OFX + 16384);
		s[0].XYZ.Y = (uint16)(m_context->XYOFFSET.OFY + 0);
		s[1].XYZ.Y = (uint16)(m_context->XYOFFSET.OFY + 16384);

		m_vertex.head = m_vertex.tail = m_vertex.next = 2;
		m_index.tail = 2;

	} else {
#ifdef ENABLE_OGL_DEBUG
		dev->PSSetShaderResource(4, NULL);
#endif
	}
}

void GSRendererOGL::EmulateBlending(bool DATE_GL42)
{
	GSDeviceOGL* dev         = (GSDeviceOGL*)m_dev;
	const GIFRegALPHA& ALPHA = m_context->ALPHA;
	bool sw_blending         = false;

	// No blending so early exit
	if (!(PRIM->ABE || PRIM->AA1 && m_vt.m_primclass == GS_LINE_CLASS)) {
#ifdef ENABLE_OGL_DEBUG
		if (m_env.PABE.PABE) {
			GL_INS("!!! ENV PABE  without ABE !!!");
		}
#endif
		dev->OMSetBlendState();
		return;
	}

	if (m_env.PABE.PABE)
	{
		GL_INS("!!! ENV PABE  not supported !!!");
		if (m_sw_blending >= ACC_BLEND_CCLIP_DALPHA) {
			m_ps_sel.pabe = 1;
			m_require_full_barrier |= (ALPHA.C == 1);
			sw_blending = true;
		}
		//Breath of Fire Dragon Quarter triggers this in battles. Graphics are fine though.
		//ASSERT(0);
	}

	// Compute the blending equation to detect special case
	uint8 blend_index  = uint8(((ALPHA.A * 3 + ALPHA.B) * 3 + ALPHA.C) * 3 + ALPHA.D);
	int blend_flag = GSDeviceOGL::m_blendMapOGL[blend_index].bogus;

	// SW Blend is (nearly) free. Let's use it.
	bool impossible_or_free_blend = (blend_flag & (BLEND_NO_BAR|BLEND_A_MAX|BLEND_ACCU)) // Blend doesn't requires the costly barrier
		|| (m_prim_overlap == PRIM_OVERLAP_NO)	// Blend can be done in a single draw
		|| (m_require_full_barrier);			// Another effect (for example fbmask) already requires a full barrier

	// Do the multiplication in shader for blending accumulation: Cs*As + Cd or Cs*Af + Cd
	bool accumulation_blend = !!(blend_flag & BLEND_ACCU);

	// Warning no break on purpose
	switch (m_sw_blending) {
		case ACC_BLEND_ULTRA:           sw_blending |= true;
		case ACC_BLEND_FULL:            if (!m_vt.m_alpha.valid && (ALPHA.C == 0)) GetAlphaMinMax();
										sw_blending |= (ALPHA.A != ALPHA.B) &&
												((ALPHA.C == 0 && m_vt.m_alpha.max > 128) || (ALPHA.C == 2 && ALPHA.FIX > 128u));
		case ACC_BLEND_CCLIP_DALPHA:    sw_blending |= (ALPHA.C == 1) || (m_env.COLCLAMP.CLAMP == 0);
										// Initial idea was to enable accurate blending for sprite rendering to handle
										// correctly post-processing effect. Some games (ZoE) use tons of sprites as particles.
										// In order to keep it fast, let's limit it to smaller draw call.
		case ACC_BLEND_SPRITE:          sw_blending |= m_vt.m_primclass == GS_SPRITE_CLASS && m_drawlist.size() < 100;
		case ACC_BLEND_FREE:            sw_blending |= impossible_or_free_blend;
		default:                        /*sw_blending |= accumulation_blend*/;
	}
	// SW Blending
	// GL42 interact very badly with sw blending. GL42 uses the primitiveID to find the primitive
	// that write the bad alpha value. Sw blending will force the draw to run primitive by primitive
	// (therefore primitiveID will be constant to 1)
	sw_blending &= !DATE_GL42;

	// Color clip
	if (m_env.COLCLAMP.CLAMP == 0) {
		if (m_prim_overlap == PRIM_OVERLAP_NO) {
			// The fastest algo that requires a single pass
			GL_INS("COLCLIP Free mode ENABLED");
			m_ps_sel.colclip = 1;
			//ASSERT(sw_blending);
			sw_blending = true;
			accumulation_blend = false; // disable the HDR algo
		} else if (accumulation_blend) {
			// A fast algo that requires 2 passes
			GL_INS("COLCLIP Fast HDR mode ENABLED");
			m_ps_sel.hdr = 1;
			sw_blending  = true; // Enable sw blending for the HDR algo
		} else if (sw_blending) {
			// A slow algo that could requires several passes (barely used)
			GL_INS("COLCLIP SW ENABLED (blending is %d/%d/%d/%d)", ALPHA.A, ALPHA.B, ALPHA.C, ALPHA.D);
			m_ps_sel.colclip = 1;
		} else {
			// Speed hack skip previous slow algo
			GL_INS("Sorry colclip isn't supported");
		}
	}

	// Seriously don't expect me to support this kind of crazyness.
	// No mix of COLCLIP + accumulation_blend + DATE GL42
	// Neither fbmask and GL42
	ASSERT(!(m_ps_sel.hdr && DATE_GL42));
	ASSERT(!(m_ps_sel.fbmask && DATE_GL42));

	// For stat to optimize accurate option
#if 0
	GL_INS("BLEND_INFO: %d/%d/%d/%d. Clamp:%d. Prim:%d number %d (drawlist %d) (sw %d)",
			ALPHA.A, ALPHA.B,  ALPHA.C, ALPHA.D, m_env.COLCLAMP.CLAMP, m_vt.m_primclass, m_vertex.next, m_drawlist.size(), sw_blending);
#endif
	if (sw_blending) {
		m_ps_sel.blend_a = ALPHA.A;
		m_ps_sel.blend_b = ALPHA.B;
		m_ps_sel.blend_c = ALPHA.C;
		m_ps_sel.blend_d = ALPHA.D;

		if (accumulation_blend) {
			// Keep HW blending to do the addition/subtraction
			dev->OMSetBlendState(blend_index, 0, false, true);
			if (ALPHA.A == 2) {
				// The blend unit does a reverse subtraction so it means
				// the shader must output a positive value.
				// Replace 0 - Cs by Cs - 0
				m_ps_sel.blend_a = ALPHA.B;
				m_ps_sel.blend_b = 2;
			}
			// Remove the addition/substraction from the SW blending
			m_ps_sel.blend_d = 2;

			// Note accumulation_blend doesn't require a barrier

		} else {
			// Disable HW blending
			dev->OMSetBlendState();

			m_require_full_barrier |= !(blend_flag & BLEND_NO_BAR);
		}

		// Require the fix alpha vlaue
		if (ALPHA.C == 2) {
			ps_cb.TA_Af.a = (float)ALPHA.FIX / 128.0f;
		}
	} else {
		m_ps_sel.clr1 = !!(blend_flag & BLEND_C_CLR);
		if (m_ps_sel.dfmt == 1 && ALPHA.C == 1) {
			// 24 bits doesn't have an alpha channel so use 1.0f fix factor as equivalent
			uint8 hacked_blend_index  = blend_index + 3; // +3 <=> +1 on C
			dev->OMSetBlendState(hacked_blend_index, 128, true);
		} else {
			dev->OMSetBlendState(blend_index, ALPHA.FIX, (ALPHA.C == 2));
		}
	}
}

void GSRendererOGL::RealignTargetTextureCoordinate(const GSTextureCache::Source* tex)
{
	if (!UserHacks_HPO) return;

	GSVertex* v             = &m_vertex.buff[0];
	const GSVector2& scale  = tex->m_texture->GetScale();
	bool  linear            = m_vt.IsLinear();
	int t_position          = v[0].U;
	GSVector4 half_offset(0.0f);

	// FIXME Let's start with something wrong same mess on X and Y
	// FIXME Maybe it will be enough to check linear

	if (PRIM->FST) {

		if (UserHacks_HPO > 1) {
			if (!linear && t_position == 8) {
				half_offset.x = 8;
				half_offset.y = 8;
			} else if (linear && t_position == 16) {
				half_offset.x = 16;
				half_offset.y = 16;
			}
		} else {
			if (!linear && t_position == 8) {
				half_offset.x = 8 - 8 / scale.x;
				half_offset.y = 8 - 8 / scale.y;
			} else if (linear && t_position == 16) {
				half_offset.x = 16 - 16 / scale.x;
				half_offset.y = 16 - 16 / scale.y;
			}
		}

		GL_INS("offset detected %f,%f t_pos %d (linear %d, scale %f)",
				half_offset.x, half_offset.y, t_position, linear, scale.x);

	} else if (m_vt.m_eq.q) {
		float tw = (float)(1 << m_context->TEX0.TW);
		float th = (float)(1 << m_context->TEX0.TH);
		float q  = v[0].RGBAQ.Q;

		// Tales of Abyss
		half_offset.x = 0.5f * q / tw;
		half_offset.y = 0.5f * q / th;

		GL_INS("ST offset detected %f,%f (linear %d, scale %f)",
				half_offset.x, half_offset.y, linear, scale.x);

	}

	vs_cb.TextureOffset = GSVector4(half_offset);
}

void GSRendererOGL::EmulateTextureSampler(const GSTextureCache::Source* tex)
{
	GSDeviceOGL* dev         = (GSDeviceOGL*)m_dev;

	// Warning fetch the texture PSM format rather than the context format. The latter could have been corrected in the texture cache for depth.
	//const GSLocalMemory::psm_t &psm = GSLocalMemory::m_psm[m_context->TEX0.PSM];
	const GSLocalMemory::psm_t &psm = GSLocalMemory::m_psm[tex->m_TEX0.PSM];
	const GSLocalMemory::psm_t &cpsm = psm.pal > 0 ? GSLocalMemory::m_psm[m_context->TEX0.CPSM] : psm;

	const uint8 wms = m_context->CLAMP.WMS;
	const uint8 wmt = m_context->CLAMP.WMT;
	bool complex_wms_wmt = !!((wms | wmt) & 2);

	bool bilinear = m_filter == 2 ? m_vt.IsLinear() : m_filter != 0;
	bool simple_sample = !tex->m_palette && cpsm.fmt == 0 && !complex_wms_wmt && !psm.depth;
	// Don't force extra filtering on sprite (it creates various upscaling issue)
	bilinear &= !((m_vt.m_primclass == GS_SPRITE_CLASS) && m_userhacks_round_sprite_offset && !m_vt.IsLinear());

	// 1 and 0 are equivalent
	m_ps_sel.wms = (wms & 2) ? wms : 0;
	m_ps_sel.wmt = (wmt & 2) ? wmt : 0;

	// Depth + bilinear filtering isn't done yet (And I'm not sure we need it anyway but a game will prove me wrong)
	// So of course, GTA set the linear mode, but sampling is done at texel center so it is equivalent to nearest sampling
	ASSERT(!(psm.depth && m_vt.IsLinear()));

	// Performance note:
	// 1/ Don't set 0 as it is the default value
	// 2/ Only keep aem when it is useful (avoid useless shader permutation)
	if (m_ps_sel.shuffle) {
		// Force a 32 bits access (normally shuffle is done on 16 bits)
		// m_ps_sel.tex_fmt = 0; // removed as an optimization
		m_ps_sel.aem     = m_env.TEXA.AEM;
		ASSERT(tex->m_target);

		// Require a float conversion if the texure is a depth otherwise uses Integral scaling
		if (psm.depth) {
			m_ps_sel.depth_fmt = (tex->m_texture->GetType() != GSTexture::DepthStencil) ? 3 : 1;
		}

		// Shuffle is a 16 bits format, so aem is always required
		GSVector4 ta(m_env.TEXA & GSVector4i::x000000ff());
		ta /= 255.0f;
		// FIXME rely on compiler for the optimization
		ps_cb.TA_Af.x = ta.x;
		ps_cb.TA_Af.y = ta.y;

		// The purpose of texture shuffle is to move color channel. Extra interpolation is likely a bad idea.
		bilinear &= m_vt.IsLinear();

		RealignTargetTextureCoordinate(tex);

	} else if (tex->m_target) {
		// Use an old target. AEM and index aren't resolved it must be done
		// on the GPU

		// Select the 32/24/16 bits color (AEM)
		m_ps_sel.tex_fmt = cpsm.fmt;
		m_ps_sel.aem     = m_env.TEXA.AEM;

		// Don't upload AEM if format is 32 bits
		if (cpsm.fmt) {
			GSVector4 ta(m_env.TEXA & GSVector4i::x000000ff());
			ta /= 255.0f;
			// FIXME rely on compiler for the optimization
			ps_cb.TA_Af.x = ta.x;
			ps_cb.TA_Af.y = ta.y;
		}

		// Select the index format
		if (tex->m_palette) {
			// FIXME Potentially improve fmt field in GSLocalMemory
			if (m_context->TEX0.PSM == PSM_PSMT4HL)
				m_ps_sel.tex_fmt |= 1 << 2;
			else if (m_context->TEX0.PSM == PSM_PSMT4HH)
				m_ps_sel.tex_fmt |= 2 << 2;
			else
				m_ps_sel.tex_fmt |= 3 << 2;

			// Alpha channel of the RT is reinterpreted as an index. Star
			// Ocean 3 uses it to emulate a stencil buffer.  It is a very
			// bad idea to force bilinear filtering on it.
			bilinear &= m_vt.IsLinear();
		}

		// Depth format
		if (tex->m_texture->GetType() == GSTexture::DepthStencil) {
			// Require a float conversion if the texure is a depth format
			m_ps_sel.depth_fmt = (psm.bpp == 16) ? 2 : 1;

			// Don't force interpolation on depth format
			bilinear &= m_vt.IsLinear();
		} else if (psm.depth) {
			// Use Integral scaling
			m_ps_sel.depth_fmt = (tex->m_texture->GetType() != GSTexture::DepthStencil) ? 3 :

				// Don't force interpolation on depth format
				bilinear &= m_vt.IsLinear();
		}

		RealignTargetTextureCoordinate(tex);

	} else if (tex->m_palette) {
		// Use a standard 8 bits texture. AEM is already done on the CLUT
		// Therefore you only need to set the index
		// m_ps_sel.aem     = 0; // removed as an optimization

		// Note 4 bits indexes are converted to 8 bits
		m_ps_sel.tex_fmt = 3 << 2;

	} else {
		// Standard texture. Both index and AEM expansion were already done by the CPU.
		// m_ps_sel.tex_fmt = 0; // removed as an optimization
		// m_ps_sel.aem     = 0; // removed as an optimization
	}

	if (m_context->TEX0.TFX == TFX_MODULATE && m_vt.m_eq.rgba == 0xFFFF && m_vt.m_min.c.eq(GSVector4i(128))) {
		// Micro optimization that reduces GPU load (removes 5 instructions on the FS program)
		m_ps_sel.tfx = TFX_DECAL;
	} else {
		m_ps_sel.tfx = m_context->TEX0.TFX;
	}

	m_ps_sel.tcc = m_context->TEX0.TCC;

	m_ps_sel.ltf = bilinear && !simple_sample;

	int w = tex->m_texture->GetWidth();
	int h = tex->m_texture->GetHeight();

	int tw = (int)(1 << m_context->TEX0.TW);
	int th = (int)(1 << m_context->TEX0.TH);

	GSVector4 WH(tw, th, w, h);

	m_ps_sel.fst = !!PRIM->FST;

	ps_cb.WH = WH;
	ps_cb.HalfTexel = GSVector4(-0.5f, 0.5f).xxyy() / WH.zwzw();
	if (complex_wms_wmt) {
		ps_cb.MskFix = GSVector4i(m_context->CLAMP.MINU, m_context->CLAMP.MINV, m_context->CLAMP.MAXU, m_context->CLAMP.MAXV);
		ps_cb.MinMax = GSVector4(ps_cb.MskFix) / WH.xyxy();
	}

	// TC Offset Hack
	m_ps_sel.tcoffsethack = !!UserHacks_TCOffset;
	ps_cb.TC_OH_TS = GSVector4(1/16.0f, 1/16.0f, UserHacks_TCO_x, UserHacks_TCO_y) / WH.xyxy();


	// Only enable clamping in CLAMP mode. REGION_CLAMP will be done manually in the shader
	m_ps_ssel.tau   = (wms != CLAMP_CLAMP);
	m_ps_ssel.tav   = (wmt != CLAMP_CLAMP);
	m_ps_ssel.ltf   = bilinear && simple_sample;
	m_ps_ssel.aniso = simple_sample;

	// Setup Texture ressources
	dev->SetupSampler(m_ps_ssel);
	dev->PSSetShaderResources(tex->m_texture, tex->m_palette);

	if (tex->m_spritehack_t && (m_ps_sel.atst == 2)) {
		m_ps_sel.atst = 1;
	}
}

GSRendererOGL::PRIM_OVERLAP GSRendererOGL::PrimitiveOverlap()
{
	// Either 1 triangle or 1 line or 3 POINTs
	// It is bad for the POINTs but low probability that they overlap
	if (m_vertex.next < 4)
		return PRIM_OVERLAP_NO;

	if (m_vt.m_primclass != GS_SPRITE_CLASS)
		return PRIM_OVERLAP_UNKNOW; // maybe, maybe not

	// Check intersection of sprite primitive only
	size_t count = m_vertex.next;
	PRIM_OVERLAP overlap = PRIM_OVERLAP_NO;
	GSVertex* v = m_vertex.buff;

	m_drawlist.clear();
	size_t i = 0;
	while (i < count) {
		// In order to speed up comparison a bounding-box is accumulated. It removes a
		// loop so code is much faster (check game virtua fighter). Besides it allow to check
		// properly the Y order.

		// .x = min(v[i].XYZ.X, v[i+1].XYZ.X)
		// .y = min(v[i].XYZ.Y, v[i+1].XYZ.Y)
		// .z = max(v[i].XYZ.X, v[i+1].XYZ.X)
		// .w = max(v[i].XYZ.Y, v[i+1].XYZ.Y)
		GSVector4i all = GSVector4i(v[i].m[1]).upl16(GSVector4i(v[i+1].m[1])).upl16().xzyw();
		all = all.xyxy().blend(all.zwzw(), all > all.zwxy());

		size_t j = i + 2;
		while (j < count) {
			GSVector4i sprite = GSVector4i(v[j].m[1]).upl16(GSVector4i(v[j+1].m[1])).upl16().xzyw();
			sprite = sprite.xyxy().blend(sprite.zwzw(), sprite > sprite.zwxy());

			// Be sure to get vertex in good order, otherwise .r* function doesn't
			// work as expected.
			ASSERT(sprite.x <= sprite.z);
			ASSERT(sprite.y <= sprite.w);
			ASSERT(all.x <= all.z);
			ASSERT(all.y <= all.w);

			if (all.rintersect(sprite).rempty()) {
				all = all.runion_ordered(sprite);
			} else {
				overlap = PRIM_OVERLAP_YES;
				break;
			}
			j += 2;
		}
		m_drawlist.push_back((j - i) >> 1); // Sprite count
		i = j;
	}

#if 0
	// Old algo: less constraint but O(n^2) instead of O(n) as above

	// You have no guarantee on the sprite order, first vertex can be either top-left or bottom-left
	// There is a high probability that the draw call will uses same ordering for all vertices.
	// In order to keep a small performance impact only the first sprite will be checked
	//
	// Some safe-guard will be added in the outer-loop to avoid corruption with a limited perf impact
	if (v[1].XYZ.Y < v[0].XYZ.Y) {
		// First vertex is Top-Left
		for(size_t i = 0; i < count; i += 2) {
			if (v[i+1].XYZ.Y > v[i].XYZ.Y) {
				return PRIM_OVERLAP_UNKNOW;
			}
			GSVector4i vi(v[i].XYZ.X, v[i+1].XYZ.Y, v[i+1].XYZ.X, v[i].XYZ.Y);
			for (size_t j = i+2; j < count; j += 2) {
				GSVector4i vj(v[j].XYZ.X, v[j+1].XYZ.Y, v[j+1].XYZ.X, v[j].XYZ.Y);
				GSVector4i inter = vi.rintersect(vj);
				if (!inter.rempty()) {
					return PRIM_OVERLAP_YES;
				}
			}
		}
	} else {
		// First vertex is Bottom-Left
		for(size_t i = 0; i < count; i += 2) {
			if (v[i+1].XYZ.Y < v[i].XYZ.Y) {
				return PRIM_OVERLAP_UNKNOW;
			}
			GSVector4i vi(v[i].XYZ.X, v[i].XYZ.Y, v[i+1].XYZ.X, v[i+1].XYZ.Y);
			for (size_t j = i+2; j < count; j += 2) {
				GSVector4i vj(v[j].XYZ.X, v[j].XYZ.Y, v[j+1].XYZ.X, v[j+1].XYZ.Y);
				GSVector4i inter = vi.rintersect(vj);
				if (!inter.rempty()) {
					return PRIM_OVERLAP_YES;
				}
			}
		}
	}
#endif

	//fprintf(stderr, "%d: Yes, code can be optimized (draw of %d vertices)\n", s_n, count);
	return overlap;
}

GSVector4i GSRendererOGL::ComputeBoundingBox(const GSVector2& rtscale, const GSVector2i& rtsize)
{
	GSVector4 scale = GSVector4(rtscale.x, rtscale.y);
	GSVector4 offset = GSVector4(-1.0f, 1.0f); // Round value
	GSVector4 box = m_vt.m_min.p.xyxy(m_vt.m_max.p) + offset.xxyy();
	return GSVector4i(box * scale.xyxy()).rintersect(GSVector4i(0, 0, rtsize.x, rtsize.y));
}

void GSRendererOGL::SendDraw()
{
	GSDeviceOGL* dev = (GSDeviceOGL*)m_dev;

	if (!m_require_full_barrier && m_require_one_barrier) {
		// Need only a single barrier
		glTextureBarrier();
		dev->DrawIndexedPrimitive();
	} else if (!m_require_full_barrier) {
		// Don't need any barrier
		dev->DrawIndexedPrimitive();
	} else if (m_prim_overlap == PRIM_OVERLAP_NO) {
		// Need full barrier but a single barrier will be enough
		glTextureBarrier();
		dev->DrawIndexedPrimitive();
	} else if (m_vt.m_primclass == GS_SPRITE_CLASS) {
		size_t nb_vertex = (GLLoader::found_geometry_shader) ? 2 : 6;

		GL_PUSH("Split the draw (SPRITE)");

#if defined(_DEBUG)
		// Check how draw call is split.
		map<size_t, size_t> frequency;
		for (const auto& it: m_drawlist)
			++frequency[it];

		string message;
		for (const auto& it: frequency)
			message += " " + to_string(it.first) + "(" + to_string(it.second) + ")";

		GL_PERF("Split single draw (%d sprites) into %zu draws: consecutive draws(frequency):%s",
			m_index.tail / nb_vertex, m_drawlist.size(), message.c_str());
#endif

		for (size_t count, p = 0, n = 0; n < m_drawlist.size(); p += count, ++n) {
			count = m_drawlist[n] * nb_vertex;
			glTextureBarrier();
			dev->DrawIndexedPrimitive(p, count);
		}
	} else {
		// FIXME: Investigate: a dynamic check to pack as many primitives as possibles
		// I'm nearly sure GSdx already have this kind of code (maybe we can adapt GSDirtyRect)
		size_t nb_vertex = GSUtil::GetClassVertexCount(m_vt.m_primclass);

		GL_PUSH("Split the draw");

		GL_PERF("Split single draw in %d draw", m_index.tail/nb_vertex);

		for (size_t p = 0; p < m_index.tail; p += nb_vertex) {
			glTextureBarrier();
			dev->DrawIndexedPrimitive(p, nb_vertex);
		}
	}
}

void GSRendererOGL::ResetStates()
{
	m_require_one_barrier  = false;
	m_require_full_barrier = false;

	m_vs_sel.key = 0;
	m_gs_sel.key = 0;
	m_ps_sel.key = 0;

	m_ps_ssel.key  = 0;
	m_om_csel.key  = 0;
	m_om_dssel.key = 0;
}

void GSRendererOGL::DrawPrims(GSTexture* rt, GSTexture* ds, GSTextureCache::Source* tex)
{
#ifdef ENABLE_OGL_DEBUG
	GSVector4i area_out = GSVector4i(m_vt.m_min.p.xyxy(m_vt.m_max.p)).rintersect(GSVector4i(m_context->scissor.in));
	GSVector4i area_in  = GSVector4i(m_vt.m_min.t.xyxy(m_vt.m_max.t));

	GL_PUSH("GL Draw from %d (area %d,%d => %d,%d) in %d (Depth %d) (area %d,%d => %d,%d)",
			tex && tex->m_texture ? tex->m_texture->GetID() : -1,
			area_in.x, area_in.y, area_in.z, area_in.w,
			rt ? rt->GetID() : -1, ds ? ds->GetID() : -1,
			area_out.x, area_out.y, area_out.z, area_out.w
		   );
#endif

	GSTexture* hdr_rt = NULL;

	const GSVector2i& rtsize = ds ? ds->GetSize()  : rt->GetSize();
	const GSVector2& rtscale = ds ? ds->GetScale() : rt->GetScale();

	bool DATE = m_context->TEST.DATE && m_context->FRAME.PSM != PSM_PSMCT24;
	bool DATE_GL42 = false;
	bool DATE_GL45 = false;
	bool DATE_one  = false;

	ResetStates();
	vs_cb.TextureOffset = GSVector4(0.0f);

	ASSERT(m_dev != NULL);
	GSDeviceOGL* dev = (GSDeviceOGL*)m_dev;
	dev->s_n = s_n;

	// HLE implementation of the channel selection effect
	//
	// Warning it must be done at the begining because it will change the
	// vertex list (it will interact with PrimitiveOverlap and accurate
	// blending)
	EmulateChannelShuffle(&rt, tex);

	// Upscaling hack to avoid various line/grid issues
	if (UserHacks_merge_sprite && tex && tex->m_target && (m_vt.m_primclass == GS_SPRITE_CLASS)) {
		if (PRIM->FST && GSLocalMemory::m_psm[tex->m_TEX0.PSM].fmt < 2 && ((m_vt.m_eq.value & 0xCFFFF) == 0xCFFFF)) {

			// Ideally the hack ought to be enabled in a true paving mode only. I don't know how to do it accurately
			// neither in a fast way. So instead let's just take the hypothesis that all sprites must have the same
			// size.
			// Tested on Tekken 5.
			GSVertex* v = &m_vertex.buff[0];
			bool is_paving = true;
			// SSE optimization: shuffle m[1] to have (4*32 bits) X, Y, U, V
			int first_dpX = v[1].XYZ.X - v[0].XYZ.X;
			int first_dpU = v[1].U - v[0].U;
			for (size_t i = 0; i < m_vertex.next; i += 2) {
				int dpX = v[i+1].XYZ.X - v[i].XYZ.X;
				int dpU = v[i+1].U - v[i].U;
				if (dpX != first_dpX || dpU != first_dpU) {
					is_paving = false;
					break;
				}
			}

#if 0
			GSVector4 delta_p = m_vt.m_max.p - m_vt.m_min.p;
			GSVector4 delta_t = m_vt.m_max.t - m_vt.m_min.t;
			bool is_blit = PrimitiveOverlap() == PRIM_OVERLAP_NO;
			GL_INS("PP SAMPLER: Dp %f %f Dt %f %f. Is blit %d, is paving %d, count %d", delta_p.x, delta_p.y, delta_t.x, delta_t.y, is_blit, is_paving, m_vertex.tail);
#endif

			if (is_paving) {
				// Replace all sprite with a single fullscreen sprite.
				GSVertex* s = &m_vertex.buff[0];

				s[0].XYZ.X = static_cast<uint16>((16.0f * m_vt.m_min.p.x) + m_context->XYOFFSET.OFX);
				s[1].XYZ.X = static_cast<uint16>((16.0f * m_vt.m_max.p.x) + m_context->XYOFFSET.OFX);
				s[0].XYZ.Y = static_cast<uint16>((16.0f * m_vt.m_min.p.y) + m_context->XYOFFSET.OFY);
				s[1].XYZ.Y = static_cast<uint16>((16.0f * m_vt.m_max.p.y) + m_context->XYOFFSET.OFY);

				s[0].U     = static_cast<uint16>(16.0f * m_vt.m_min.t.x);
				s[0].V     = static_cast<uint16>(16.0f * m_vt.m_min.t.y);
				s[1].U     = static_cast<uint16>(16.0f * m_vt.m_max.t.x);
				s[1].V     = static_cast<uint16>(16.0f * m_vt.m_max.t.y);

				m_vertex.head = m_vertex.tail = m_vertex.next = 2;
				m_index.tail = 2;
			}
		}
	}

	// Always check if primitive overlap. The function will return PRIM_OVERLAP_UNKNOW for non sprite primitive
	m_prim_overlap = PrimitiveOverlap();
#ifdef ENABLE_OGL_DEBUG
	if (m_sw_blending && (m_prim_overlap != PRIM_OVERLAP_NO) && (m_context->FRAME.Block() == m_context->TEX0.TBP0) && (m_vertex.next > 2)) {
		GL_INS("ERROR: Source and Target are the same!");
	}
#endif

	EmulateTextureShuffleAndFbmask();

	// DATE: selection of the algorithm. Must be done before blending because GL42 is not compatible with blending

	if (DATE) {
		if (m_prim_overlap == PRIM_OVERLAP_NO || m_texture_shuffle) {
			// It is way too complex to emulate texture shuffle with DATE. So just use
			// the slow but accurate algo
			m_require_full_barrier = true;
			DATE_GL45 = true;
			DATE = false;
		} else if (m_om_csel.wa && (!m_context->TEST.ATE || m_context->TEST.ATST == ATST_ALWAYS)) {
			// Performance note: check alpha range with GetAlphaMinMax()
			// Note: all my dump are already above 120fps, but it seems to reduce GPU load
			// with big upscaling
			GetAlphaMinMax();
			if (m_context->TEST.DATM && m_vt.m_alpha.max < 128) {
				// Only first pixel (write 0) will pass (alpha is 1)
				GL_PERF("Fast DATE with alpha %d-%d", m_vt.m_alpha.min, m_vt.m_alpha.max);
				DATE_one = true;
			} else if (!m_context->TEST.DATM && m_vt.m_alpha.min >= 128) {
				// Only first pixel (write 1) will pass (alpha is 0)
				GL_PERF("Fast DATE with alpha %d-%d", m_vt.m_alpha.min, m_vt.m_alpha.max);
				DATE_one = true;
			} else if ((m_vt.m_primclass == GS_SPRITE_CLASS && m_drawlist.size() < 50) || (m_index.tail < 100)) {
				// texture barrier will split the draw call into n draw call. It is very efficient for
				// few primitive draws. Otherwise it sucks.
				GL_PERF("Slower DATE with alpha %d-%d", m_vt.m_alpha.min, m_vt.m_alpha.max);
				m_require_full_barrier = true;
				DATE_GL45 = true;
				DATE = false;
			} else if (m_accurate_date) {
				GL_PERF("Slow DATE with alpha %d-%d", m_vt.m_alpha.min, m_vt.m_alpha.max);

				if (GLLoader::found_GL_ARB_shader_image_load_store) {
					DATE_GL42 = true;
				} else {
					m_require_full_barrier = true;
					DATE_GL45 = true;
					DATE = false;
				}
			}
		} else if (!m_om_csel.wa && (!m_context->TEST.ATE || m_context->TEST.ATST == ATST_ALWAYS)) {
			// TODO: is it legal ? Likely but it need to be tested carefully
			// DATE_GL45 = true;
			// m_require_one_barrier = true; << replace it with a cheap barrier

		}

		// Will save my life !
		ASSERT(!(DATE_GL45 && DATE_one));
		ASSERT(!(DATE_GL42 && DATE_one));
		ASSERT(!(DATE_GL42 && DATE_GL45));
	}

	// Blend

	if (!IsOpaque() && rt) {
		EmulateBlending(DATE_GL42);
	} else {
		dev->OMSetBlendState(); // No blending please
	}

	if (m_ps_sel.dfmt == 1) {
		// Disable writing of the alpha channel
		m_om_csel.wa = 0;
	}

	// DATE (setup part)

	if (DATE) {
		GSVector4i dRect = ComputeBoundingBox(rtscale, rtsize);

		// Reduce the quantity of clean function
		glScissor( dRect.x, dRect.y, dRect.width(), dRect.height() );
		GLState::scissor = dRect;

		// Must be done here to avoid any GL state pertubation (clear function...)
		// Create an r32ui image that will containt primitive ID
		if (DATE_GL42) {
			dev->InitPrimDateTexture(rt, dRect);
		} else if (DATE_one) {
			dev->ClearStencil(ds, 1);
		} else {
			GSVector4 src = GSVector4(dRect) / GSVector4(rtsize.x, rtsize.y).xyxy();
			GSVector4 dst = src * 2.0f - 1.0f;

			GSVertexPT1 vertices[] =
			{
				{GSVector4(dst.x, dst.y, 0.0f, 0.0f), GSVector2(src.x, src.y)},
				{GSVector4(dst.z, dst.y, 0.0f, 0.0f), GSVector2(src.z, src.y)},
				{GSVector4(dst.x, dst.w, 0.0f, 0.0f), GSVector2(src.x, src.w)},
				{GSVector4(dst.z, dst.w, 0.0f, 0.0f), GSVector2(src.z, src.w)},
			};

			dev->SetupDATE(rt, ds, vertices, m_context->TEST.DATM);
		}
	}

	//

	dev->BeginScene();

	// om

	if (m_context->TEST.ZTE)
	{
		m_om_dssel.ztst = m_context->TEST.ZTST;
		m_om_dssel.zwe = !m_context->ZBUF.ZMSK;
	}
	else
	{
		m_om_dssel.ztst = ZTST_ALWAYS;
	}

	// vs

	// The real GS appears to do no masking based on the Z buffer format and writing larger Z values
	// than the buffer supports seems to be an error condition on the real GS, causing it to crash.
	// We are probably receiving bad coordinates from VU1 in these cases.
	vs_cb.DepthMask = GSVector2i(0xFFFFFFFF, 0xFFFFFFFF);

	if (m_om_dssel.ztst >= ZTST_ALWAYS && m_om_dssel.zwe)
	{
		if (m_context->ZBUF.PSM == PSM_PSMZ24)
		{
			if (m_vt.m_max.p.z > 0xffffff)
			{
				ASSERT(m_vt.m_min.p.z > 0xffffff);
				// Fixme :Following conditional fixes some dialog frame in Wild Arms 3, but may not be what was intended.
				if (m_vt.m_min.p.z > 0xffffff)
				{
					GL_INS("Bad Z size on 24 bits buffers")
					vs_cb.DepthMask = GSVector2i(0x00FFFFFF, 0x00FFFFFF);
					m_om_dssel.ztst = ZTST_ALWAYS;
				}
			}
		}
		else if (m_context->ZBUF.PSM == PSM_PSMZ16 || m_context->ZBUF.PSM == PSM_PSMZ16S)
		{
			if (m_vt.m_max.p.z > 0xffff)
			{
				ASSERT(m_vt.m_min.p.z > 0xffff); // sfex capcom logo
				// Fixme : Same as above, I guess.
				if (m_vt.m_min.p.z > 0xffff)
				{
					GL_INS("Bad Z size on 16 bits buffers")
					vs_cb.DepthMask = GSVector2i(0x0000FFFF, 0x0000FFFF);
					m_om_dssel.ztst = ZTST_ALWAYS;
				}
			}
		}
	}

	// FIXME Opengl support half pixel center (as dx10). Code could be easier!!!
	float sx = 2.0f * rtscale.x / (rtsize.x << 4);
	float sy = 2.0f * rtscale.y / (rtsize.y << 4);
	float ox = (float)(int)m_context->XYOFFSET.OFX;
	float oy = (float)(int)m_context->XYOFFSET.OFY;
	float ox2 = -1.0f / rtsize.x;
	float oy2 = -1.0f / rtsize.y;

	//This hack subtracts around half a pixel from OFX and OFY. (Cannot do this directly,
	//because DX10 and DX9 have a different pixel center.)
	//
	//The resulting shifted output aligns better with common blending / corona / blurring effects,
	//but introduces a few bad pixels on the edges.

	if (rt && rt->LikelyOffset)
	{
		ox2 *= rt->OffsetHack_modx;
		oy2 *= rt->OffsetHack_mody;
	}

	// Note: DX does y *= -1.0
	vs_cb.Vertex_Scale_Offset = GSVector4(sx, sy, ox * sx + ox2 + 1, oy * sy + oy2 + 1);
	// END of FIXME

	// GS_SPRITE_CLASS are already flat (either by CPU or the GS)
	m_ps_sel.iip = (m_vt.m_primclass == GS_SPRITE_CLASS) ? 1 : PRIM->IIP;

	if (DATE_GL45) {
		m_ps_sel.date = 5 + m_context->TEST.DATM;
	} else if (DATE_one) {
		m_require_one_barrier = true;
		m_ps_sel.date       = 5 + m_context->TEST.DATM;
		m_om_dssel.date     = 1;
		m_om_dssel.date_one = 1;
	} else if (DATE) {
		if (DATE_GL42)
			m_ps_sel.date = 1 + m_context->TEST.DATM;
		else
			m_om_dssel.date = 1;
	}

	m_ps_sel.fba = m_context->FBA.FBA;

	if (PRIM->FGE)
	{
		m_ps_sel.fog = 1;

		GSVector4 fc = GSVector4::rgba32(m_env.FOGCOL.u32[0]);
#if _M_SSE >= 0x401
		// Blend AREF to avoid to load a random value for alpha (dirty cache)
		ps_cb.FogColor_AREF = fc.blend32<8>(ps_cb.FogColor_AREF);
#else
		ps_cb.FogColor_AREF = fc;
#endif
	}

	if (m_context->TEST.ATE)
		m_ps_sel.atst = m_context->TEST.ATST;
	else
		m_ps_sel.atst = ATST_ALWAYS;

	if (m_context->TEST.ATE && m_context->TEST.ATST > 1)
		ps_cb.FogColor_AREF.a = (float)m_context->TEST.AREF;

	// By default don't use texture
	m_ps_sel.tfx = 4;
	int  atst = m_ps_sel.atst;

	if (tex) {
		EmulateTextureSampler(tex);
	}
	// Always bind the RT. This way special effect can use it.
	dev->PSSetShaderResource(3, rt);


	// GS

#if 0
	if (m_vt.m_primclass == GS_POINT_CLASS) {
		// Upscaling point will create aliasing because point has a size of 0 pixels.
		// This code tries to replace point with sprite. So a point in 4x will be replaced by
		// a 4x4 sprite.
		m_gs_sel.point = 1;
		// FIXME this formula is potentially wrong
		GSVector4 point_size = GSVector4(rtscale.x / rtsize.x, rtscale.y / rtsize.y) * 2.0f;
		vs_cb.TextureScale = vs_cb.TextureScale.xyxy(point_size);
	}
#endif
	m_gs_sel.sprite = m_vt.m_primclass == GS_SPRITE_CLASS;

	dev->SetupPipeline(m_vs_sel, m_gs_sel, m_ps_sel);

	// rs
	const GSVector4& hacked_scissor = m_channel_shuffle ? GSVector4(0, 0, 1024, 1024) : m_context->scissor.in;
	GSVector4i scissor = GSVector4i(GSVector4(rtscale).xyxy() * hacked_scissor).rintersect(GSVector4i(rtsize).zwxy());

	SetupIA();

	dev->OMSetColorMaskState(m_om_csel);
	dev->SetupOM(m_om_dssel);

	dev->SetupCB(&vs_cb, &ps_cb);

	if (DATE_GL42) {
		GL_PUSH("Date GL42");
		// It could be good idea to use stencil in the same time.
		// Early stencil test will reduce the number of atomic-load operation

		// Create an r32i image that will contain primitive ID
		// Note: do it at the beginning because the clean will dirty the FBO state
		//dev->InitPrimDateTexture(rtsize.x, rtsize.y);

		// I don't know how much is it legal to mount rt as Texture/RT. No write is done.
		// In doubt let's detach RT.
		dev->OMSetRenderTargets(NULL, ds, &scissor);

		// Don't write anything on the color buffer
		// Neither in the depth buffer
		glDepthMask(false);
		// Compute primitiveID max that pass the date test (Draw without barrier)
		dev->DrawIndexedPrimitive();

		// Ask PS to discard shader above the primitiveID max
		glDepthMask(GLState::depth_mask);

		m_ps_sel.date = 3;
		dev->SetupPipeline(m_vs_sel, m_gs_sel, m_ps_sel);

		// Be sure that first pass is finished !
		dev->Barrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	}

	if (m_ps_sel.hdr) {
		hdr_rt = dev->CreateTexture(rtsize.x, rtsize.y, GL_RGBA32F);

		dev->CopyRectConv(rt, hdr_rt, ComputeBoundingBox(rtscale, rtsize), false);

		dev->OMSetRenderTargets(hdr_rt, ds, &scissor);
	} else {
		dev->OMSetRenderTargets(rt, ds, &scissor);
	}

	if (m_context->TEST.DoFirstPass())
	{
		SendDraw();
	}

	if (m_context->TEST.DoSecondPass())
	{
		ASSERT(!m_env.PABE.PABE);

		static const uint32 iatst[] = {1, 0, 5, 6, 7, 2, 3, 4};

		m_ps_sel.atst = iatst[atst];
		if (tex && tex->m_spritehack_t && (m_ps_sel.atst == 2)) {
			m_ps_sel.atst = 1;
		}

		dev->SetupPipeline(m_vs_sel, m_gs_sel, m_ps_sel);

		bool z = m_om_dssel.zwe;
		bool r = m_om_csel.wr;
		bool g = m_om_csel.wg;
		bool b = m_om_csel.wb;
		bool a = m_om_csel.wa;

		switch(m_context->TEST.AFAIL)
		{
			case AFAIL_KEEP: z = r = g = b = a = false; break; // none
			case AFAIL_FB_ONLY: z = false; break; // rgba
			case AFAIL_ZB_ONLY: r = g = b = a = false; break; // z
			case AFAIL_RGB_ONLY: z = a = false; break; // rgb
			default: __assume(0);
		}

		if (z || r || g || b || a)
		{
			m_om_dssel.zwe = z;
			m_om_csel.wr = r;
			m_om_csel.wg = g;
			m_om_csel.wb = b;
			m_om_csel.wa = a;

			dev->OMSetColorMaskState(m_om_csel);
			dev->SetupOM(m_om_dssel);

			SendDraw();
		}
	}

	if (DATE_GL42) {
		dev->RecycleDateTexture();
	}

	dev->EndScene();

	// Warning: EndScene must be called before StretchRect otherwise
	// vertices will be overwritten. Trust me you don't want to do that.
	if (hdr_rt) {
		GSVector4 dRect(ComputeBoundingBox(rtscale, rtsize));
		GSVector4 sRect = dRect / GSVector4(rtsize.x, rtsize.y).xyxy();
		dev->StretchRect(hdr_rt, sRect, rt, dRect, ShaderConvert_MOD_256, false);

		dev->Recycle(hdr_rt);
	}
}
