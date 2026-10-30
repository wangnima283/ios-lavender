#pragma once

/*
 * Board selection for the SDM660/SDM636 boot shim.
 *
 * WHYRED (Redmi Note 5 / Note 5 Pro, SDM636) and LAVENDER (Redmi Note 7,
 * SDM660) share the GICv3, timer, SDCC v5 and continuous-splash layout.  The
 * only shim-visible difference is the panel height (2160 vs 2340 pixels).
 *
 * VIOLET (Redmi Note 7 Pro, SM6150/SDM675) keeps the 1080x2340 panel and the
 * same peripherals, but its ABL reserves the continuous-splash buffer at
 * 0x9c000000 instead of 0x9d400000 (18 MB earlier, same 36 MB size): see
 * resources/violet_dtb.bin, reserved-memory/cont_splash_region@9c000000.
 *
 * Build with -DBOARD_WHYRED for whyred; the default stays lavender so the
 * existing host tests and lavender images are unchanged.
 */
#if defined(BOARD_WHYRED)
#define BOARD_NAME	"whyred"
#define BOARD_SOC	"SDM636"
#define BOARD_PANEL_H	2160
#define BOARD_FB_BASE	0x9d400000UL
#elif defined(BOARD_VIOLET)
#define BOARD_NAME	"violet"
#define BOARD_SOC	"SM6150"
#define BOARD_PANEL_H	2340
#define BOARD_FB_BASE	0x9c000000UL
#else
#define BOARD_NAME	"lavender"
#define BOARD_SOC	"SDM660"
#define BOARD_PANEL_H	2340
#define BOARD_FB_BASE	0x9d400000UL
#endif

#define BOARD_PANEL_W	1080
#define FB_BASE		BOARD_FB_BASE	/* ABL continuous-splash framebuffer */
#define FB_W		BOARD_PANEL_W
#define FB_H		BOARD_PANEL_H
#define FB_STRIDE	(FB_W * 4)	/* 4320 bytes, a8r8g8b8 */
