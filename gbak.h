#ifndef GBAK_H
#define GBAK_H

// TODO: Rename BG_Map to TileMap. Tileset to TileSheet 
#include <stddef.h>

typedef unsigned int uint;
typedef unsigned long ulong;
typedef unsigned short ushort;
typedef unsigned char uchar;

/*
Display Modes
    Mode 0: 4 available tiled backgrounds. None can be rotated or scaled.
    Mode 1: 3 available tiled backgrounds. One can be rotated and scaled.
    Mode 2: 2 available tiled backgrounds. Both can be rotated and scaled. 
    Mode 3: Bitmap mode
    Mode 4: Double buffered draw mode with palette colors (pixels are 1 btye) uses gba_swap_buffer to refresh.


    Mode:	BG0 	BG1 	BG2 	BG3
    -----------------------------------
    0 		reg 	reg 	reg 	reg
    1 		reg 	reg 	aff 	-
    2 		- 		- 		aff 	aff 
*/
#define GBA_MODE0 0x00
#define GBA_MODE1 0x01
#define GBA_MODE2 0x02
#define GBA_MODE3 0x03
#define GBA_MODE4 0x04

#define GBA_BUTTON_A (1 << 0)
#define GBA_BUTTON_B (1 << 1)
#define GBA_BUTTON_SELECT (1 << 2)
#define GBA_BUTTON_START (1 << 3)
#define GBA_BUTTON_RIGHT (1 << 4)
#define GBA_BUTTON_LEFT (1 << 5)
#define GBA_BUTTON_UP (1 << 6)
#define GBA_BUTTON_DOWN (1 << 7)
#define GBA_BUTTON_R (1 << 8)
#define GBA_BUTTON_L (1 << 9)

// Max number of backgrounds
#define GBA_BG_COUNT 4

#define GBA_COLOR_MODE 1 //256 colors

// Max number of colors in palette block
#define GBA_PALETTE_COUNT 256

//Screen size
#define GBA_SCREEN_WIDTH 240
#define GBA_SCREEN_HEIGHT 160
#define GBA_SCREEN_SIZE GBA_SCREEN_HEIGHT * GBA_SCREEN_WIDTH

//Images in memory are represented as 8x8 tiles 
#define GBA_TILE_WIDTH 8
#define GBA_TILE_HEIGHT 8

struct gba_config
{
	uchar mode; // Mode 0-1
	uchar num_bgs; // Number of tile backgrounds used 0-4
	uchar use_sprite_2d;
};

// Initialize the video mode
void gba_init(struct gba_config config);

// Reset VRAM and controls
void gba_reset();

// wait for the screen to be in vblank
void gba_vsync();
void gba_wait(uint sec);

// Checks whether a particular button has been pressed. Returns 0 if not
uchar gba_button_state(ushort button);

/*-------------------------Immediate Modes (3 and 4) --------------------------------------*/

// Mode 3
void gba_pixel(int x, int y, uchar r, uchar g, uchar b);

// Mode 4 (Buffered screen. Prevent tearing)
void gba_clear_palette();
uchar gba_add_color(uchar r, uchar g, uchar b);
void gba_set_color(int x, int y, uchar color_index);
uchar gba_color_count();
void gba_clear_screen(uchar color);
void gba_refresh_screen();
void gba_draw_rect(uchar x, uchar y, uchar w, uchar h, uchar  color_index); 

/* ---------------------- Tiled Background ------------------------------*/

void gba_bg_init(uchar bg_index, uint char_block_n, uint screen_block_n, ushort size, ushort priority, ushort wrap);

void gba_bg_reset();

// Load background palette colors
void gba_bg_palette(const ushort* palette_data);

// Load background image data into char block n
void gba_bg_image(uint char_block_n, const uchar* image_data, uint width, uint height);

// Load background tile data into screen block n
void gba_bg_tilemap(uint screen_block_n, const ushort* tilemap_data, uint width, uint height);

// Load background palette colors
void gba_bg_scroll(int bg_index, int offset_x, int offset_y);

/* ---------------------- Tiled Objects (Sprites) ------------------------------*/
/*				size
			0		1		2		3
shape	0	8x8		16x16	32x32	64x64
		1	16x8	32x8	32x16	64x32
		2	8x16	8x32	16x32	32x64
*/

typedef enum gba_obj_size 
{
    GBA_OBJ_8_8,
    GBA_OBJ_16_16,
    GBA_OBJ_32_32,
    GBA_OBJ_64_64,
    GBA_OBJ_16_8,
    GBA_OBJ_32_8,
    GBA_OBJ_32_16,
    GBA_OBJ_64_32,
    GBA_OBJ_8_16,
    GBA_OBJ_8_32,
    GBA_OBJ_16_32,
    GBA_OBJ_32_64
} gba_obj_size;
#define GBA_OBJ_COUNT 128
#define GBA_OBJ_INVALID -1

//load the palette into the sprite palette memory block 
void gba_obj_palette(const ushort* palette_data);

//load the image into the sprite image memory block 
void gba_obj_image(const uchar* image_data, uint width, uint height);

//load the sprite object data into the sprite object memory block 
void gba_obj_data(const ushort* obj_data, uint size);
int gba_obj_new(gba_obj_size size, int priority);
uchar gba_obj_width(uint object_index);
void gba_obj_set_pos(uint object_index, int x, int y) ;
void gba_obj_get_pos(uint object_index, int *x, int *y);
void gba_obj_set_offset(uint object_index, int offset) ;
void gba_obj_move_by(uint object_index, int dx, int dy) ;
void gba_obj_flip(uint object_index, int h_flip, int v_flip);
void gba_obj_reset_all();
void gba_obj_update_all();

/* ---------------------- Memory and Utilities ------------------------------*/

/* 
Screen and Character blocks (memory representation of vram ) 
	1 block = 512 bytes
	2 blocks = 1024 bytes = 1 kilobyte = 0x0400
	- Screen blocks are 2 kb
		VRAM contains 32 Screen Blocks
	- Character blocks are 16 kb
		VRAM contains 4 char Blocks
	
    Character Block are used to store background image data (tileset)
	Screen Blocks are used to store map data (tilemap)
	
    Note:
	    - Screen and Character blocks are different ways to access chunks of VRAM,
	    - If Char blocks 0 and 1 are used (2 bgs), screen blocks 16-31 are available
	
	BLOCKS:
	Char		Screen Blocks
			----------------------------------------
	0		| 0  | 1  | 2  | 3  | 4  | 5  | 6  | 7  |
			---------------------------------------
	1		| 8  | 9  | 10 | 11 | 12 | 13 | 14 | 15 |
			----------------------------------------
	2		| 16 | 17 | 18 | 19 | 20 | 21 | 22 | 23 |
			----------------------------------------
	3		| 24 | 25 | 26 | 27 | 28 | 29 | 30 | 31 |
			----------------------------------------
*/
ushort* gba_char_block(unsigned long block_n);
ushort* gba_screen_block(unsigned long block_n);
unsigned long gba_char_block_offset(ushort* block);
unsigned long gba_screen_block_offset(ushort* block);

// Copy arrays using GBA DMA hardware 
void gba_copy16(ushort* dest, const ushort* source, ushort size) ;
void gba_copy32(uint* dest, const uint* source, ushort size);

// Fill array using GBA DMA hardware 
void gba_fill16(ushort* dest, const ushort* source, ushort count);
void gba_fill32(uint* dest, const uint* source, ushort count);

// VRAM-safe Copy arrays using GBA DMA hardware 
// note: will copy source on next vram refresh. So must ensure will not change until then
void gba_vram_copy16(ushort* dest, const ushort* source, ushort size) ;
void gba_vram_copy32(uint* dest, const uint* source, ushort size);

// VRAM-safe Fill array using GBA DMA hardware 
void gba_vram_fill16(ushort* dest, const ushort source, ushort count);
void gba_vram_fill32(uint* dest, const uint source, ushort count);


#endif //GBAK_H