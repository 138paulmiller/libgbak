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

/*-------------------------Bitmap Mode --------------------------------------*/

void gba_put_pixel(int x, int y, uchar r, uchar g, uchar b);

// this function takes a video buffer and returns to you the other one */
void gba_swap_buffer();

/* ---------------------- Tiled Mode ------------------------------*/

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

//load the palette into the sprite palette memory block 
void gba_obj_palette(const ushort* palette_data);

//load the image into the sprite image memory block 
void gba_obj_img(const uchar* image_data, uint width, uint height);

//load the sprite object data into the sprite object memory block 
void gba_obj(const ushort* obj_data, uint size);

/* ---------------------- Memory ------------------------------*/

// Direct access to VRAM
ushort* gba_char_block(unsigned long block_n);
ushort* gba_screen_block(unsigned long block_n);
unsigned long gba_char_block_offset(ushort* block);
unsigned long gba_screen_block_offset(ushort* block);

// Copy data using DMA format
void gba_dma16_copy(ushort* dest, const ushort* source, int size) ;
void gba_dma32_copy(uint* dest, const uint* source, int size);

#endif //GBAK_H