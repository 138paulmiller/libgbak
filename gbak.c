#include "gbak.h"

// sprite image mapping 2D maps char block as 2D 8x8 tiles as image 
#define SPRITE_MAP_2D 0x0

// Sprite map 1D reads row of 8x8 tiles in char block as image
#define SPRITE_MAP_1D 0x40

// Enable main display control to draw sprites
#define SPRITE_ENABLE 0x1000

// Enable the background at index N
#define BG_ENABLE_0 0x100
#define BG_ENABLE_1 0x200
#define BG_ENABLE_2 0x400
#define BG_ENABLE_3 0x800

#define MOSAIC_ENABLE 0

#define DISPLAY_BACK_BUFFER 0x10;

/* 
	VRAM (video ram) is a pointer to base memory addr of where graphics data is stored
	spans from 06000000 to 06017FFF (64 kb)
 	Size of address is 2 bytes (short) used since ports are 16 bits large
*/
volatile ushort* vram = (volatile ushort*) 0x06000000;

/*
  	Display control register is 4 bytes and is used to control graphic modes
	Bits	|F	  |E    |D    |C	| B	  | A	 | 9	| 8   | 7  | 6   | 5  | 4  | 3  |2   1   0
	Field	|ObjW |Win1 |Win0 |Obj  | BG3 |	BG2  | BG1  | BG0 |	FB | OM  | HB | PS | GB | Mode

    Mode: Sets video mode. (GBA_MODE0..4) 0, 1, 2 are tiled modes; 3, 4, 5 are bitmap modes.
    GB  : Is set if cartridge is a GBC game. Read-only.
    PS  : Page select. Modes 4 and 5 can use page flipping for smoother animation. This bit selects the displayed page (and allowing the other one to be drawn on without artifacts).
    HB  : Allows access to OAM in an HBlank. OAM is normally locked in VDraw. Will reduce the amount of sprite pixels rendered per line.
    OM  : Object mapping mode. Tile memory can be seen as a 32x32 matrix of tiles. 
          When sprites are composed of multiple tiles high. this bit tells whether the
          2D mapping, OM=0: - Next row of tiles are beneath the previous
          1D mapping, OM=1: - Next row of tiles are to the right
    FB  : Force a screen blank.
    BG0-BG3, Obj:  Enables rendering of the corresponding background and sprites. (BG_ENABLE, SPRITE_ENABLE)
    W0-OW:	Enables the use of windows 0, 1 and Object window, respectively. Windows can be used to mask out certain areas (like the lamp did in Zelda:LTTP)
*/
volatile unsigned long* display_control = (volatile unsigned long*) 0x04000000; //note 4 bytes is long addr

/*
  Display status register is 2 bytes and is used to read display statuses
	F E D C | B A 9 8 | 7 6 5 4  3 2 1 0 
	T T T T | T T T T | X X Y H  V Z G W 
*/
volatile ushort* display_status = (volatile ushort*) 0x04000004;

/*
	Read-only location of y location of display.  
	GBA display height is 160 and is followed by 68 lines of vblank period. 	
 	Waiting for value to reach 160 (vblank) before updating can sync display to 60 fps (Hz). 
*/
volatile ushort* scan_vcount = (volatile ushort*) 0x04000006;

/* 
Button state register 
    Holds the bits which indicate whether each button has been pressed

	Bits    | 9 | 8 |   7  |  6 |   5  |  4    |   3   |   2    | 1 | 0 |
	Button  | L | R | DOWN | UP | LEFT | RIGHT | START | SELECT | B | A |
 */
volatile ushort* buttons = (volatile ushort*) 0x04000130;

/* ----------------------- Begin Tiled Mode ------------------------------------
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
	    - If Char blocks 0 and 1 are used, screen blocks 16-31 are available
	
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

/*
	Background control registers (2 bytes) - Write Only

	Bits   |F E  | D    |C B A 9 8 |7   |6	 |5 4	| 3 2	| 1 0
	Field  |Size |Wrap  |SBB 	   |CM  |Mos | 	~ 	| CBB 	| P	
	
	Size:		Regular		Affine 	(Tile WxH)
		0 	 	32x32 		16x16
		1 		64x32 	 	32x32
		2 		32x64 	 	64x64
		3 		64x64 	  	128x128
	
	Wrap: If set, wraps background vertically and horizontally on display
	SBB : Screen Base Block.Values 0-31 base screenblock for tilemap indices
	CM	: Color Mode. 0 = 16 colors (4bpp), 1 = 256 colors (8bpp) 
	Mos : If set, enables mosaic effect
	CBB : Character Base Block(0-3) character block for tilesheet image
	P   : Determines priority of background (used for draw order 0-3)	
*/
volatile ushort* bg0_controls = (volatile ushort*) 0x4000008;
volatile ushort* bg1_controls = (volatile ushort*) 0x400000a;
volatile ushort* bg2_controls = (volatile ushort*) 0x400000c;
volatile ushort* bg3_controls = (volatile ushort*) 0x400000e; 

/*
	Background scroll registers (2 bytes) - Write Only
    Control the rendering offset for the corresponding background
*/
volatile short* bg0_scroll_x = (volatile short*) 0x4000010;
volatile short* bg1_scroll_x = (volatile short*) 0x4000014;
volatile short* bg2_scroll_x = (volatile short*) 0x4000018;
volatile short* bg3_scroll_x = (volatile short*) 0x400001c;

volatile short* bg0_scroll_y = (volatile short*) 0x4000012;
volatile short* bg1_scroll_y = (volatile short*) 0x4000016;
volatile short* bg2_scroll_y = (volatile short*) 0x400001a;
volatile short* bg3_scroll_y = (volatile short*) 0x400001e; 

/*
	Palettes are used to store all colors used by an image
	Background palette is at 05000000h 
 	Sprite palette is at 05000200h
	Each palette is 0x0200 (256) bytes
*/
volatile ushort* bg_palette = (volatile ushort*) 0x05000000;
volatile ushort* sprite_palette = (volatile ushort*) 0x05000200;

/*
    Address where sprite image data is stored. 
    Each value is an 2 byte index into the color palette
*/
volatile ushort* sprite_image_block = (volatile ushort*) 0x6010000;

/*
OAM - Object Attribute Memory
    Address where the sprite instances are stored (attributes)
    Each Sprite is defined by 8 bytes (2 bytes per attribute)
OBJ Attribute 0 (R/W)

  Bit   Expl.
  0-7   Y-Coordinate           (0-255)
  8     Rotation/Scaling Flag  (0=Off, 1=On)
  When Rotation/Scaling used (Attribute 0, bit 8 set):
    9     Double-Size Flag     (0=Normal, 1=Double)
  When Rotation/Scaling not used (Attribute 0, bit 8 cleared):
    9     OBJ Disable          (0=Normal, 1=Not displayed)
  10-11 OBJ Mode  (0=Normal, 1=Semi-Transparent, 2=OBJ Window, 3=Prohibited)
  12    OBJ Mosaic             (0=Off, 1=On)
  13    Colors/Palettes        (0=16/16, 1=256/1)
  14-15 OBJ Shape              (0=Square,1=Horizontal,2=Vertical,3=Prohibited)

Caution: A very large OBJ (of 128 pixels vertically, ie. a 64 pixels OBJ in a Double Size area) located at Y>128 will be treated as at Y>-128, the OBJ is then displayed parts offscreen at the TOP of the display, it is then NOT displayed at the bottom.

OBJ Attribute 1 (R/W)

  Bit   Details.
  0-8   X-Coordinate           (0-511) 
          When Rotation/Scaling used (Attribute 0, bit 8 set):
    9-13  Rotation/Scaling Parameter Selection (0-31)
          (Selects one of the 32 Rotation/Scaling Parameters that
          can be defined in OAM, for details read next chapter.)
          When Rotation/Scaling not used (Attribute 0, bit 8 cleared):
    9-11  Not used
    12    Horizontal Flip      (0=Normal, 1=Mirrored)
    13    Vertical Flip        (0=Normal, 1=Mirrored)
    14-15 OBJ Size               (0..3, depends on OBJ Shape, see Attr 0)
          Size  Square   Horizontal  Vertical
          0     8x8      16x8        8x16
          1     16x16    32x8        8x32
          2     32x32    32x16       16x32
          3     64x64    64x32       32x64

OBJ Attribute 2 (Read/Write)
  Bit   Expl.
  0-9   Character Name          (0-1023=Tile Number)
  10-11 Priority relative to BG (0-3; 0=Highest)
  12-15 Palette Number   (0-15) (Not used in 256 color/1 palette mode)
 
*/
volatile ushort* sprite_obj_block = (volatile ushort*) 0x7000000;

// ----------------------------- End Tiled Mode ---------------------------------//

// ----------------------------- Begin Pixel Mode ---------------------------------//

/*
    Buffer addresses to be used when double-buffering is enabled (MODE4)
*/
volatile ushort* active_buffer = (volatile ushort*) 0x06000000;
volatile ushort* front_buffer = (volatile ushort*)  0x06000000;
volatile ushort* back_buffer = (volatile ushort*)   0x0600A000;

// The color palette used in graphics Mode 4
volatile ushort* palette = (volatile ushort*) 0x5000000;
uchar palette_count = 0; 

// ----------------------------- End Pixel Mode ---------------------------------//

/* ------------------------------  Begin DMA  ------------------------------------ 
  DMA (Direct Memory Address) control register (32- bit)

| 1F | 1E | 1D 1C | 1B | 1A | 19 | 18 17 |16 15 | 14 13 12 11 10 | F E D C B A 9 8 7 6 5 4 3 2 1 0
| En | I  | Tm    | -  | C  | R  | Sa    | Da   | -              | 		Size 
 En - enable flag
 I  - Interrupt request, raises interuupt if set
 Tm - Timing Mode. Specifies when the transfer should start.
    00: immediately
    01: at vblank
    10: at hblank
    11: at each scanline?? (unsure untested) 
 C  - chunk size: if 0 halfword(16bit) else if 1 word(32bit)
 R  - Repeats at every vblank or hblank if timing mode is set to either
 Da - Destination adjustment, address behavior after each transfer
    00: increment after each transfer (default)
    01: decrement after each transfer
    10: none; address is fixed
    11: increment the destination during the transfer, and reset it.

Sa - Source Adjustment. Works just like the two bits for the destination 
		Except 11 is not a valid opcode.
Size - the amount of data to be transfered  
*/

// flag enables DMA when written to DMA control register 
#define DMA_ENABLE       0x80000000
#define DMA_SRC_FIXED    0x01000000
#define DMA_AT_REFRESH   0x30000000
#define DMA_AT_VBLANK   0x10000000
#define DMA_AT_HBLANK   0x20000000

// DMA transfer size flags
#define DMA_16 0x00000000
#define DMA_32 0x04000000

volatile uint* dma_control = (volatile uint*) 0x40000DC;

// DMA register for address of data's source location 
volatile uint* dma_src = (volatile uint*) 0x40000D4;

// DMA register for address of data's destination 
volatile uint* dma_dest = (volatile uint*) 0x40000D8;

//------------------------------  End DMA  ------------------------------------// 
// ------------------------------ Begin Timer --------------------------------//
#define TIMER_FREQ_1 0
#define TIMER_FREQ_64 1
#define TIMER_FREQ_256  2
#define TIMER_FREQ_1024 3
#define TIMER_CASCADE 0x4	//Increments timer when preceding timer overflows
#define TIMER_ENABLE 0x0080	// Enable timer

volatile ushort* timer0			= (volatile ushort*)0x04000100;
volatile ushort* timer0_control	= (volatile ushort*)0x04000102;	
volatile ushort* timer1			= (volatile ushort*)0x04000104;	
volatile ushort* timer1_control	= (volatile ushort*)0x04000106;
volatile ushort* timer2			= (volatile ushort*)0x04000108;
volatile ushort* timer2_control	= (volatile ushort*)0x0400010A;
volatile ushort* timer3			= (volatile ushort*)0x0400010C;
volatile ushort* timer3_control	= (volatile ushort*)0x0400010E;

void gba_init(struct gba_config config)
{
    switch(config.mode)
    {
        case 0:
            *display_control = GBA_MODE0;
            break;   
        case 1:
            *display_control = GBA_MODE1;
            break;
        case 2:
            *display_control = GBA_MODE2;
            break;
        case 3:
            *display_control = GBA_MODE3 | BG_ENABLE_2;
            return;
        case 4:
            *display_control = GBA_MODE4 | BG_ENABLE_2;
            return;
    }

    switch(config.num_bgs)
    {
        case 0: 
            break;
        case 1:
            *display_control |= BG_ENABLE_0 ;
            break;   
        case 2:
            *display_control |= BG_ENABLE_0 | BG_ENABLE_1;
            break;
        case 3:
            *display_control |= BG_ENABLE_0 | BG_ENABLE_1 | BG_ENABLE_2;
            break;
        case 4:
            *display_control |= BG_ENABLE_0 | BG_ENABLE_1 | BG_ENABLE_2 | BG_ENABLE_3;
            break;
    }

    if(config.num_bgs > 0)
    {
        if(config.use_sprite_2d > 0)
        { 
            *display_control |= SPRITE_ENABLE | SPRITE_MAP_2D;
        }
        else
        {
            *display_control |= SPRITE_ENABLE | SPRITE_MAP_1D;
        }
    }
    gba_reset();
}

// wait for the screen to be in vblank
void gba_vsync( ) 
{
    /* wait until all 160 vram lines have been updated */
    while (*scan_vcount >= 160) {}
	while (*scan_vcount < 160) {}
}

void gba_wait(uint sec)
{
	*timer2= -0x3000;  // 0x3000 ticks till overflow (incrementing timer 3)
	*timer2_control = TIMER_ENABLE | TIMER_FREQ_1024;   // we're using the 1024 cycle timer
	*timer3_control= TIMER_ENABLE | TIMER_CASCADE;

	uint cycles = 0;
	while(1)
	{
		gba_vsync();
		if(*timer3 != -1)
		{
			cycles++;
			if((cycles / 60) >= sec)
			{
				return;
			}
            *timer2= -0x3000;          // 0x4000 ticks till overflow
			*timer2_control = TIMER_ENABLE | TIMER_FREQ_1024;
            *timer3_control = TIMER_ENABLE | TIMER_CASCADE;
		}
	}
}

void gba_reset()
{
    *bg0_controls = 0;
    *bg1_controls = 0;
    *bg2_controls = 0;
    *bg3_controls = 0;
    *bg0_scroll_x = 0;
    *bg1_scroll_x = 0;
    *bg2_scroll_x = 0;
    *bg3_scroll_x = 0;
    *bg0_scroll_y = 0;
    *bg1_scroll_y = 0;
    *bg2_scroll_y = 0;
    *bg3_scroll_y = 0; 

	/* clear the map to be all blanks */
    for (int char_block_n = 0; char_block_n < 4; char_block_n++) 
	{
        ushort* base = gba_char_block(0);
        for(int i = 0; i < 8192 * 4; ++i)
        {
            *base = 0;
            ++base;
        }
    }
}

void gba_run(void(*on_draw)(), void(*on_input)())
{
	while (1)
	{
		gba_vsync();

        if(on_input) on_input();
        if(on_draw) on_draw();
	}
}

uchar gba_button_state(ushort button)
{
    //if the buttons register anded with the button is zero, then button is down
	//return not zero if 0
    return !(*buttons & button);
}
/*
 16-bit int color is bgr ranged 0-32 (5 bits each color)
    bbbbbgggggrrrrr 
*/
#define RGB2PIXEL(_r, _g, _b) ((_b & 0x1f) << 10) | ((_g & 0x1f) << 5) | (_r & 0x1f)

inline void gba_pixel(int x, int y, uchar r, uchar g, uchar b) 
{
	active_buffer[y * GBA_SCREEN_WIDTH + x] = RGB2PIXEL(r,g,b);
}

inline void gba_clear_palette()
{
    palette_count = 0;
    for(ushort i = 0; i < GBA_PALETTE_COUNT; ++i)
    {
        palette[i] = 0;
    }
}

inline uchar gba_add_color(uchar r, uchar g, uchar b) 
{
    ushort color = RGB2PIXEL(r,g,b);

    palette[palette_count] = color;
    palette_count++;
    return palette_count - 1;
}

inline void gba_clear_screen(uchar color_index)
{
    const ushort pixel = (color_index & 0x00ff) | (color_index << 8);
    gba_vram_fill16((ushort*)active_buffer, pixel, GBA_SCREEN_SIZE >> 1);
    *dma_control |= DMA_AT_REFRESH;
}

inline void gba_set_color(int x, int y, uchar color_index)
{
    //find the pixel index which is the regular index divided by two */
    const ushort index = ((y * GBA_SCREEN_WIDTH + x) >> 1);
    ushort pixel = active_buffer[index];
    // Join the pixels by column
    if (x & 1) //is odd 
    {
        active_buffer[index] = (color_index << 8) | (pixel & 0x00ff);
    } 
    else 
    {
        active_buffer[index] = (pixel & 0xff00) | color_index;
    }
    return;
}

inline uchar gba_color_count()
{
    return palette_count;
}

inline void gba_refresh_screen() 
{
    //Swap buffers
    if(active_buffer == front_buffer) 
    {
        // clear back buffer bit and return back buffer pointer */
        *display_control &= ~DISPLAY_BACK_BUFFER;
        active_buffer = back_buffer;
        return;
    } 
    
    // set back buffer bit and return front buffer */
    *display_control |= DISPLAY_BACK_BUFFER;
    active_buffer = front_buffer;
}

inline void gba_draw_rect(uchar x, uchar y, uchar w, uchar h, uchar color_index) 
{
    uchar cx,cy;
	const uchar cx_end = x + w;
    const uchar cy_end = y + h; 
    for (cy = y; cy < cy_end; cy++) 
	{
        for (cx = x; cx < cx_end; cx++) 
        {
            switch(cx % 10)
            {
                case 9: gba_set_color(cx++, cy, color_index);
                case 8: gba_set_color(cx++, cy, color_index);
                case 7: gba_set_color(cx++, cy, color_index);
                case 6: gba_set_color(cx++, cy, color_index);
                case 5: gba_set_color(cx++, cy, color_index);
                case 4: gba_set_color(cx++, cy, color_index);
                case 3: gba_set_color(cx++, cy, color_index);
                case 2: gba_set_color(cx++, cy, color_index);
                case 1: gba_set_color(cx++, cy, color_index);
                case 0: gba_set_color(cx, cy, color_index);
            }
        }
    }
}

// ---------------------------- Tile Mode ------------------------------------
void gba_bg_init(uchar bg_index, uint char_block_n, uint screen_block_n,
							ushort size, ushort priority, ushort wrap)
{
	ushort control_flags = priority 
			  | (char_block_n << 2)     
			  | (MOSAIC_ENABLE << 6)  
			  | (GBA_COLOR_MODE << 7) 
			  | (screen_block_n << 8)
			  | (wrap << 13) 
			  | (size << 14);

	switch(bg_index)
	{
		case 0: *bg0_controls = control_flags; 
				break;
		case 1: *bg1_controls = control_flags; 
				break;
		case 2: *bg2_controls = control_flags; 
				break;
		case 3: *bg3_controls = control_flags; 
				break;
	}
}

void gba_bg_palette(const ushort* palette_data)
{
  	gba_copy16((ushort*)bg_palette, palette_data, GBA_PALETTE_COUNT);
}

// Load background image data into char block n
void gba_bg_image(uint char_block_n, const uchar* image_data, uint width, uint height)
{
    ushort* char_block = gba_char_block(char_block_n);
    //divide by 2, Since we are transfer bytes, but the dma expects 16 bit values, we are counting 2 chars per short
	gba_copy16(char_block, (ushort*)image_data, (width * height) / 2);
}

// Load background image data into char block n
void gba_bg_tilemap(uint screen_block_n, const ushort* tilemap_data, uint width, uint height)
{
	//load tilemap data directly into nth screen block
	gba_copy16(gba_screen_block(screen_block_n), tilemap_data, width * height);
}

void gba_bg_scroll(int bg_index, int offset_x, int offset_y)
{
	switch(bg_index)
	{
		case 0: *bg0_scroll_x = offset_x; 
				*bg0_scroll_y = offset_y;
				break;
		case 1: *bg1_scroll_x = offset_x; 
				*bg1_scroll_y = offset_y;
				break;
		case 2: *bg2_scroll_x = offset_x; 
				*bg2_scroll_y = offset_y;
				break;
		case 3: *bg3_scroll_x = offset_x; 
				*bg3_scroll_y = offset_y;
				break;
    }
}

void gba_obj_palette(const ushort* palette_data)
{
    gba_copy16((ushort*)sprite_palette, palette_data, GBA_PALETTE_COUNT);
}

void gba_obj_img(const uchar* image_data, uint width, uint height)
{
    //divide by 2, Since we are transfer bytes, but the dma expects 16 bit values, we are counting 2 chars per short
    const uint count = (width * height) / 2; 
    gba_copy16((ushort*)sprite_image_block, (ushort*)image_data, count);
}

void gba_obj(const ushort* obj_data, uint size)
{
    gba_copy16((ushort*) sprite_obj_block, obj_data, size);
}

// Returns base address of nth character block (0-3)	
ushort* gba_char_block(unsigned long block_n)
{
	//calculate distance from base vram addr by multiplying by char block size 16 kb
	return (ushort*)(0x06000000 + block_n*0x4000); //0x4000 = 16 kb
}

//Returns base address of nth screen block (0-31)	
ushort* gba_screen_block(unsigned long block_n)
{
	//calculate distance from base vram addr by multiplying by screen block size 2kb
	return (ushort*)(0x06000000 + block_n*0x800); //0x800 = 2 kb
}

//Given the char block base address return the nth block index (0-4)	
unsigned long gba_char_block_offset(ushort* block)
{
	//calculate distance from base vram addr
	return (unsigned long)(block - vram)/0x4000;
}

//Given the screen block base address return the nth block index (0-31)
unsigned long gba_screen_block_offset(ushort* block)
{
	//calculate distance from base vram addr
	return (unsigned long)(block - vram)/0x800;
}

void gba_copy16(ushort* dest, const ushort* source, ushort size) 
{
    *dma_src = (uint) source;
    *dma_dest = (uint) dest;
    *dma_control = size | DMA_16 | DMA_ENABLE;
}

void gba_copy32(uint* dest, const uint* source, ushort size) 
{
    *dma_src = (uint) source;
    *dma_dest = (uint) dest;
    *dma_control = size | DMA_32 | DMA_ENABLE;
}

void gba_fill16(ushort* dest, const ushort* source, ushort count)
{
    *dma_src = (uint) source;
    *dma_dest = (uint) dest;
    *dma_control = count | DMA_16 | DMA_ENABLE | DMA_SRC_FIXED;  
}

void gba_fill32(uint* dest, const uint* source, ushort count)
{
    *dma_src = (uint) source;
    *dma_dest = (uint) dest;
    *dma_control = count | DMA_32 | DMA_ENABLE | DMA_SRC_FIXED;
}

void gba_vram_copy16(ushort* dest, const ushort* source, ushort size) 
{
    *dma_src = (uint) source;
    *dma_dest = (uint) dest;
    *dma_control = size | DMA_16 | DMA_ENABLE | DMA_AT_REFRESH;
}

void gba_vram_copy32(uint* dest, const uint* source, ushort size) 
{
    *dma_src = (uint) source;
    *dma_dest = (uint) dest;
    *dma_control = size | DMA_32 | DMA_ENABLE | DMA_AT_REFRESH;
}

ushort pending_source16;
void gba_vram_fill16(ushort* dest, const ushort source, ushort count)
{
    pending_source16 = source;
    *dma_src = (uint)((ushort*)&pending_source16);
    *dma_dest = (uint)(dest);
    *dma_control = (count) | DMA_16 | DMA_ENABLE | DMA_SRC_FIXED;
    *dma_control |= DMA_AT_REFRESH;
}

uint pending_source32;
void gba_vram_fill32(uint* dest, const uint source, ushort count)
{
    pending_source32 = source;
    *dma_src = (uint)((ushort*)&pending_source32);
    *dma_dest = (uint)(dest);
    *dma_control = (count) | DMA_32 | DMA_ENABLE | DMA_SRC_FIXED;
    *dma_control |= DMA_AT_REFRESH;
}
