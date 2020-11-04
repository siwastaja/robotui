/*
	PULUROBOT RN1-CLIENT  Stand-alone GUI client prototype

	(c) 2017-2018 Pulu Robotics and other contributors
	Maintainer: Antti Alhonen <antti.alhonen@iki.fi>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.


*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <dirent.h>

// For opening 3D point clouds from disk in posix environment:
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>


#include <GL/glew.h>
#include <SFML/System.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
//#include <glm/gtc/transform.hpp>

#include "client_memdisk.h"

extern "C" {
#include "../robotsoft/voxmap.h"
#include "../robotsoft/voxmap_memdisk.h"
}

#define RGBA32(r_,g_,b_,a_)  ((r_) | ((g_)<<8) | ((b_)<<16) | ((a_)<<24))


#if 0
#define MAX_LEVELS 8
#define LEVEL_EMPTY 0

// List of floor levels (not sorted), terminated by 0 (or by being the last index MAX_LEVELS-1)
typedef struct __attribute__((packed))
{
	uint16_t levels[MAX_LEVELS];
} map_block_2d_t;
typedef struct
{
	map_block_2d_t* blocks[8]; // Number of blocks is VOX_XS[rl]*VOX_YS[rl]
} map_piece_2d_t;
#endif

static sf::RenderWindow* p_win;


static uint8_t pages_exist[MAX_PAGES_X*MAX_PAGES_Y];

// Simple 2D texture representing the complete world, just one pixel per page.
static sf::Texture tex_full_map;


typedef struct
{
	sf::Texture* textures[MAX_RESOLEVELS];
	sf::Texture* routing_texture;
} map_2d_page_pile_t;
#define ROUTING_RL 1 // has to match the ROUTING_RL on the robot's routing.c
#define ROUTING_MAP_PAGE_W 128

map_2d_page_pile_t piles[MAX_PAGES_X*MAX_PAGES_Y];

int view2d_min_z = -300;
int view2d_max_z = 1900;

static char mapdir[1024];

typedef struct __attribute__((packed))
{
	uint32_t dummy1;
	uint16_t dummy2;
	uint8_t dummy3;
	uint8_t flags;
	uint32_t routing[ROUTING_MAP_PAGE_W][ROUTING_MAP_PAGE_W/32 + 1];
} routing_page_t;

//static int display_limit[4] = {1, 3, 6, 6};
//static int display_limit[4]   = {3, 6, 15, 15};
static int display_limit[4]   = {1, 1, 1, 1};

void set_display_limit(int val)
{
	for(int i=0; i<4; i++)
	{
		int newval = val*(i+1);
		if(newval < 1) newval = 1;
		else if(newval > 15) newval = 15;
		display_limit[i] = newval;
	}
}

static void load_page_pile_from_disk(int px, int py, int rl, int do_reload)
{
	assert(px >= 0 && px < MAX_PAGES_X && py >= 0 && py < MAX_PAGES_Y && rl >= 0 && rl < MAX_RESOLEVELS);

	// Load routing, if necessary, and if found.
	if((!piles[py*MAX_PAGES_X+px].routing_texture || do_reload))// && px==255 && py==255)
	{

		char fname[4096];
		snprintf(fname, 4096, "./current_routing/routing_%u_%u.routing", px, py);
		FILE* f = fopen(fname, "r");

		if(f)
		{
			routing_page_t rout_in; // about 2.5kB, in stack
			uint32_t* tmprout = (uint32_t*)malloc(ROUTING_MAP_PAGE_W*ROUTING_MAP_PAGE_W*sizeof(uint32_t));
			assert(tmprout);

			if(fread(&rout_in, sizeof(rout_in), 1, f) != 1 || ferror(f))
			{
				printf("ERROR: Reading file %s failed.\n", fname);
			}
			else
			{
				if(!piles[py*MAX_PAGES_X+px].routing_texture)
				{
					//printf("New texture\n");
					piles[py*MAX_PAGES_X+px].routing_texture = new sf::Texture;
					piles[py*MAX_PAGES_X+px].routing_texture->create(ROUTING_MAP_PAGE_W, ROUTING_MAP_PAGE_W);
					piles[py*MAX_PAGES_X+px].routing_texture->setSmooth(false);
				}


				for(int yy=0; yy<ROUTING_MAP_PAGE_W; yy++)
				{
					for(int xx=0; xx<ROUTING_MAP_PAGE_W; xx++)
					{
						int y = yy/32;
						int y_remain = yy%32;

						// In SFML coordinate system, y is mirrored - mirror in the texture.
		
						if(rout_in.routing[xx][y] & (1<<y_remain))
						{
							tmprout[(ROUTING_MAP_PAGE_W-1-yy)*ROUTING_MAP_PAGE_W+xx] = RGBA32(0,0,0,255);
						}
						else
						{
							tmprout[(ROUTING_MAP_PAGE_W-1-yy)*ROUTING_MAP_PAGE_W+xx] = RGBA32(255,255,255,255);
						}

					}

				}

				piles[py*MAX_PAGES_X+px].routing_texture->update((uint8_t*)tmprout);

			}

			free(tmprout);
			fclose(f);

		}
	}

	//printf("load_page_pile(%d,%d,rl%d)\n", px, py, rl);
	if(piles[py*MAX_PAGES_X+px].textures[rl] && !do_reload)
	{
		return;
	}

	int32_t* max_z = malloc(VOX_XS[rl]*VOX_YS[rl]*sizeof(int32_t));
	for(int i=0; i<VOX_XS[rl]*VOX_YS[rl]; i++)
		max_z[i] = INT32_MIN;

	int file_found = 0;

	for(int pz=0; pz < MAX_PAGES_Z; pz++)
	{
		voxmap_t voxmap;
		static char fnamebuf[2048];
		int ret = read_voxmap(&voxmap, gen_fname(mapdir, px, py, pz, rl, fnamebuf));

		if(ret >= 0)
		{
			file_found = 1;
//			printf("INFO: Succesfully loaded file from the disk (%d,%d,%d,rl%d)\n", px, py, pz, rl);

			assert(voxmap.header.xs == VOX_XS[rl]);
			assert(voxmap.header.ys == VOX_YS[rl]);
			assert(voxmap.header.zs == VOX_ZS[rl]);

			for(int xy=0; xy<VOX_XS[rl]*VOX_YS[rl]; xy++)
			{
				int max_accepted_oz = (view2d_max_z + (MAX_PAGES_Z/2)*MAP_PAGE_Z_H_MM)/VOX_UNITS[rl] - pz*VOX_ZS[rl];
				if(max_accepted_oz > VOX_ZS[rl]-1) max_accepted_oz = VOX_ZS[rl]-1;
				int z;
				for(z=max_accepted_oz; z >= 0; z--)
				{
					if((voxmap.voxels[xy*VOX_ZS[rl]+z] & 0x0f) >= display_limit[rl])
						break;
				}
				// z has highest z, or -1 if not voxel found at all
				
				// The pz loop goes from down to up: if we have anything, it's the biggest.
				if(z > -1)
				{
					// Test if we are outside of display range: if we are, only apply if there is no in-range reading yet.
					int new_max_z = pz*VOX_ZS[rl]+z;
					new_max_z *= VOX_UNITS[rl];
					new_max_z -= (MAX_PAGES_Z/2)*MAP_PAGE_Z_H_MM;

//					int prev_max_z = max_z[xy];
					//prev_max_z *= VOX_UNITS[rl];
					//prev_max_z -= (MAX_PAGES_Z/2)*MAP_PAGE_Z_H_MM;

//					if(new_max_z <= view2d_max_z) //||  // this is ok as is
//					   (prev_max_z < view2d_min_z || prev_max_z > view2d_max_z)) // it's bigger: only replace if the previous max was out-range
						max_z[xy] = new_max_z; //pz*VOX_ZS[rl]+z;
						
				}
			}
			deinit_voxmap(&voxmap);
		}
	}

	if(!file_found)
	{
		free(max_z);
		return;
	}

	uint8_t* tmpimg = malloc(VOX_XS[rl]*VOX_YS[rl]*4);
	assert(tmpimg);

	for(int yy=0; yy<VOX_YS[rl]; yy++)
	{
		for(int xx=0; xx<VOX_XS[rl]; xx++)
		{
			int r, g, b, a;
			if(max_z[yy*VOX_XS[rl]+xx] == INT32_MIN)
			{
				r = 255;
				g = 255;
				b = 255;
				a = 255;
			}
			else
			{
				int32_t out_z = max_z[yy*VOX_XS[rl]+xx];
				//out_z *= VOX_UNITS[rl]; // to mm
				//out_z -= (MAX_PAGES_Z/2)*MAP_PAGE_Z_H_MM; // to zero-referenced absolute mm

				//const int biggest_possible = VOX_ZS[0]*MAX_PAGES_Z;

				if(out_z < view2d_min_z)
				{
					r = 190;
					g = 190;
					b = 255;
					a = 255;
				}
				else if(out_z > view2d_max_z)
				{
					r = 255;
					g = 190;
					b = 190;
					a = 255;
				}
				else
				{

#define COLMAX 220
					int range = view2d_max_z - view2d_min_z;

//					r = (255*(out_z-view2d_min_z))/range;
					r = (2*COLMAX*(out_z-view2d_min_z))/(range)-COLMAX;
					if(r < 0) r = 0; else if(r > COLMAX) r = COLMAX;

//					b = (COLMAX*(view2d_max_z-out_z))/range;
					b = (2*COLMAX*(view2d_max_z-out_z))/(range)-COLMAX;
					if(b < 0) b = 0; else if(b > COLMAX) b = COLMAX;

					//g =  0;
					g =  COLMAX - r - b;
					if(g < 0) g = 0; else if(g > COLMAX) g = COLMAX;

					int ysq = 2000/(50+abs(r-g));
					r += ysq;
					g += ysq;

//					r = r-r/4 + 35;
//					g = g-g/4 + 35;
//					b = b-b/4 + 35;

					r += b/3;
					g += b/3;

					r += 25;
					g += 25;
					b += 25;
					if(r < 0) r = 0; else if(r > 255) r = 255;
					if(g < 0) g = 0; else if(g > 255) g = 255;

					a = 255;
				}
			}

			// In SFML coordinate system, y is mirrored - mirror in the texture.
			tmpimg[((VOX_YS[rl]-1-yy)*VOX_XS[rl]+xx)*4+0] = r;
			tmpimg[((VOX_YS[rl]-1-yy)*VOX_XS[rl]+xx)*4+1] = g;
			tmpimg[((VOX_YS[rl]-1-yy)*VOX_XS[rl]+xx)*4+2] = b;
			tmpimg[((VOX_YS[rl]-1-yy)*VOX_XS[rl]+xx)*4+3] = a;
		}
	}

	free(max_z);

	if(!piles[py*MAX_PAGES_X+px].textures[rl])
	{
		piles[py*MAX_PAGES_X+px].textures[rl] = new sf::Texture;
		piles[py*MAX_PAGES_X+px].textures[rl]->create(VOX_XS[rl], VOX_YS[rl]);
		piles[py*MAX_PAGES_X+px].textures[rl]->setSmooth(false);
	}

	piles[py*MAX_PAGES_X+px].textures[rl]->update(tmpimg);
	free(tmpimg);
}



void init_memdisk()
{
	map_set_1();
	assert(tex_full_map.create(MAX_PAGES_X, MAX_PAGES_Y));

	// Create a white base image, instead of random junk
	uint8_t* tmp = malloc(sizeof(uint8_t)*MAX_PAGES_X*MAX_PAGES_Y*4);
	memset(tmp, 255, sizeof(uint8_t)*MAX_PAGES_X*MAX_PAGES_Y*4);
	tex_full_map.update(tmp);
	free(tmp);
}


#if 0
void load_pages_from_disk(int px_start, int px_end, int py_start, int py_end, int pz_start, int pz_end, int rl)
{
	if(px_start < PX_MIN) px_start = PX_MIN;
	else if(px_start > PX_MAX) px_start = PX_MAX;

	if(py_start < PY_MIN) py_start = PY_MIN;
	else if(py_start > PY_MAX) py_start = PY_MAX;

	if(pz_start < PZ_MIN) pz_start = PZ_MIN;
	else if(pz_start > PZ_MAX) pz_start = PZ_MAX;

	if(px_end < PX_MIN) px_end = PX_MIN;
	else if(px_end > PX_MAX) px_end = PX_MAX;

	if(py_end < PY_MIN) py_end = PY_MIN;
	else if(py_end > PY_MAX) py_end = PY_MAX;

	if(pz_end < PZ_MIN) pz_end = PZ_MIN;
	else if(pz_end > PZ_MAX) pz_end = PZ_MAX;


	int n_x = px_end - px_start + 1;
	int n_y = py_end - py_start + 1;
	int n_z = pz_end - pz_start + 1;

}
#endif

extern double mm_per_pixel;
extern double origin_x;
extern double origin_y;

void draw_full_map(sf::RenderWindow& win)
{

	sf::Sprite sprite;

	float scale = (float)MAP_PAGE_XY_W_MM/mm_per_pixel;

	sprite.setOrigin(MAX_PAGES_X/2, MAX_PAGES_Y/2);
	sprite.setTexture(tex_full_map);
	sprite.setPosition((origin_x)/mm_per_pixel, (origin_y)/mm_per_pixel);
	sprite.setScale(sf::Vector2f(scale, scale));
	win.draw(sprite);

}

extern int screen_x;
extern int screen_y;

extern bool show_routing;
void draw_page_piles(sf::RenderWindow& win)
{
	int pix_start_x = -1.0*origin_x/mm_per_pixel;
	int pix_end_x = -1.0*origin_x/mm_per_pixel + screen_x;
	int pix_start_y = origin_y/mm_per_pixel - screen_y;
	int pix_end_y = origin_y/mm_per_pixel + 0;

	int mm_start_x = pix_start_x*mm_per_pixel;
	int mm_end_x = pix_end_x*mm_per_pixel;
	int mm_start_y = pix_start_y*mm_per_pixel;
	int mm_end_y = pix_end_y*mm_per_pixel;

	// po_coords: rl argument only affects the offsets, not used, so rl=0 is fine.
	po_coords_t poc_s = po_coords(mm_start_x, mm_start_y, 0, 0);
	po_coords_t poc_e = po_coords(mm_end_x, mm_end_y, 0, 0);

	for(int py=poc_s.py; py<=poc_e.py; py++)
//	for(int py=256-16; py<=256+16; py++)
	{
		for(int px=poc_s.px; px<=poc_e.px; px++)
//		for(int px=256-16; px<=256+16; px++)
		{
			if(py<0 || py >= MAX_PAGES_Y || px<0 || px > MAX_PAGES_X)
			{
				//printf("WARN: OOR px=%d, py=%d (draw_page_piles)\n", px, py);
				continue;
			}

			if(show_routing)
			{
				if(piles[py*MAX_PAGES_X+px].routing_texture)
				{
					sf::Sprite sprite;

					float scale = (float)MAP_PAGE_XY_W_MM/mm_per_pixel/(float)VOX_XS[ROUTING_RL];

					sprite.setOrigin(0, 0);
					sprite.setTexture(*piles[py*MAX_PAGES_X+px].routing_texture);
					sprite.setPosition(
						(origin_x+(px-MAX_PAGES_X/2)*MAP_PAGE_XY_W_MM)/mm_per_pixel - scale/2.0,
						(origin_y+(-1.0*((py-MAX_PAGES_Y/2+1)*MAP_PAGE_XY_W_MM)))/mm_per_pixel + scale/2.0);
					// +/- scale/2.0: align to the middle of the voxels.
					sprite.setScale(sf::Vector2f(scale, scale));
					sprite.setColor(sf::Color(255,255,255,128));
					win.draw(sprite);

				}
			}
			else
			{
				// Draw the highest resolution loaded:
				for(int rl=0; rl<4; rl++)
				{
					if(piles[py*MAX_PAGES_X+px].textures[rl])
					{
						sf::Sprite sprite;

						float scale = (float)MAP_PAGE_XY_W_MM/mm_per_pixel/(float)VOX_XS[rl];

						sprite.setOrigin(0, 0);
						sprite.setTexture(*piles[py*MAX_PAGES_X+px].textures[rl]);
						sprite.setPosition(
							(origin_x+(px-MAX_PAGES_X/2)*MAP_PAGE_XY_W_MM)/mm_per_pixel - scale/2.0,
							(origin_y+(-1.0*((py-MAX_PAGES_Y/2+1)*MAP_PAGE_XY_W_MM)))/mm_per_pixel + scale/2.0);
						// +/- scale/2.0: align to the middle of the voxels.
						sprite.setScale(sf::Vector2f(scale, scale));
						sprite.setColor(sf::Color(255,255,255,128));
						win.draw(sprite);

						break;
					}
				}
			 }
		}
	}
}

static inline void free_page_pile(int px, int py, int rl)
{
//	printf("Freeing page pile %d,%d,rl%d\n", px, py, rl);
	delete piles[py*MAX_PAGES_X+px].textures[rl];
	piles[py*MAX_PAGES_X+px].textures[rl] = NULL;
}

// Frees everything outside the range
static void load_page_pile_range_from_disk(int sx, int ex, int sy, int ey, int rl, int do_reload)
{
	if(sx < 0) sx = 0;
	if(ex > MAX_PAGES_X-1) ex = MAX_PAGES_X-1;

	if(sy < 0) sy = 0;
	if(ey > MAX_PAGES_Y-1) ey = MAX_PAGES_Y-1;

	for(int py=0; py<sy; py++)
	{
		for(int px=0; px<MAX_PAGES_X; px++)
		{
			if(piles[py*MAX_PAGES_X+px].textures[rl])
				free_page_pile(px, py, rl);
		}
	}

	for(int py=ey+1; py<MAX_PAGES_Y; py++)
	{
		for(int px=0; px<MAX_PAGES_X; px++)
		{
			if(piles[py*MAX_PAGES_X+px].textures[rl])
				free_page_pile(px, py, rl);
		}
	}

	for(int py=sy; py<=ey; py++)
	{
		for(int px=0; px<sx; px++)
		{
			if(piles[py*MAX_PAGES_X+px].textures[rl])
				free_page_pile(px, py, rl);
		}

		for(int px=ex+1; px<MAX_PAGES_X; px++)
		{
			if(piles[py*MAX_PAGES_X+px].textures[rl])
				free_page_pile(px, py, rl);
		}

	}

	for(int py=sy; py<=ey; py++)
	{
		for(int px=sx; px<=ex; px++)
		{
			load_page_pile_from_disk(px, py, rl, do_reload);
		}
	}
}

static void free_resolevel_completely(int rl)
{
	//printf("free_resolevel_completely(%d)\n", rl);
	for(int py=0; py < MAX_PAGES_Y; py++)
	{
		for(int px=0; px < MAX_PAGES_X; px++)
		{
			if(piles[py*MAX_PAGES_X+px].textures[rl])
				free_page_pile(px, py, rl);
		}
	}
}


int manage_page_pile_ranges()
{
	int pix_start_x = -1.0*origin_x/mm_per_pixel;
	int pix_end_x = -1.0*origin_x/mm_per_pixel + screen_x;
	int pix_start_y = origin_y/mm_per_pixel - screen_y;
	int pix_end_y = origin_y/mm_per_pixel + 0;

	int mm_start_x = pix_start_x*mm_per_pixel;
	int mm_end_x = pix_end_x*mm_per_pixel;
	int mm_start_y = pix_start_y*mm_per_pixel;
	int mm_end_y = pix_end_y*mm_per_pixel;

	po_coords_t poc_s = po_coords(mm_start_x, mm_start_y, 0, 0);
	po_coords_t poc_e = po_coords(mm_end_x, mm_end_y, 0, 0);


	int px_start = poc_s.px;
	int px_end = poc_e.px;
	int py_start = poc_s.py;
	int py_end = poc_e.py;

	// Use a hysteresis band: don't free pages as easily as we load them.
	static const int n_load_allowed[MAX_RESOLEVELS]  = {4*4, 16*16, 64*64, 128*128};
	static const int n_free_required[MAX_RESOLEVELS] = {7*7, 20*20, 80*80, 160*160};
//	static const int n_load_allowed[MAX_RESOLEVELS]  = {100*100, 100*100, 100*100, 128*128};
//	static const int n_free_required[MAX_RESOLEVELS] = {80*80, 80*80, 80*80, 160*160};

	#define NEAR_EXTRA 1
	#define FAR_EXTRA 4

	int n_near = ((px_end+NEAR_EXTRA)-(px_start-NEAR_EXTRA))*((py_end+NEAR_EXTRA)-(py_start-NEAR_EXTRA));
	int n_far = ((px_end+FAR_EXTRA)-(px_start-FAR_EXTRA))*((py_end+FAR_EXTRA)-(py_start-FAR_EXTRA));

	// Load pages on up to 2 resolevels
	// Free all other resolevels completely

	int completely_free[MAX_RESOLEVELS] = {1, 1, 1, 1};

	//printf("n_near=%d, n_far=%d\n", n_near, n_far);

	int n_rls_loaded = 0;
	for(int rl=0; rl<MAX_RESOLEVELS; rl++)
	{
		if(n_far <= n_load_allowed[rl])
		{
			load_page_pile_range_from_disk(px_start-FAR_EXTRA, px_end+FAR_EXTRA, py_start-FAR_EXTRA, py_end+FAR_EXTRA, rl, 0);
			completely_free[rl] = 0;
			n_rls_loaded++;
		}
		else if(n_near <= n_load_allowed[rl])
		{
			load_page_pile_range_from_disk(px_start-NEAR_EXTRA, px_end+NEAR_EXTRA, py_start-NEAR_EXTRA, py_end+NEAR_EXTRA, rl, 0);
			completely_free[rl] = 0;
			n_rls_loaded++;
		}

		if(n_rls_loaded >= 2)
			break;
	}

	int use_fullmap = 0;
	for(int rl=0; rl<4; rl++)
	{
		if(n_near > n_free_required[rl] || completely_free[rl])
		{
			free_resolevel_completely(rl);
			use_fullmap++;
		}
	}


	int n_loaded[4] = {0,0,0,0};
	for(int py=0; py<MAX_PAGES_Y; py++)
	{
		for(int px=0; px<MAX_PAGES_X; px++)
		{
			for(int rl=0; rl<4; rl++)
			{
				if(piles[py*MAX_PAGES_X+px].textures[rl])
					n_loaded[rl]++;
			}
		}
	}
	//printf("manage: n_loaded per rl: %4d  %4d  %4d  %4d\n", n_loaded[0],n_loaded[1],n_loaded[2],n_loaded[3]);

	return use_fullmap>=4;
}

void map_set_1()
{
	strncpy(mapdir, "../robotsoft/current_maps", 1023);
}
void map_set_2()
{
	strncpy(mapdir, "../robotsoft/current_maps_2", 1023);
}


void build_fullmap()
{
	memset(pages_exist, 0, MAX_PAGES_X*MAX_PAGES_Y);
	DIR *d;
	struct dirent *dir;
	d = opendir(mapdir);

	uint8_t* highest_pz = calloc(MAX_PAGES_X*MAX_PAGES_Y, sizeof(uint8_t));
	assert(highest_pz);

	if(d)
	{
		while( (dir = readdir(d)) != 0)
		{
			unsigned int px, py, pz, rl;

			if(sscanf(dir->d_name, "voxmap_x%u_y%u_z%u_r%u", &px, &py, &pz, &rl) == 4)
			{
				assert(px >= 0 && px < MAX_PAGES_X && py >= 0 && py < MAX_PAGES_Y && pz >=0 && pz < MAX_PAGES_Z && rl >= 0 && rl < MAX_RESOLEVELS);
				pages_exist[py*MAX_PAGES_X+px] = 1;
				if(pz+1 > highest_pz[py*MAX_PAGES_X+px])
					highest_pz[py*MAX_PAGES_X+px] = pz+1;

			//	load_page_pile_from_disk(px, py, rl, 0);
			}

		}

		closedir(d);
	}


	uint8_t* tmp = malloc(sizeof(uint8_t)*MAX_PAGES_X*MAX_PAGES_Y*4);
	assert(tmp);
	memset(tmp, 255, sizeof(uint8_t)*MAX_PAGES_X*MAX_PAGES_Y*4);

	for(int yy=0; yy<MAX_PAGES_Y; yy++)
	{
		for(int xx=0; xx<MAX_PAGES_X; xx++)
		{
			if(highest_pz[yy*MAX_PAGES_X+xx] > 0)
			{
				int r, g, b;
	/*
				r = -1*(MAX_PAGES_Z-2*highest_pz[i])*(256/MAX_PAGES_Z);
				if(r < 0) r=0; else if(r>255) r=255;

				b = +1*(MAX_PAGES_Z-2*highest_pz[i])*(256/MAX_PAGES_Z);
				if(b < 0) b=0; else if(b>255) b=255;
	*/
				r = (255*(highest_pz[yy*MAX_PAGES_X+xx]-0))/MAX_PAGES_Z;
				if(r < 0) r = 0; else if(r > 255) r = 255;

				b = (255*(MAX_PAGES_Z-highest_pz[yy*MAX_PAGES_X+xx]))/MAX_PAGES_Z;
				if(b < 0) b = 0; else if(b > 255) b = 255;

				g = 0; //256 - r - b;
				if(g < 0) g=0; else if(g>255) g=255;

				tmp[((MAX_PAGES_Y-1-yy)*MAX_PAGES_X+xx)*4+0] = r;
				tmp[((MAX_PAGES_Y-1-yy)*MAX_PAGES_X+xx)*4+1] = g;
				tmp[((MAX_PAGES_Y-1-yy)*MAX_PAGES_X+xx)*4+2] = b;
				tmp[((MAX_PAGES_Y-1-yy)*MAX_PAGES_X+xx)*4+3] = 255;
			}	
		}
	}

	free(highest_pz);

	tex_full_map.update(tmp);
	free(tmp);
}

void reload_map()
{
	build_fullmap();
	for(int rl=0; rl<4; rl++)
		free_resolevel_completely(rl);

	manage_page_pile_ranges();
}

void build_model();

sf::Shader shader;


static glm::mat4 projection;
static glm::mat4 view;
static int pvm_loc;


void resize_opengl(sf::RenderWindow& win)
{
	glViewport(0, 0, win.getSize().x, win.getSize().y);

	float ratio = static_cast<float>(win.getSize().x) / win.getSize().y;

	projection = glm::perspective(glm::radians(60.0f), ratio, 0.1f, 10000.0f);
}

// pieces per map page for each resolevel
static int pieces_x[MAX_RESOLEVELS];
static int pieces_y[MAX_RESOLEVELS];
static int pieces_z[MAX_RESOLEVELS];

#define MAX_MESHES 4096

typedef struct __attribute__((packed))
{
	GLfloat x;
	GLfloat y;
	GLfloat z;
} vec3_t;

#define GL_UINT_2_10_10_10_REV(a_, b_, g_, r_) (((a_)<<30) | ((b_)<<20) | ((g_)<<10) | (r_))

typedef struct __attribute__((packed))
{
//	vec3_t position;
	uint32_t position;
//	vec3_t normal;
	uint32_t color;
} vertex_attrs_t;


typedef struct __attribute__((packed))
{
	vec3_t position;
	uint32_t color;
} point_attrs_t;


typedef struct
{
	uint32_t vao;
	uint32_t vbo;
	uint32_t n_triangles;
} mesh_t;


/*
	3D world coordinates are half blocks of the resolevel0, e.g. 16mm if rl0 resolution is 32mm.
	This is to optimize the memory usage for meshes: vertex coordinates can use smallest possible
	integer type (for example, cube at 0,0,0mm has vertices at -16mm, 16mm, etc. - this is 1 unit).

	With this, giving 10 bits per coordinate component, with 32mm rl0 (16mm 3D world resolution), maximum
	mesh size is 16384*16384*16384 mm.

	3d piece uses all positive coordinates, and is referenced to its corner.

	The 3D world coordinates are referenced to zero, by simply dividing the absolute mm by rl0_size/2 (16 for example)
*/

typedef struct
{
	// Start coordinates in 3D World coordinates.
	int x_start;
	int y_start;
	int z_start;
	// Size in 3D World blocks - end coordinates are start+size
	// Use xs = 0 to denote empty piece
	int xs;
	int ys;
	int zs;

	// Referencing to the original page
	int px;
	int py;
	int pz;
	int rl;

	mesh_t mesh;
} piece_3d_t;

piece_3d_t pieces_3d[MAX_MESHES];

void free_mesh(mesh_t* mesh)
{
	assert(mesh->vao > 0);
	assert(mesh->vbo > 0);
//	assert(mesh->n_triangles > 0);

	mutex_gl.lock();
	p_win->setActive(true);

	glDeleteBuffers(1, &(mesh->vbo));
	glDeleteBuffers(1, &(mesh->vao));
	p_win->setActive(false);
	mutex_gl.unlock();

	mesh->vao = 0;
	mesh->vbo = 0;
	mesh->n_triangles = 0;
}


static void free_piece(int idx)
{
	assert(pieces_3d[idx].xs > 0);
	free_mesh(&pieces_3d[idx].mesh);
	pieces_3d[idx].px = -1;
	pieces_3d[idx].py = -1;
	pieces_3d[idx].pz = -1;
	pieces_3d[idx].rl = -1;
	pieces_3d[idx].xs = 0;
}


void init_opengl(sf::RenderWindow& win)
{
	p_win = &win;

	// Prebuild "pieces per map page" arrays:
	for(int rl=0; rl < MAX_RESOLEVELS; rl++)
	{
		pieces_x[rl] = VOX_XS[rl]/64;
		if(pieces_x[rl] < 1) pieces_x[rl] = 1;
		pieces_y[rl] = VOX_YS[rl]/64;
		if(pieces_y[rl] < 1) pieces_y[rl] = 1;
		pieces_z[rl] = VOX_ZS[rl]/64;
		if(pieces_z[rl] < 1) pieces_z[rl] = 1;

	}

	// Init "illegal" page coordinate references to speed up checking
	for(int idx=0; idx<MAX_MESHES; idx++)
	{
		pieces_3d[idx].xs = 0;
		pieces_3d[idx].px = -1;
		pieces_3d[idx].py = -1;
		pieces_3d[idx].pz = -1;
		pieces_3d[idx].rl = -1;
	}



	glewInit();


	if(shader.loadFromFile("vertex_shader.glsl", "fragment_shader.glsl") == false)
	{
		printf("Failed to load shaders. Please reinstall the files.\n");
		abort();
	}


	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	glClearDepth(1.0);

	glEnable(GL_PROGRAM_POINT_SIZE);
	glPointSize(4.0);

	sf::Shader::bind(&shader);
	int shader_id = shader.getNativeHandle();
	pvm_loc = glGetUniformLocation(shader_id, "pvm");
	printf("shader_id = %d, pvm_loc = %d\n", shader_id, pvm_loc);
	sf::Shader::bind(0);

	resize_opengl(win);
}




// 31.5Mbytes
#define MAX_CUBES_PER_MESH (32*32*32)
#define MAX_TRIANGLES_PER_MESH (MAX_CUBES_PER_MESH*12)
#define MAX_VERTICES_PER_MESH (MAX_TRIANGLES_PER_MESH*3)



extern bool show_free;

#define TEST_VOXMAP(x_, y_, z_, rl_) (show_free? \
	(((p_voxmap->voxels[(y_)*VOX_XS[rl_]*VOX_ZS[rl_]+(x_)*VOX_ZS[rl_]+(z_)] & 0xf0)>>4) >= display_limit[rl_]) : \
	((p_voxmap->voxels[(y_)*VOX_XS[rl_]*VOX_ZS[rl_]+(x_)*VOX_ZS[rl_]+(z_)] & 0x0f) >= display_limit[rl_]))


sf::Mutex mutex_gl;


mesh_t voxmap_to_mesh(voxmap_t* p_voxmap, int x_start, int x_end, int y_start, int y_end, int z_start, int z_end, int pz, int rl)
{

//	printf("voxmap_to_mesh x %d..%d  y %d..%d  z %d..%d,  rl%d\n", x_start, x_end, y_start, y_end, z_start, z_end, rl);
	static vertex_attrs_t attrs[MAX_VERTICES_PER_MESH];

	int facehalf=VOX_RELATIONS[rl];

	int vertices[] = {
	facehalf, facehalf, facehalf,
	facehalf, -facehalf, facehalf,
	-facehalf, -facehalf, facehalf,

	-facehalf, facehalf, facehalf,
	facehalf, facehalf, facehalf,
	-facehalf, -facehalf, facehalf,



	facehalf, facehalf, -facehalf,
	-facehalf, facehalf, -facehalf,
	-facehalf, -facehalf, -facehalf,

	facehalf, -facehalf, -facehalf,
	facehalf, facehalf, -facehalf,
	-facehalf, -facehalf, -facehalf,



	facehalf, facehalf, facehalf,
	-facehalf, facehalf, facehalf,
	-facehalf, facehalf, -facehalf,

	facehalf, facehalf, -facehalf,
	facehalf, facehalf, facehalf,
	-facehalf, facehalf, -facehalf,



	facehalf, -facehalf, facehalf,
	facehalf, -facehalf, -facehalf,
	-facehalf, -facehalf, -facehalf,

	-facehalf, -facehalf, facehalf,
	facehalf, -facehalf, facehalf,
	-facehalf, -facehalf, -facehalf,



	facehalf, facehalf, facehalf,
	facehalf, facehalf, -facehalf,
	facehalf, -facehalf, -facehalf,

	facehalf, -facehalf, facehalf,
	facehalf, facehalf, facehalf,
	facehalf, -facehalf, -facehalf,



	-facehalf, facehalf, facehalf,
	-facehalf, -facehalf, facehalf,
	-facehalf, -facehalf, -facehalf,

	-facehalf, facehalf, -facehalf,
	-facehalf, facehalf, facehalf,
	-facehalf, -facehalf, -facehalf
	};


	int v = 0;
	for(int yy=y_start; yy<y_end; yy++)
	{
		for(int xx=x_start; xx<x_end; xx++)
		{
			for(int zz=z_start; zz<z_end; zz++)
			{
				if(TEST_VOXMAP(xx, yy, zz, rl))
				{
					int ignore_faces[6] = {0};

					if(xx > 0 && TEST_VOXMAP(xx-1, yy, zz, rl))
						ignore_faces[5] = 1;

					if(xx < VOX_XS[rl]-1 && TEST_VOXMAP(xx+1, yy, zz, rl))
						ignore_faces[4] = 1;

					if(yy > 0 && TEST_VOXMAP(xx, yy-1, zz, rl))
						ignore_faces[1] = 1;

					if(yy < VOX_YS[rl]-1 && TEST_VOXMAP(xx, yy+1, zz, rl))
						ignore_faces[0] = 1;

					if(zz > 0 && TEST_VOXMAP(xx, yy, zz-1, rl))
						ignore_faces[3] = 1;

					if(zz < VOX_ZS[rl]-1 && TEST_VOXMAP(xx, yy, zz+1, rl))
						ignore_faces[2] = 1;

					int r, g, b, a;

					int range = view2d_max_z - view2d_min_z;


					int out_z = pz*VOX_ZS[rl]+zz;
					out_z *= VOX_UNITS[rl];
					out_z -= (MAX_PAGES_Z/2)*MAP_PAGE_Z_H_MM;

					r = (2*COLMAX*(out_z-view2d_min_z))/(range)-COLMAX;
					if(r < 0) r = 0; else if(r > COLMAX) r = COLMAX;

					b = (2*COLMAX*(view2d_max_z-out_z))/(range)-COLMAX;
					if(b < 0) b = 0; else if(b > COLMAX) b = COLMAX;

					//g =  0;
					g =  COLMAX - r - b;
					if(g < 0) g = 0; else if(g > COLMAX) g = COLMAX;

					int ysq = 2000/(50+abs(r-g));
					r += ysq;
					g += ysq;

					r += b/3;
					g += b/3;

					r += 25;
					g += 25;
					b += 25;
					if(r < 0) r = 0; else if(r > 255) r = 255;
					if(g < 0) g = 0; else if(g > 255) g = 255;

					a = 255;

//					float x = (xx-x_start)*VOX_RELATIONS[rl]*2.0;
//					float y = (zz-z_start)*VOX_RELATIONS[rl]*2.0;
//					float z = (-1*(yy-y_start))*VOX_RELATIONS[rl]*2.0;

					int x = facehalf + (xx-x_start)*VOX_RELATIONS[rl]*2;
					int y = facehalf + (zz-z_start)*VOX_RELATIONS[rl]*2;
					int z = facehalf + (yy-y_start)*VOX_RELATIONS[rl]*2;
//					int z = -1*facehalf + (-1*(yy-y_start)+(y_end-y_start))*VOX_RELATIONS[rl]*2;

					assert(x >= facehalf && x < 1024-facehalf);
					assert(y >= facehalf && y < 1024-facehalf);
					assert(z >= facehalf && z < 1024-facehalf);
					//x = 0.0;
					//y = 0.0;
					//z = 0.0;

					// No need to check this limit in the inmost loop. Let's ignore full cubes, if we need to.
					if(v >= MAX_VERTICES_PER_MESH-36)
					{
						printf("WARNING: Mesh vertex limit exceeded - dropping some voxels\n");
						goto SKIP_VOXELS;
					}

					for(int face=0; face<6; face++)
					{
						if(ignore_faces[face])
							continue;


						for(int i=face*6; i<(face+1)*6; i++)
						{
//							attrs[v].position.x = vertices[i*3+0] + x;
//							attrs[v].position.y = vertices[i*3+1] + y;
//							attrs[v].position.z = vertices[i*3+2] + z;

							int vx = vertices[i*3+0] + x;
							int vy = vertices[i*3+1] + y;
							int vz = vertices[i*3+2] + z;

							assert(vx >= 0 && vx < 1024);
							assert(vy >= 0 && vy < 1024);
							assert(vz >= 0 && vz < 1024);

							if(rl==0)
							{
								assert(vx >= 0 && vx <= 128);
								assert(vy >= 0 && vy <= 128);
								assert(vz >= 0 && vz <= 128);
							}
							else if(rl==1)
							{
								assert(vx >= 0 && vx <= 256);
								assert(vy >= 0 && vy <= 256);
								assert(vz >= 0 && vz <= 256);
							}
							else if(rl==2)
							{
								assert(vx >= 0 && vx <= 512);
								assert(vy >= 0 && vy <= 512);
								assert(vz >= 0 && vz <= 512);
							}
							else //if(rl==3)
							{
								assert(vx >= 0 && vx <= 512);
								assert(vy >= 0 && vy <= 512);
								assert(vz >= 0 && vz <= 512);
							}

							attrs[v].position = GL_UINT_2_10_10_10_REV(0,
								vz,
								vy,
								vx);

							//attrs[v].normal.x = normals[i*3+0];
							//attrs[v].normal.y = normals[i*3+1];
							//attrs[v].normal.z = normals[i*3+2];
							if(face == 2) // top face: floor
								attrs[v].color = RGBA32(r, g, b, a);
							else if(face == 3) // bottom face: ceiling
								attrs[v].color = RGBA32(r/2, g/2, b/2, a);
							else if(face == 0 || face == 1)
								attrs[v].color = RGBA32(11*r/16, 11*g/16, 11*b/16, a);
							else
								attrs[v].color = RGBA32(13*r/16, 13*g/16, 13*b/16, a);
							v++;
						}

					}

				}
				
			}
		}
	}

	SKIP_VOXELS:;


	int32_t mem = (v*sizeof(vertex_attrs_t));
//	printf("Mesh generation done. vertex count=%d Total memory: %d bytes\n", v, mem);


	mesh_t ret;

	ret.n_triangles = v/3;

	mutex_gl.lock();
	p_win->setActive(true);


	glGenVertexArrays(1, &ret.vao);
	glGenBuffers(1, &ret.vbo);

	glBindVertexArray(ret.vao);

	glBindBuffer(GL_ARRAY_BUFFER, ret.vbo);

	glBufferData(GL_ARRAY_BUFFER, v*sizeof(vertex_attrs_t), attrs, GL_STATIC_DRAW);  

	// vertex positions, layout 0
	glEnableVertexAttribArray(0);	
	glVertexAttribPointer(0, 4, GL_UNSIGNED_INT_2_10_10_10_REV, GL_FALSE, sizeof(vertex_attrs_t), (void*)offsetof(vertex_attrs_t, position));

	// colors, layout 1
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(vertex_attrs_t), (void*)offsetof(vertex_attrs_t, color));

	glBindVertexArray(0);

	p_win->setActive(false);

	mutex_gl.unlock();

	return ret;
}







#include <glm/gtx/string_cast.hpp>

static void render_piece(piece_3d_t* p)
{
	if(p->mesh.n_triangles < 1)
		return;


	// No need to rotate the models, only translate

	glm::mat4 model = glm::mat4(1.0f);
	model = glm::translate(model, glm::vec3(p->x_start, p->z_start, p->y_start));
	glm::mat4 pvm = projection * view * model;


//	printf("point_transformed: %s\n", glm::to_string(point_transformed).c_str());
	int n_outside = 0;
	float edges[8*3] =
	{
		p->x_start, p->y_start, p->z_start,
		p->x_start, p->y_start, p->z_start+p->zs,
		p->x_start, p->y_start+p->ys, p->z_start,
		p->x_start, p->y_start+p->ys, p->z_start+p->zs,
		p->x_start+p->xs, p->y_start, p->z_start,
		p->x_start+p->xs, p->y_start, p->z_start+p->zs,
		p->x_start+p->xs, p->y_start+p->ys, p->z_start,
		p->x_start+p->xs, p->y_start+p->ys, p->z_start+p->zs
	};

	for(int i = 0; i < 8; i++)
	{
		glm::vec4 point = glm::vec4(edges[i*3+0], edges[i*3+2], edges[i*3+1], 1.0f);

		glm::vec4 point_transformed = (projection * view) * point;

		if(point_transformed.x < -1.2*point_transformed.w || point_transformed.x > 1.2*point_transformed.w ||
		   point_transformed.y < -1.5*point_transformed.w || point_transformed.y > 1.5*point_transformed.w ||
		   point_transformed.z < -1.2*point_transformed.w || point_transformed.z > 1.2*point_transformed.w)
			n_outside++;

	}

	if(n_outside == 8)
		return;

	glBindVertexArray(p->mesh.vao);


	// Pieces are large, and there are few of them (several hundreds).
	// Don't calculate projection*view*model on GPU for every vertex.

	glUniformMatrix4fv(pvm_loc, 1, GL_FALSE, glm::value_ptr(pvm));
	glDrawArrays(GL_TRIANGLES, 0, p->mesh.n_triangles*3);
}

#include "../robotsoft/small_cloud.h"
//#include "../robotsoft/slam_cloud.h" // for properly dimensioned realtime_cloud_t
#include <zlib.h>

// 3D point-cloud type: actual data lives in the GPU memory. Only store necessary metadata here. vao and vbo reference to the GPU memory.
typedef struct
{
	// Reference coordinates in 3D World coordinates.
	int ref_x;
	int ref_y;
	int ref_z;

	uint32_t vao;
	uint32_t vbo;
	uint32_t n_points;
} pc_t;

int realtime_pc_enabled = 0;
static pc_t realtime_pc;

void free_pc(pc_t* pc)
{
	if(pc->vao > 0)
	{
		mutex_gl.lock();
		p_win->setActive(true);

		glDeleteBuffers(1, &pc->vbo);
		glDeleteVertexArrays(1, &pc->vao);


		p_win->setActive(false);

		mutex_gl.unlock();

	}
}

pc_t generate_pc(small_cloud_t* cloud, int n_points, int ref_x, int ref_y, int ref_z)
{
	pc_t ret = {0};

	ret.ref_x = ref_x / (VOX_UNITS[0]/2);
	ret.ref_y = ref_y / (VOX_UNITS[0]/2);
	ret.ref_z = ref_z / (VOX_UNITS[0]/2);

	point_attrs_t* attrs = malloc(n_points * sizeof(point_attrs_t));

	for(int pi=0; pi<n_points; pi++)
	{
		tmp_point_t p = get_small_cloud_point(cloud[pi]);

		int r, g, b, a;
		int range = view2d_max_z - view2d_min_z;
		int out_z = p.z;
		r = (2*COLMAX*(out_z-view2d_min_z))/(range)-COLMAX;
		if(r < 0) r = 0; else if(r > COLMAX) r = COLMAX;
		b = (2*COLMAX*(view2d_max_z-out_z))/(range)-COLMAX;
		if(b < 0) b = 0; else if(b > COLMAX) b = COLMAX;
		g =  COLMAX - r - b;
		if(g < 0) g = 0; else if(g > COLMAX) g = COLMAX;
		int ysq = 2000/(50+abs(r-g));
		r += ysq;
		g += ysq;
		r += b/3;
		g += b/3;
		r += 25;
		g += 25;
		b += 25;
		if(r < 0) r = 0; else if(r > 255) r = 255;
		if(g < 0) g = 0; else if(g > 255) g = 255;
		a = 255;

		// if flag is set, color is desaturated so that flagged data can be visually seen as different.
		if(GET_SMALL_CLOUD_FLAG(cloud[pi]))
		{
			int rt = r, gt = g, bt = b;
			r = (2*rt + gt + bt)/4;
			g = (2*gt + rt + bt)/4;
			b = (2*bt + rt + gt)/4;
		}

		// Convert millimeters to 3D world coordinate units:
		attrs[pi].position.x = p.x / (VOX_UNITS[0]/2);
		attrs[pi].position.y = p.z / (VOX_UNITS[0]/2);
		attrs[pi].position.z = p.y / (VOX_UNITS[0]/2);
		attrs[pi].color = RGBA32(r, g, b, a);
	}

	int32_t mem = n_points * sizeof(point_attrs_t);
	printf("pc generation done. point count=%d Total memory: %d bytes\n", n_points, mem);

	ret.n_points = n_points;

	mutex_gl.lock();
	p_win->setActive(true);

	glGenVertexArrays(1, &ret.vao);
	glGenBuffers(1, &ret.vbo);

	glBindVertexArray(ret.vao);

	glBindBuffer(GL_ARRAY_BUFFER, ret.vbo);

	glBufferData(GL_ARRAY_BUFFER, n_points*sizeof(point_attrs_t), attrs, GL_STATIC_DRAW);  

	// vertex positions, layout 0
	glEnableVertexAttribArray(0);	
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(point_attrs_t), (void*)offsetof(point_attrs_t, position));

	// colors, layout 1
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(point_attrs_t), (void*)offsetof(point_attrs_t, color));

	glBindVertexArray(0);

	p_win->setActive(false);

	free(attrs);

	mutex_gl.unlock();

	return ret;
}



// 2D version of the incoming 3D pointcloud:
#define PC2D_SIZE 1024
#define PC2D_LOWER_CUTOFF -500
#define PC2D_UPPER_CUTOFF 2000
typedef struct
{
	int ref_x;
	int ref_y;
	int ref_z;
	sf::Texture* texture;
} pc_2d_t;

static pc_2d_t realtime_pc_2d;

int generate_pc_2d(pc_2d_t* pc_2d, small_cloud_t* cloud, int n_points, int ref_x, int ref_y, int ref_z)
{
	pc_2d->ref_x = ref_x;
	pc_2d->ref_y = ref_y;
	pc_2d->ref_z = ref_z;

	if(pc_2d->texture == NULL)
	{
		// Texture init needed
		pc_2d->texture = new sf::Texture;
		pc_2d->texture->create(PC2D_SIZE, PC2D_SIZE);
		pc_2d->texture->setSmooth(false);
	}

	// calloc gives zeroes, so alpha is zero as well (transparent)
	uint32_t* pixels = calloc(PC2D_SIZE*PC2D_SIZE, sizeof(uint32_t));

	uint8_t* zmap = calloc(PC2D_SIZE*PC2D_SIZE, sizeof(int8_t)); // Highest reading relative to ref_z, but not exceeding magical ceiling cutoff. Resolution 32mm. Offset of min_z.

	int i = 0;
	for(int pi=0; pi<n_points; pi++)
	{
		tmp_point_t p = get_small_cloud_point(cloud[pi]);

		p.y *= -1; // screen axis swapped

		if(p.z > view2d_max_z || p.z < view2d_min_z)
			continue;

		// Convert millimeters to second highest resolevel VOX units:
		int x = p.x / (VOX_UNITS[1]) + PC2D_SIZE/2;
		int y = p.y / (VOX_UNITS[1]) + PC2D_SIZE/2;

		if(x < 0 || y < 0 || x >= PC2D_SIZE || y >= PC2D_SIZE)
			continue;

		uint8_t cmp_z = (p.z-view2d_min_z)/32;
		if(cmp_z > zmap[y*PC2D_SIZE+x])
		{
			zmap[y*PC2D_SIZE+x] = cmp_z;

			int r, g, b, a;
			int range = view2d_max_z - view2d_min_z;
			int out_z = p.z;
			r = (2*COLMAX*(out_z-view2d_min_z))/(range)-COLMAX;
			if(r < 0) r = 0; else if(r > COLMAX) r = COLMAX;
			b = (2*COLMAX*(view2d_max_z-out_z))/(range)-COLMAX;
			if(b < 0) b = 0; else if(b > COLMAX) b = COLMAX;
			g =  COLMAX - r - b;
			if(g < 0) g = 0; else if(g > COLMAX) g = COLMAX;
			int ysq = 2000/(50+abs(r-g));
			r += ysq;
			g += ysq;
			r += b/3;
			g += b/3;
			r += 25;
			g += 25;
			b += 25;
			if(r < 0) r = 0; else if(r > 255) r = 255;
			if(g < 0) g = 0; else if(g > 255) g = 255;
			a = 255;

			uint32_t color = RGBA32(r, g, b, a);
			pixels[y*PC2D_SIZE+x] = color;
			i++;
		}
	}

//	printf("Generated %d pixels\n", i);

	pc_2d->texture->update((uint8_t*)pixels);
	free(pixels);
	free(zmap);
	return 0;
}

void draw_pc_2d(sf::RenderWindow& win, pc_2d_t* pc_2d)
{
	if(!pc_2d || !pc_2d->texture)
		return;

	sf::Sprite sprite;

	float scale = (float)(VOX_UNITS[1])/mm_per_pixel;
	sprite.setOrigin(PC2D_SIZE/2, PC2D_SIZE/2);
	sprite.setTexture(*pc_2d->texture);
	sprite.setPosition((origin_x+pc_2d->ref_x)/mm_per_pixel, (origin_y-pc_2d->ref_y)/mm_per_pixel);
//	sprite.setPosition(500, 500);
	sprite.setScale(sf::Vector2f(scale, scale));
	win.draw(sprite);

//	printf("drawing sprite\n");
}

void draw_realtime_pc_2d(sf::RenderWindow& win)
{
	draw_pc_2d(win, &realtime_pc_2d);
}

static void render_pc(pc_t* p)
{
	if(p->n_points < 1)
		return;

	// No need to rotate the models, only translate

	glm::mat4 model = glm::mat4(1.0f);
	model = glm::translate(model, glm::vec3(p->ref_x, p->ref_z, p->ref_y));
	glm::mat4 pvm = projection * view * model;

	glBindVertexArray(p->vao);

	glUniformMatrix4fv(pvm_loc, 1, GL_FALSE, glm::value_ptr(pvm));
	glDrawArrays(GL_POINTS, 0, p->n_points);
}

// Decompresses zlibbed pointcloud data in buf, generates either 2d or 3d graphics depending on the view mode.
void process_realtime_pointcloud(uint8_t* buf, int buflen, bool view_3d)
{
	small_cloud_header_t* p_head = (small_cloud_header_t*)buf;
	uint8_t* p_data = buf + sizeof(small_cloud_header_t);

	assert(p_head->magic == 0xaa14);
	assert(p_head->compression == 1);
//	assert(p_head->n_points <= MAX_REALTIME_N_POINTS);

	int n_points = p_head->n_points;

	assert(n_points < 50000000);

	small_cloud_t *cloud = malloc(sizeof(small_cloud_t) * n_points);

	assert(cloud);

	z_stream strm;
	strm.zalloc = Z_NULL;
	strm.zfree = Z_NULL;
	strm.opaque = Z_NULL;
	strm.avail_in = buflen - sizeof(small_cloud_header_t);
	strm.next_in = p_data;

	if(inflateInit(&strm) != Z_OK)
	{
		printf("ERROR: ZLIB initialization failed\n");
		abort();
	}

	int ret = 0;

	strm.avail_out = sizeof(small_cloud_t) * n_points;
	strm.next_out = (uint8_t*)cloud;

	ret = inflate(&strm, Z_FINISH);
	assert(ret != Z_STREAM_ERROR);

	switch(ret)
	{
		case Z_NEED_DICT:
		case Z_DATA_ERROR:
		case Z_MEM_ERROR:
		{
			printf("ERROR: realtime point cloud decompression error, inflate() returned %d\n", ret);
			abort();
		}
		default: break;
	}

	assert(strm.avail_in == 0);

	inflateEnd(&strm);


	if(view_3d)
	{
		free_pc(&realtime_pc);
		realtime_pc = generate_pc(cloud, n_points, p_head->ref_x_mm, p_head->ref_y_mm, p_head->ref_z_mm);
		realtime_pc_enabled = 1;
	}
	else
	{
		generate_pc_2d(&realtime_pc_2d, cloud, n_points, p_head->ref_x_mm, p_head->ref_y_mm, p_head->ref_z_mm);
	}

	free(cloud);
}

void process_file_pointcloud(const char* fname, bool view_3d)
{
	int fd = open(fname, O_RDONLY);
	
	if(fd < 0)
	{
		printf("Error %d opening file %s: %s\n", errno, fname, strerror(errno));
		return;
	}

	struct stat s;
	int ret = fstat(fd, &s);
	if(ret < 0)
	{
		printf("Error %d fstat() file %s: %s\n", errno, fname, strerror(errno));
		return;
	}

	uint8_t* buf = mmap(NULL, s.st_size, PROT_READ, MAP_SHARED, fd, 0);

	if(buf == MAP_FAILED)
	{
		printf("Error %d memory-mapping file %s: %s\n", errno, fname, strerror(errno));
		return;
	}

	process_realtime_pointcloud(buf, s.st_size, view_3d);

	munmap(buf, s.st_size);
	close(fd);
}

void process_tcp_voxmap(uint8_t* buf, int buflen)
{
	voxmap_header_t* p_head = (voxmap_header_t*)buf;
	uint8_t* p_data = buf + sizeof(voxmap_header_t);

	assert(p_head->magic == 0xaa13);
	if(p_head->api_version != 0x0420)
	{
		printf("WARNING: tcp voxmap has api version 0x%04x, expecting 0x0420. Data may be incorrect; trying to use it. Make sure you use same software versions on both ends.\n", 
			p_head->api_version);
	}
	assert(p_head->compression == 1);

	// Find the resolevel:
	int rl;
	for(rl=0; rl<MAX_RESOLEVELS; rl++)
	{
		if(p_head->xs == VOX_XS[rl] && p_head->ys == VOX_YS[rl] && p_head->zs == VOX_ZS[rl])
			break;
	}

	if(rl==MAX_RESOLEVELS)
	{
		printf("WARNING: tcp voxmap has incompatible dimensions (%d x %d x %d), not matching the voxmap memdisk system possibilities. Ignoring the voxmap. Make sure you use same software versions on both ends.\n",
			p_head->xs, p_head->ys, p_head->zs);
		return;
	}

	po_coords_t po = po_coords(p_head->ref_x_mm, p_head->ref_y_mm, p_head->ref_z_mm, rl);


	static char fnamebuf[2048];
	FILE* f = fopen(gen_fname(mapdir, po.px, po.py, po.pz, rl, fnamebuf), "w");
	if(!f)
	{
		printf("ERROR opening %s for write. Ignoring received map.\n", fnamebuf);
		return;
	}

	if(fwrite(buf, buflen, 1, f) != 1)
	{
		printf("ERROR: Writing map to file %s failed. Ignoring received map.\n", fnamebuf);
	}

	fclose(f);
}

void render_3d(double campos_x, double campos_y, double campos_z, double camera_yaw, double camera_pitch)
{
	sf::Shader::bind(&shader);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	glm::vec3 pos;
	pos.x = campos_x / ((double)VOX_UNITS[0]/2.0);
	pos.y = campos_z / ((double)VOX_UNITS[0]/2.0);
	pos.z = -1*campos_y / ((double)VOX_UNITS[0]/2.0);

	glm::vec3 targ;
	targ.x = pos.x + 1.0*cos(camera_pitch)*cos(camera_yaw);
	targ.y = pos.y + 1.0*sin(camera_pitch);
	targ.z = pos.z - 1.0*cos(camera_pitch)*sin(camera_yaw);

	glm::vec3 up = glm::vec3(0.0, 1.0, 0.0);
	

	glm::mat4 scale = glm::mat4(1.0f);
	scale = glm::scale(scale, glm::vec3(1.0, 1.0, -1.0));

	view = glm::lookAt(pos, targ, up) * scale;

	


	for(int i=0; i<MAX_MESHES; i++)
	{
		if(pieces_3d[i].xs > 0)
		{
			render_piece(&pieces_3d[i]);
		}
	}

	extern bool show_realtime_pc;
	if(realtime_pc_enabled && show_realtime_pc)
	{
		render_pc(&realtime_pc);
	}

	glBindVertexArray(0); // Unbind the VAO only once, no need to do it for every object


	sf::Shader::bind(NULL);


}

static int find_free_piece()
{
	for(int i=0; i<MAX_MESHES; i++)
	{
		if(pieces_3d[i].xs == 0)
			return i;
	}

	return -1;
}

static int find_piece_by_page(int px, int py, int pz)
{
	for(int i=0; i<MAX_MESHES; i++)
	{
		if(pieces_3d[i].px == px && pieces_3d[i].py == py && pieces_3d[i].pz == pz)
			return i;
	}

	return -1;
}

static int find_piece_by_page_and_rl(int px, int py, int pz, int rl)
{
	for(int i=0; i<MAX_MESHES; i++)
	{
		if(pieces_3d[i].px == px && pieces_3d[i].py == py && pieces_3d[i].pz == pz && pieces_3d[i].rl == rl)
			return i;
	}

	return -1;
}

static void free_pieces_by_page_and_rl(int px, int py, int pz, int rl)
{
	for(int i=0; i<MAX_MESHES; i++)
	{
		if(pieces_3d[i].px == px && pieces_3d[i].py == py && pieces_3d[i].pz == pz && pieces_3d[i].rl == rl && pieces_3d[i].xs > 0)
			free_piece(i);
	}
}

static void free_all_pieces()
{
	for(int i=0; i<MAX_MESHES; i++)
	{
		if(pieces_3d[i].xs > 0)
			free_piece(i);
	}

}

void reload_3d_map(double campos_x, double campos_y, double campos_z, double camera_yaw, double camera_pitch)
{
	free_all_pieces();
//	manage_mesh_ranges(campos_x, campos_y, campos_z, camera_yaw, camera_pitch, 1);
}

void read_voxmap_to_meshes(int px, int py, int pz, int rl)
{
	voxmap_t voxmap;
	static char fnamebuf[2048];
	int ret = read_voxmap(&voxmap, gen_fname(mapdir, px, py, pz, rl, fnamebuf));

	if(ret >= 0)
	{
//		printf("INFO: Succesfully loaded file from disk (%d,%d,%d,rl%d)\n", px, py, pz, rl);

		assert(voxmap.header.xs == VOX_XS[rl]);
		assert(voxmap.header.ys == VOX_YS[rl]);
		assert(voxmap.header.zs == VOX_ZS[rl]);

		int xs = VOX_XS[rl]/pieces_x[rl];
		int ys = VOX_YS[rl]/pieces_y[rl];
		int zs = VOX_ZS[rl]/pieces_z[rl];

//		int y_piece = 0, x_piece = 0, z_piece = 0;
		for(int y_piece = 0; y_piece < pieces_y[rl]; y_piece++)
		{
			for(int x_piece = 0; x_piece < pieces_x[rl]; x_piece++)
			{
				for(int z_piece = 0; z_piece < pieces_z[rl]; z_piece++)
				{
					int idx = find_free_piece();

//					printf("New mesh idx=%d, page(%d,%d,%d,rl%d), piece(%d,%d,%d)\n", 
//						idx, px, py, pz, rl, x_piece, y_piece, z_piece);

					if(idx < 0)
					{
						printf("WARNING: read_voxmap_to_meshes: piece table full, stopping mesh generation.\n");
						goto STOP_GEN;
					}
					mesh_t new_mesh = voxmap_to_mesh(&voxmap,
						x_piece*xs, (x_piece+1)*xs,
						y_piece*ys, (y_piece+1)*ys,
						z_piece*zs, (z_piece+1)*zs,
						pz, rl);

					pieces_3d[idx].mesh = new_mesh;
					pieces_3d[idx].px = px;
					pieces_3d[idx].py = py;
					pieces_3d[idx].pz = pz;
					pieces_3d[idx].rl = rl;

					pieces_3d[idx].xs = xs*VOX_RELATIONS[rl]*2;
					pieces_3d[idx].ys = ys*VOX_RELATIONS[rl]*2;
					pieces_3d[idx].zs = zs*VOX_RELATIONS[rl]*2;

					pieces_3d[idx].x_start = (px-MAX_PAGES_X/2)*VOX_XS[0]*2 + x_piece*xs*VOX_RELATIONS[rl]*2;
					pieces_3d[idx].y_start = (py-MAX_PAGES_Y/2)*VOX_YS[0]*2 + y_piece*ys*VOX_RELATIONS[rl]*2;
					pieces_3d[idx].z_start = (pz-MAX_PAGES_Z/2)*VOX_ZS[0]*2 + z_piece*zs*VOX_RELATIONS[rl]*2;

//					printf("size (%d, %d, %d), start (%d,%d,%d)\n", 
//						pieces_3d[idx].xs, pieces_3d[idx].ys, pieces_3d[idx].zs,
//						pieces_3d[idx].x_start, pieces_3d[idx].y_start, pieces_3d[idx].z_start);


				}

			}

		}

		STOP_GEN:;
		deinit_voxmap(&voxmap);
	}
}


volatile int manag_px;
volatile int manag_py;
volatile int manag_pz;
volatile int manag_xbias;
volatile int manag_ybias;
volatile int manag_zbias;


static int do_manage_mesh_ranges()
{
	int poc_px = manag_px;
	int poc_py = manag_py;
	int poc_pz = manag_pz;
	int xbias = manag_xbias;
	int ybias = manag_ybias;
	int zbias = manag_zbias;
	
//	int xbias = 0;
//	int ybias = 0;
//	int zbias = 0;

	static const int extra_x[MAX_RESOLEVELS] = {1, 2, 4, 8};
	static const int extra_y[MAX_RESOLEVELS] = {1, 2, 4, 8};
	static const int extra_z[MAX_RESOLEVELS] = {1, 2, 3, 8};


	printf("bias %d  %d  %d\n", xbias, ybias, zbias);

	int start_x[MAX_RESOLEVELS];
	int end_x[MAX_RESOLEVELS];
	int start_y[MAX_RESOLEVELS];
	int end_y[MAX_RESOLEVELS];
	int start_z[MAX_RESOLEVELS];
	int end_z[MAX_RESOLEVELS];

	for(int rl=0; rl<4; rl++)
	{
		start_x[rl] = poc_px-extra_x[rl]+xbias*rl;
		end_x[rl] = poc_px+extra_x[rl]+xbias*rl;
		start_y[rl] = poc_py-extra_y[rl]+ybias*rl;
		end_y[rl] = poc_py+extra_y[rl]+ybias*rl;
		start_z[rl] = poc_pz-extra_z[rl]+zbias*rl;
		end_z[rl] = poc_pz+extra_z[rl]+zbias*rl;

		if(start_x[rl] < 0) start_x[rl] = 0;
		if(end_x[rl] > MAX_PAGES_X-1) end_x[rl] = MAX_PAGES_X-1;
		if(start_y[rl] < 0) start_y[rl] = 0;
		if(end_y[rl] > MAX_PAGES_Y-1) end_y[rl] = MAX_PAGES_Y-1;
		if(start_z[rl] < 0) start_z[rl] = 0;
		if(end_z[rl] > MAX_PAGES_Z-1) end_z[rl] = MAX_PAGES_Z-1;
	}


	for(int py = start_y[3]; py <= end_y[3]; py++)
	{
		for(int px = start_x[3]; px <= end_x[3]; px++)
		{
			for(int pz = start_z[3]; pz <= end_z[3]; pz++)
			{
				for(int rl=0; rl<4; rl++)
				{
					if(py >= start_y[rl] && py <= end_y[rl] &&
					   px >= start_x[rl] && px <= end_x[rl] &&
					   pz >= start_z[rl] && pz <= end_z[rl])
					{
						// This page should be loaded at this rl - first free all lower rls

						// printf("LOAD   (%3d,%3d,%3d,rl%d)\n", px, py, pz, rl);

						if(find_piece_by_page_and_rl(px, py, pz, rl) == -1)
						{
							//if(px==261&&py==252&&pz==15)  printf("LOAD   (%3d,%3d,%3d,rl%d)\n", px, py, pz, rl);
							read_voxmap_to_meshes(px, py, pz, rl);
						}

						for(int lower_rl=rl+1; lower_rl<4; lower_rl++)
						{
							//if(px==261&&py==252&&pz==15) printf("FREE1  (%3d,%3d,%3d,rl%d)\n", px, py, pz, lower_rl);
							free_pieces_by_page_and_rl(px, py, pz, lower_rl);
						}

						break; // Don't load lower rls
					}
					else
					{
						//if(px==261&&py==252&&pz==15)  printf("FREE2  (%3d,%3d,%3d,rl%d)\n", px, py, pz, rl);

						// This page shoudln't be loaded at this rl
						free_pieces_by_page_and_rl(px, py, pz, rl);
					}
				}
			}
		}
	}	
}

sf::Thread thread(&do_manage_mesh_ranges);

int manage_mesh_ranges(double campos_x, double campos_y, double campos_z, double camera_yaw, double camera_pitch, int force)
{

//	int px = MAX_PAGES_X/2;
//	int py = MAX_PAGES_Y/2;
//	int pz = MAX_PAGES_Z/2;


	po_coords_t poc = po_coords(campos_x, campos_y, campos_z, 0);

	double x_bias = 1.5*cos(camera_yaw);
	double y_bias = 1.5*sin(camera_yaw);
	double z_bias = 1.5*sin(camera_pitch);

//	printf("f bias  %.2f  %.2f  %.2f\n", x_bias, y_bias, z_bias);

	int xb = (int)x_bias;
	int yb = (int)y_bias;
	int zb = (int)z_bias;

	static int prev_px = -1, prev_py = -1, prev_pz = -1;
	static int prev_xbias = -999, prev_ybias = -999, prev_zbias = -999;

	if(!force && poc.px == prev_px && poc.py == prev_py && poc.pz == prev_pz && prev_xbias == xb && prev_ybias == yb && prev_zbias == zb)
	{
		return;
	}

	prev_px = poc.px;
	prev_py = poc.py;
	prev_pz = poc.pz;

	prev_xbias = xb;
	prev_ybias = yb;
	prev_zbias = zb;

	thread.wait();

	manag_px = poc.px;
	manag_py = poc.py;
	manag_pz = poc.pz;


	manag_xbias = xb;
	manag_ybias = yb;
	manag_zbias = zb;

//	printf("manag_bias  %d  %d  %d\n", manag_xbias, manag_);

	thread.launch();

/*
	int px = poc.px;
	int py = poc.py;
	int pz = poc.pz;

	printf("(%3d,%3d,%3d,rl%d)\n", px, py, pz, rl);


	free_all_pieces();


	if(find_piece_by_page_and_rl(px, py, pz, rl) == -1)
		read_voxmap_to_meshes(px, py, pz, rl);
	else
	{
		printf("KAKKA!\n");
		abort();
	}
*/


	
/*
	for(int py = MAX_PAGES_Y/2-5; py < MAX_PAGES_Y/2+5; py++)
	{
		for(int px = MAX_PAGES_X/2-5; px < MAX_PAGES_X/2+5; px++)
		{
			for(int pz = MAX_PAGES_Z/2-2; pz < MAX_PAGES_Z/2+2; pz++)
			{
				if(find_piece_by_page(px, py, pz) == -1)
				{
					read_voxmap_to_meshes(px, py, pz, 2);
				}
			}

		}

	}
*/
	return 0;

}

void wait_manage_mesh()
{
//	printf("wait...\n");
	thread.wait();
//	printf("...ok\n");
}


