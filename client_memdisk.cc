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

#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <GL/glu.h>

#include "client_memdisk.h"

extern "C" {
#include "../robotsoft/voxmap.h"
#include "../robotsoft/voxmap_memdisk.h"
}


#if 0
int read_map_page(world_t* w, int pagex, int pagey)
{
	char fname[1024];
	sprintf(fname, "%08x_%u_%u_%u.map", robot_id, w->id, pagex, pagey);

//	printf("Info: Attempting to read map page %s\n", fname);

	FILE *f = fopen(fname, "r");
	if(!f)
	{
		if(errno == ENOENT)
			return 2;
		fprintf(stderr, "Error %d opening %s for read\n", errno, fname);
		return 1;
	}

	int ret;
	if( (ret = fread(w->pages[pagex][pagey], sizeof(map_page_t), 1, f)) != 1)
	{
		printf("Error: Reading map data failed, fread returned %d. feof=%d, ferror=%d\n", ret, feof(f), ferror(f));
	}

	fclose(f);
	return 0;
}

int load_map_page(world_t* w, int pagex, int pagey)
{
	if(w->pages[pagex][pagey])
	{
//		printf("Info: reloading already allocated map page %d,%d\n", pagex, pagey);
	}
	else
	{
//		printf("Info: Allocating mem for page %d,%d\n", pagex, pagey);
		w->pages[pagex][pagey] = (map_page_t*)malloc(sizeof(map_page_t));
	}

	int ret = read_map_page(w, pagex, pagey);
	if(ret)
	{
		printf("Error: Reading map file failed. Initializing empty map\n");
		memset(w->pages[pagex][pagey], 0, sizeof(map_page_t));
		return 1;
	}
	return 0;
}

int unload_map_page(world_t* w, int pagex, int pagey)
{
	if(w->pages[pagex][pagey])
	{
		printf("Info: Freeing mem for page %d,%d\n", pagex, pagey);
		free(w->pages[pagex][pagey]);
		w->pages[pagex][pagey] = 0;
	}
	else
	{
		printf("Warn: Trying to unload a map page which is already free.\n");
	}
	return 0;
}

int unload_map_pages(world_t* w, int cur_pagex, int cur_pagey)
{
	for(int x = 0; x < MAP_W; x++)
	{
		for(int y = 0; y < MAP_W; y++)
		{
			if(w->pages[x][y] && (abs(cur_pagex - x) > 2 || abs(cur_pagey - y) > 2))
			{
				unload_map_page(w, x, y);
			}

		}
	}
	return 0;
}
#endif

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



static uint8_t pages_exist[MAX_PAGES_X*MAX_PAGES_Y];

static sf::Texture tex_full_map;


typedef struct
{
	sf::Texture* textures[8];
} map_2d_page_pile_t;

map_2d_page_pile_t piles[MAX_PAGES_X*MAX_PAGES_Y];

int view2d_min_z = -10000;
int view2d_max_z = 10000;

void load_page_pile_from_disk(int px, int py, int rl, int do_reload)
{
	assert(px >= 0 && px < MAX_PAGES_X && py >= 0 && py < MAX_PAGES_Y && rl >= 0 && rl < 8);

	int do_read = 0;
	if(piles[py*MAX_PAGES_X+px].textures[rl])
	{
		// Already loaded
		if(do_reload)
		{
			do_read = 1;
		}
	}
	else
	{
		do_read = 1;
		piles[py*MAX_PAGES_X+px].textures[rl] = new sf::Texture;
		piles[py*MAX_PAGES_X+px].textures[rl]->create(VOX_XS[rl], VOX_YS[rl]);
		piles[py*MAX_PAGES_X+px].textures[rl]->setSmooth(false);
	}

	if(do_read)
	{
		int32_t* max_z = malloc(VOX_XS[rl]*VOX_YS[rl]*sizeof(int32_t));
		for(int i=0; i<VOX_XS[rl]*VOX_YS[rl]; i++)
			max_z[i] = -1;

		for(int pz=0; pz < MAX_PAGES_Z; pz++)
		{
			voxmap_t voxmap;
			
			int ret = read_uncompressed_voxmap(&voxmap, gen_fname("./current_maps", px, py, pz, rl));

			if(ret >= 0)
			{
				printf("INFO: Succesfully loaded file from the disk (%d,%d,%d,rl%d)\n", px, py, pz, rl);

				assert(voxmap.header.xs == VOX_XS[rl]);
				assert(voxmap.header.ys == VOX_YS[rl]);
				assert(voxmap.header.zs == VOX_ZS[rl]);
				for(int xy=0; xy<VOX_XS[rl]*VOX_YS[rl]; xy++)
				{
					int z;
					for(z=VOX_ZS[rl]-1; z >= 0; z--)
					{
						if(voxmap.voxels[xy*VOX_ZS[rl]+z] & 0x0f)
							break;
					}
					// z has highest z, or -1 if not voxel found at all
					
					// The pz loop goes from down to up: if we have anything, it's the biggest.
					if(z > -1)
						max_z[xy] = pz*VOX_ZS[rl]+z;
				}
			}
		}

		uint8_t* tmpimg = malloc(VOX_XS[rl]*VOX_YS[rl]*4);
		assert(tmpimg);

		for(int yy=0; yy<VOX_YS[rl]; yy++)
		{
			for(int xx=0; xx<VOX_XS[rl]; xx++)
			{
				int r, g, b, a;
				if(max_z[yy*VOX_XS[rl]+xx] < 0)
				{
					r = 255;
					g = 255;
					b = 255;
					a = 255;
				}
				else
				{
					int32_t out_z = max_z[yy*VOX_XS[rl]+xx];
					out_z *= VOX_UNITS[rl]; // to mm
					out_z -= (MAX_PAGES_Z/2)*MAP_PAGE_Z_H_MM; // to zero-referenced absolute mm

					//const int biggest_possible = VOX_ZS[0]*MAX_PAGES_Z;

					if(out_z < view2d_min_z)
					{
						r = 160;
						g = 160;
						b = 255;
						a = 255;
					}
					else if(out_z > view2d_max_z)
					{
						r = 255;
						g = 160;
						b = 160;
						a = 255;
					}
					else
					{

						int range = view2d_max_z - view2d_min_z;

						r = (255*(out_z-view2d_min_z))/range;
						if(r < 0) r = 0; else if(r > 255) r = 255;

						b = (255*(view2d_max_z-out_z))/range;
						if(b < 0) b = 0; else if(b > 255) b = 255;

						g =  256 - r - b;
						if(g < 0) g = 0; else if(g > 255) g = 255;

						a = 255;
					}
				}

				// In SFML coordinate system, y is mirrored - mirror in the texture.
				tmpimg[((VOX_XS[rl]-1-yy)*VOX_XS[rl]+xx)*4+0] = r;
				tmpimg[((VOX_XS[rl]-1-yy)*VOX_XS[rl]+xx)*4+1] = g;
				tmpimg[((VOX_XS[rl]-1-yy)*VOX_XS[rl]+xx)*4+2] = b;
				tmpimg[((VOX_XS[rl]-1-yy)*VOX_XS[rl]+xx)*4+3] = a;
			}
		}

		piles[py*MAX_PAGES_X+px].textures[rl]->update(tmpimg);
		free(tmpimg);
	}
}



void init_memdisk()
{
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

void draw_page_piles(sf::RenderWindow& win, int rl)
{
	int pix_start_x = origin_x/mm_per_pixel - screen_x/2;
	int pix_end_x = origin_x/mm_per_pixel + screen_x/2;
	int pix_start_y = origin_y/mm_per_pixel - screen_y/2;
	int pix_end_y = origin_y/mm_per_pixel + screen_y/2;

	int mm_start_x = pix_start_x*mm_per_pixel;
	int mm_end_x = pix_end_x*mm_per_pixel;
	int mm_start_y = pix_start_y*mm_per_pixel;
	int mm_end_y = pix_end_y*mm_per_pixel;

	po_coords_t poc_s = po_coords(mm_start_x, mm_start_y, 0, rl);
	po_coords_t poc_e = po_coords(mm_end_x, mm_end_y, 0, rl);

//	for(int py=poc_s.py; py<=poc_e.py; py++)
	for(int py=256-16; py<=256+16; py++)
	{
//		for(int px=poc_s.px; px<=poc_e.px; px++)
		for(int px=256-16; px<=256+16; px++)
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
				win.draw(sprite);
			}
		}
	}
}

void reload_page_pile_range()
{
//	int startx = -MAP_MIDDLE_UNIT*MAP_UNIT_W + x*MAP_PAGE_W*MAP_UNIT_W + origin_x;
//	int starty = -1*(-MAP_MIDDLE_UNIT*MAP_UNIT_W + (y+1)*MAP_PAGE_W*MAP_UNIT_W) + origin_y;

//	int startx = + x*MAP_PAGE_W*MAP_UNIT_W + origin_x;
//	int starty = -1*((y+1)*MAP_PAGE_W*MAP_UNIT_W) + origin_y;


	//int start_x_mm


	//load_page_pile_from_disk(int px, int py, int rl, int do_reload)

}


void load_all_pages_on_disk()
{
	memset(pages_exist, 0, MAX_PAGES_X*MAX_PAGES_Y);
	DIR *d;
	struct dirent *dir;
	d = opendir("./current_maps");

	uint8_t* highest_pz = calloc(MAX_PAGES_X*MAX_PAGES_Y, sizeof(uint8_t));
	assert(highest_pz);

	if(d)
	{
		while( (dir = readdir(d)) != 0)
		{
			unsigned int px, py, pz, rl;

			if(sscanf(dir->d_name, "voxmap_x%u_y%u_z%u_r%u.pluuvox", &px, &py, &pz, &rl) == 4)
			{
				assert(px >= 0 && px < MAX_PAGES_X && py >= 0 && py < MAX_PAGES_Y && pz >=0 && pz < MAX_PAGES_Z && rl >= 0 && rl < 8);
				pages_exist[py*MAX_PAGES_X+px] = 1;
				if(pz+1 > highest_pz[py*MAX_PAGES_X+px])
					highest_pz[py*MAX_PAGES_X+px] = pz+1;

				load_page_pile_from_disk(px, py, rl, 0);
			}

		}

		closedir(d);
	}


	uint8_t* tmp = malloc(sizeof(uint8_t)*MAX_PAGES_X*MAX_PAGES_Y*4);
	assert(tmp);
	memset(tmp, 255, sizeof(uint8_t)*MAX_PAGES_X*MAX_PAGES_Y*4);

	for(int i=0; i<MAX_PAGES_X*MAX_PAGES_Y; i++)
	{
		if(highest_pz[i] > 0)
		{
			int r, g, b;
			r = -1*(MAX_PAGES_Z-2*highest_pz[i])*(256/MAX_PAGES_Z);
			if(r < 0) r=0; else if(r>255) r=255;

			b = +1*(MAX_PAGES_Z-2*highest_pz[i])*(256/MAX_PAGES_Z);
			if(b < 0) b=0; else if(b>255) b=255;

			g = 256 - r - b;
			if(g < 0) g=0; else if(g>255) g=255;

			tmp[i*4+0] = r;
			tmp[i*4+1] = g;
			tmp[i*4+2] = b;
			tmp[i*4+3] = 255;
		}	
	}

	free(highest_pz);

	tex_full_map.update(tmp);
	free(tmp);
}


