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



	This program takes direct TCP connection to the robot:
	./rn1client robot_hostname robot_port

	Needs code quality improvement. I haven't been able to decide whether this is 
	a prototype-to-be-replaced, or a maintained application. It works nevertheless :-).

	Library dependencies:
	SFML (at least 2.4.2 works; SFML tends to have slight compatibility breaks every now and then)

*/


#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <errno.h>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <SFML/Graphics.hpp>
#include <SFML/Network.hpp>


#define DEFINE_API_VARIABLES
#include "../robotsoft/api_board_to_soft.h"
#include "../robotsoft/api_soft_to_board.h"
#undef DEFINE_API_VARIABLES

#include "../robotsoft/mapping.h"
#include "../robotsoft/datatypes.h"
#include "../rn1-brain/comm.h"
#include "client_memdisk.h"
#include "uthash.h"
#include "utlist.h"
#include "sfml_gui.h"


#define I16FROMBUF(b_, s_)  ( ((uint16_t)b_[(s_)+0]<<8) | ((uint16_t)b_[(s_)+1]<<0) )
#define I32FROMBUF(b_, s_)  ( ((uint32_t)b_[(s_)+0]<<24) | ((uint32_t)b_[(s_)+1]<<16) | ((uint32_t)b_[(s_)+2]<<8) | ((uint32_t)b_[(s_)+3]<<0) )
#define I32TOBUF(i_, b_, s_) {(b_)[(s_)] = ((i_)>>24)&0xff; (b_)[(s_)+1] = ((i_)>>16)&0xff; (b_)[(s_)+2] = ((i_)>>8)&0xff; (b_)[(s_)+3] = ((i_)>>0)&0xff; }
#define I16TOBUF(i_, b_, s_) {(b_)[(s_)] = ((i_)>>8)&0xff; (b_)[(s_)+1] = ((i_)>>0)&0xff; }


world_t world;


int pict_id, pict_bpp, pict_xs, pict_ys, dbg_boost;
uint8_t pict_data[1000000];

char status_text[2000];

uint32_t robot_id = 0xacdcabba;
int cur_speed_limit = 45;

sf::Font arial;

int screen_x = 1200;
int screen_y = 700;

double click_x, click_y;

double mm_per_pixel = 10.0;
double origin_x = 0;
double origin_y = 0;


double cur_angle = 0.0;
double cur_pitch = 0.0;
double cur_roll = 0.0;
double cur_x = 0.0;
double cur_y = 0.0;

int show_dbgpoint, dbgpoint_x, dbgpoint_y, dbgpoint_r, dbgpoint_g, dbgpoint_b;

int num_pers_dbgpoints;
int pers_dbgpoint_x[100], pers_dbgpoint_y[100], pers_dbgpoint_r[100], pers_dbgpoint_g[100], pers_dbgpoint_b[100];


double route_start_x, route_start_y;

typedef enum {MODE_INVALID = -1, MODE_ROUTE = 0, MODE_MANUAL_FWD, MODE_MANUAL_BACK, MODE_FORCE_FWD, MODE_FORCE_BACK, MODE_POSE, MODE_ADDCONSTRAINT, MODE_REMCONSTRAINT} click_mode_t;
click_mode_t click_mode;

double dest_x, dest_y;
click_mode_t dest_type = MODE_INVALID;

double robot_xs = 650.0;
double robot_ys = 460.0;
double lidar_xoffs = 120.0;
double lidar_yoffs = 0.0;


#define RGBA32(r_,g_,b_,a_)  ((r_) | ((g_)<<8) | ((b_)<<16) | ((a_)<<24))


float offset_z = 400.0;
float blue_z = 1200.0;

int cur_slice = 0;
int max_voxmap_alpha = 200;
int voxmap_alpha;


#define VOXMAP_ALPHA 255
static const uint32_t voxmap_blank_color = RGBA32(0UL,  0UL, 0UL, 50UL);
static const uint32_t forbidden_color = RGBA32(255UL,  190UL, 190UL, 255UL);

static const uint32_t voxmap_colors[16] = {
/* 0           */ RGBA32(220UL, 50UL,220UL, VOXMAP_ALPHA),
/* 1           */ RGBA32(140UL, 50UL,220UL, VOXMAP_ALPHA),
/* 2           */ RGBA32(100UL,120UL,220UL, VOXMAP_ALPHA),
/* 3           */ RGBA32( 20UL,100UL,240UL, VOXMAP_ALPHA),
/* 4           */ RGBA32(  0UL,130UL,200UL, VOXMAP_ALPHA),
/* 5           */ RGBA32(  0UL,160UL,160UL, VOXMAP_ALPHA),
/* 6           */ RGBA32(  0UL,200UL,130UL, VOXMAP_ALPHA),
/* 7           */ RGBA32(  0UL,220UL, 70UL, VOXMAP_ALPHA),
/* 8           */ RGBA32(  0UL,250UL,  0UL, VOXMAP_ALPHA),
/* 9           */ RGBA32( 50UL,220UL,  0UL, VOXMAP_ALPHA),
/* 10          */ RGBA32( 90UL,190UL,  0UL, VOXMAP_ALPHA),
/* 11          */ RGBA32(120UL,150UL,  0UL, VOXMAP_ALPHA),
/* 12          */ RGBA32(150UL,120UL,  0UL, VOXMAP_ALPHA),
/* 13          */ RGBA32(190UL, 90UL,  0UL, VOXMAP_ALPHA),
/* 14          */ RGBA32(220UL, 50UL,  0UL, VOXMAP_ALPHA),
/* 15          */ RGBA32(250UL,  0UL,  0UL, VOXMAP_ALPHA),
};



int charging;
int charge_finished;
float bat_voltage;
float cha_voltage;
int bat_percentage;

int cur_cmd_status;
void print_cur_cmd_status(sf::RenderWindow& win)
{
	char tbu[1000];
	sf::Text t;
	t.setFont(arial);
	switch(cur_cmd_status)
	{
		case 355: sprintf(tbu, "Working on: Direct (manual) move"); break;
		case 356: sprintf(tbu, "Working on: Routefinding"); break;
		case 357: sprintf(tbu, "Working on: Finding the charger"); break;
		default: sprintf(tbu, "State bar"); break;
	}

	t.setString(tbu);
	t.setCharacterSize(14);
	t.setFillColor(sf::Color(200,255,200,200));
	t.setPosition(10, screen_y-28);
	win.draw(t);
	t.setFillColor(sf::Color(0,100,0,255));
	t.setPosition(8, screen_y-30);
	win.draw(t);
}

const char* click_mode_names[8] =
{
	"click to find route",
	"click to go directly (forward drive)",
	"click to go directly (REVERSE drive)",
	"click to force the robot (forward drive)",
	"click to force the robot (REVERSE drive)",
	"click to rotate the pose",
	"click to add a forbidden place",
	"click to remove a forbidden place"
};

const sf::Color click_mode_colors[8] = {
sf::Color(110, 255, 110, 190),
sf::Color(235, 235, 110, 190),
sf::Color(235, 235, 110, 190),
sf::Color(255, 110, 110, 190),
sf::Color(255, 110, 110, 190),
sf::Color(110, 200, 200, 190),
sf::Color(255, 110, 190, 190),
sf::Color(255, 110, 190, 190)
};

static int rsync_running = 0;

static char *rsync_argv[4];

void init_rsync_argv()
{
	rsync_argv[0] = (char*)malloc(100);
	strcpy(rsync_argv[0], "/bin/bash");

	rsync_argv[1] = (char*)malloc(100);
	strcpy(rsync_argv[1], "map_sync.sh");

	rsync_argv[2] = (char*)malloc(1024);

	rsync_argv[3] = NULL;
}

void deinit_rsync_argv()
{
	free(rsync_argv[0]);
	free(rsync_argv[1]);
	free(rsync_argv[2]);
}

pid_t my_pid;
static void run_map_rsync()
{
	if(rsync_running)
	{
		printf("rsync still running\n");
		return;
	}

	if((my_pid = fork()) == 0)
	{
		if((execve(rsync_argv[0], (char **)rsync_argv , NULL)) == -1)
		{
			printf("run_map_rsync(): execve failed\n");
		}
	}
	else
	{
		rsync_running = 1;
	}
}

static int poll_map_rsync()
{
	if(!rsync_running)
		return -998;

	int status = 0;
	if(waitpid(my_pid , &status , WNOHANG) == 0)
		return -999;

	rsync_running = 0;
	printf("rsync returned %d\n", status);
	return status;
}


void page_coords(int mm_x, int mm_y, int* pageidx_x, int* pageidx_y, int* pageoffs_x, int* pageoffs_y)
{
	int unit_x = mm_x / MAP_UNIT_W;
	int unit_y = mm_y / MAP_UNIT_W;
	unit_x += MAP_MIDDLE_UNIT;
	unit_y += MAP_MIDDLE_UNIT;
	int page_x = unit_x / MAP_PAGE_W;
	int page_y = unit_y / MAP_PAGE_W;
	int offs_x = unit_x - page_x*MAP_PAGE_W;
	int offs_y = unit_y - page_y*MAP_PAGE_W;

	*pageidx_x = page_x;
	*pageidx_y = page_y;
	*pageoffs_x = offs_x;
	*pageoffs_y = offs_y;
}

void unit_coords(int mm_x, int mm_y, int* unit_x, int* unit_y)
{
	int unit_x_t = mm_x / MAP_UNIT_W;
	int unit_y_t = mm_y / MAP_UNIT_W;
	unit_x_t += MAP_MIDDLE_UNIT;
	unit_y_t += MAP_MIDDLE_UNIT;

	*unit_x = unit_x_t;
	*unit_y = unit_y_t;
}

void mm_from_unit_coords(int unit_x, int unit_y, int* mm_x, int* mm_y)
{
	unit_x -= MAP_MIDDLE_UNIT;
	unit_y -= MAP_MIDDLE_UNIT;

	*mm_x = unit_x * MAP_UNIT_W;
	*mm_y = unit_y * MAP_UNIT_W;
}

void page_coords_from_unit_coords(int unit_x, int unit_y, int* pageidx_x, int* pageidx_y, int* pageoffs_x, int* pageoffs_y)
{
	int page_x = unit_x / MAP_PAGE_W;
	int page_y = unit_y / MAP_PAGE_W;
	int offs_x = unit_x - page_x*MAP_PAGE_W;
	int offs_y = unit_y - page_y*MAP_PAGE_W;

	*pageidx_x = page_x;
	*pageidx_y = page_y;
	*pageoffs_x = offs_x;
	*pageoffs_y = offs_y;
}


typedef struct
{
	int x;
	int y;
} xy_t;

typedef struct route_unit_T
{
	xy_t loc;
	int backmode;
	route_unit_T* prev;
	route_unit_T* next;
} route_unit_t;


#define sq(x) ((x)*(x))

void dev_draw_circle(sf::RenderWindow& win, int unit_x, int unit_y, int r, int g, int b, int dir)
{
	int x_mm, y_mm;
	mm_from_unit_coords(unit_x, unit_y, &x_mm, &y_mm);

	sf::CircleShape circ(14.0/mm_per_pixel);
	circ.setOrigin(14.0/mm_per_pixel, 14.0/mm_per_pixel);
	circ.setFillColor(sf::Color(r,g,b));

	if(dir==-123)
	{
		circ.setFillColor(sf::Color::Transparent);
		circ.setOutlineThickness(2.0);
		circ.setOutlineColor(sf::Color(r,g,b));

	}

	circ.setPosition((x_mm+origin_x+MAP_UNIT_W/2)/mm_per_pixel,(-1*y_mm+origin_y+MAP_UNIT_W/2)/mm_per_pixel);
	win.draw(circ);

	if(dir >= 0)
	{
		sf::ConvexShape arrow(3);
		arrow.setPoint(0, sf::Vector2f(0,-12.0/mm_per_pixel));
		arrow.setPoint(1, sf::Vector2f(0,12.0/mm_per_pixel));
		arrow.setPoint(2, sf::Vector2f(12.0/mm_per_pixel,0));

		arrow.setOrigin(0,0);

		arrow.setFillColor(sf::Color((r/2),(g/2),(b/2)));

		arrow.setRotation((float)dir*(360.0/32.0));
		arrow.setPosition((x_mm+origin_x+MAP_UNIT_W/2)/mm_per_pixel,(-1*y_mm+origin_y+MAP_UNIT_W/2)/mm_per_pixel);
		win.draw(arrow);
	}

}

void draw_map(sf::RenderWindow& win);

#define TODEG(x) ((360.0*x)/(2.0*M_PI))


route_unit_t *some_route = NULL;
route_unit_t *p_cur_step = NULL;

void clear_route(route_unit_t **route)
{
	route_unit_t *elt, *tmp;
	DL_FOREACH_SAFE(*route,elt,tmp)
	{
		DL_DELETE(*route,elt);
		free(elt);
	}
	*route = NULL;
}

void draw_route_mm(sf::RenderWindow& win, route_unit_t **route)
{
	if(!route)
		return;

	route_unit_t *rt;
	route_unit_t *prev;
	int first = 1;
	DL_FOREACH(*route, rt)
	{
		float x1, x2, y1, y2;

		if(first)
		{
			x1 = (route_start_x+origin_x)/mm_per_pixel;
			y1 = (-1*route_start_y+origin_y)/mm_per_pixel;
			first = 0;
		}
		else
		{
			x1 = (prev->loc.x+origin_x)/mm_per_pixel;
			y1 = (-1*prev->loc.y+origin_y)/mm_per_pixel;
		}

		x2 = (rt->loc.x+origin_x)/mm_per_pixel;
		y2 = (-1*rt->loc.y+origin_y)/mm_per_pixel;
		sf::RectangleShape rect(sf::Vector2f( sqrt(pow(x2-x1,2)+pow(y2-y1,2)), 6.0));
		rect.setOrigin(0, 3.0);
		rect.setPosition(x1, y1);
		rect.setRotation(atan2(y2-y1,x2-x1)*180.0/M_PI);
		rect.setFillColor(rt->backmode?sf::Color(180,0,0,170):sf::Color(0,180,0,170));

		win.draw(rect);

		prev = rt;
	}
}

void draw_drive_diag(sf::RenderWindow& win, drive_diag_t *mm)
{

	int32_t ang_err;
	int32_t lin_err;
	int32_t cur_x;
	int32_t cur_y;
	int32_t target_x;
	int32_t target_y;
	int32_t id;
	int32_t remaining;
	uint32_t micronavi_stop_flags;


	float x1, x2, y1, y2;
	x1 = (mm->cur_x+origin_x)/mm_per_pixel;
	y1 = (-1*mm->cur_y+origin_y)/mm_per_pixel;
	x2 = (mm->target_x+origin_x)/mm_per_pixel;
	y2 = (-1*mm->target_y+origin_y)/mm_per_pixel;

	sf::RectangleShape rect(sf::Vector2f( sqrt(pow(x2-x1,2)+pow(y2-y1,2)), 2.0));
	rect.setOrigin(0, 1.0);
	rect.setPosition(x1, y1);
	rect.setRotation(atan2(y2-y1,x2-x1)*180.0/M_PI);
	rect.setFillColor(sf::Color(255,255,255,170));

	win.draw(rect);


	sf::Text t;
	char buf[256];
	t.setFont(arial);

	sprintf(buf, "%.1f deg", ANG_I32TOFDEG(mm->ang_err));
	t.setString(buf);
	t.setCharacterSize(12);
	t.setFillColor(sf::Color(255,255,255,255));
	t.setPosition((x1+x2)/2 + 4, (y1+y2)/2 - 7);
	win.draw(t);

	sprintf(buf, "%d mm", mm->lin_err);
	t.setString(buf);
	t.setCharacterSize(12);
	t.setFillColor(sf::Color(255,255,255,255));
	t.setPosition((x1+x2)/2 + 4, (y1+y2)/2 + 7);
	win.draw(t);

}


//#define WALL_LEVEL(i) ((int)(i).num_obstacles*4)
#define WALL_LEVEL(i) ((int)(i).num_obstacles*2)

void draw_page(sf::RenderWindow& win, map_page_t* page, int startx, int starty)
{
	if(!page)
		return;

	static uint32_t pixels[MAP_PAGE_W*MAP_PAGE_W];

	memset(pixels, 0xff, sizeof pixels);

	for(int x = 0; x < MAP_PAGE_W; x++)
	{
		for(int y = 0; y < MAP_PAGE_W; y++)
		{
			if(page->meta[(y/2)*(MAP_PAGE_W/2)+(x/2)].constraints & CONSTRAINT_FORBIDDEN)
			{
				pixels[(MAP_PAGE_W-1-y)*MAP_PAGE_W+x] = forbidden_color;
			}
			else
			{
				uint32_t val = page->voxmap[y*MAP_PAGE_W+x];
				/* really one at once:
				if(val&(1<<cur_slice))
					pixels[yy*xs+xx] = voxmap_colors[cur_slice];
				else
					pixels[yy*xs+xx] = voxmap_blank_color;
				*/

				pixels[(MAP_PAGE_W-1-y)*MAP_PAGE_W+x] = voxmap_blank_color;
				for(int slice = 0; slice<cur_slice; slice++)
				{
					if(val&(1<<slice))
						pixels[(MAP_PAGE_W-1-y)*MAP_PAGE_W+x] = voxmap_colors[slice];
				}

			}
		}
	}

	float scale = ((float)MAP_PAGE_W_MM/mm_per_pixel)/256.0f;

	sf::Texture t;
	t.create(256, 256);
	t.setSmooth(false);
	t.update((uint8_t*)pixels);
	sf::Sprite sprite;

//	sprite.setOrigin(0, 255);
	sprite.setTexture(t);
	sprite.setPosition((startx)/mm_per_pixel, (starty)/mm_per_pixel);
	sprite.setScale(sf::Vector2f(scale, scale));
	win.draw(sprite);

	sf::RectangleShape b1(sf::Vector2f(MAP_PAGE_W_MM/mm_per_pixel, 1));
	b1.setPosition((startx+0*MAP_UNIT_W)/mm_per_pixel, (starty+0*MAP_UNIT_W)/mm_per_pixel);
	b1.setFillColor(sf::Color(0,0,0,64));
	win.draw(b1);

	sf::RectangleShape b2(sf::Vector2f(MAP_PAGE_W_MM/mm_per_pixel, 1));
	b2.setPosition((startx+0*MAP_UNIT_W)/mm_per_pixel, (starty+256*MAP_UNIT_W)/mm_per_pixel);
	b2.setFillColor(sf::Color(0,0,0,64));
	win.draw(b2);

	sf::RectangleShape b3(sf::Vector2f(1, MAP_PAGE_W_MM/mm_per_pixel));
	b3.setPosition((startx+0*MAP_UNIT_W)/mm_per_pixel, (starty+0*MAP_UNIT_W)/mm_per_pixel);
	b3.setFillColor(sf::Color(0,0,0,64));
	win.draw(b3);

	sf::RectangleShape b4(sf::Vector2f(1, MAP_PAGE_W_MM/mm_per_pixel));
	b4.setPosition((startx+256*MAP_UNIT_W)/mm_per_pixel, (starty+0*MAP_UNIT_W)/mm_per_pixel);
	b4.setFillColor(sf::Color(0,0,0,64));
	win.draw(b4);

}

int32_t hwdbg[10];

void draw_hwdbg(sf::RenderWindow& win)
{
	sf::Text t;
	char buf[500];
	t.setFont(arial);
	t.setCharacterSize(11);
	t.setFillColor(sf::Color(0,0,0,190));
	for(int i = 0; i<10; i++)
	{
		sprintf(buf, "dbg[%2i] = %11d (%08x)", i, hwdbg[i], hwdbg[i]);
		t.setString(buf);
		t.setPosition(10,screen_y-170-30 + 15*i);
		win.draw(t);
	}
}

pwr_status_t latest_pwr;

void draw_bat_status(sf::RenderWindow& win)
{
	sf::Text t;
	char buf[256];
	t.setFont(arial);

	sprintf(buf, "BATT %2.2f V (%d%%)", latest_pwr.bat_mv/1000.0, latest_pwr.bat_percent);
	t.setString(buf);
	t.setCharacterSize(18);
	float vlevel = (float)latest_pwr.bat_percent/100.0;
	int r = (1.0-vlevel)*250.0;
	int g = vlevel*250.0;
	if(r > 250) r = 250;
	if(r<0) r=0;
	if(g > 250) g = 250;
	if(g<0) g=0;
	t.setFillColor(sf::Color(r,g,0));
	t.setPosition(screen_x-180,screen_y-45);
	win.draw(t);

	sprintf(buf, "charger input %2.2f V", latest_pwr.charger_input_mv/1000.0);
	t.setString(buf);
	t.setCharacterSize(14);
	if(cha_voltage < 1.0)
		t.setFillColor(sf::Color(50,50,255));
	else if(cha_voltage < 22.0 || cha_voltage > 27.0)
		t.setFillColor(sf::Color(255,0,0));
	else
		t.setFillColor(sf::Color(0,255,0));

	t.setPosition(screen_x-180,screen_y-20);
	win.draw(t);

	int cur_tot = latest_pwr.pha_charging_current_ma + latest_pwr.phb_charging_current_ma;

	if(cur_tot > 0)
	{
		sprintf(buf, "charging @ %2.2f A", cur_tot/1000.0);
		t.setString(buf);
		t.setCharacterSize(16);
		t.setFillColor(sf::Color(200,110,0));
		t.setPosition(screen_x-130,screen_y-65);
		win.draw(t);
	}

}

int state_is_unsynchronized;

void draw_texts(sf::RenderWindow& win)
{
	sf::Text t;
	char buf[256];
	t.setFont(arial);

	const int bot_box_xs = 400;
	const int bot_box_ys = 63;
	sf::RectangleShape rect(sf::Vector2f( bot_box_xs, bot_box_ys));
	rect.setPosition(screen_x/2 - bot_box_xs/2, screen_y-bot_box_ys-10-30);
	rect.setFillColor(sf::Color(255,255,255,160));
	win.draw(rect);


	sprintf(buf, "robot: x=%d  y=%d  mm  (yaw=%.1f pitch=%.1f roll=%.1f )", (int)cur_x, (int)cur_y, cur_angle, cur_pitch, cur_roll);
	t.setString(buf);
	t.setCharacterSize(14);
	t.setFillColor(sf::Color(0,0,0,160));
	t.setPosition(screen_x/2-bot_box_xs/2+10,screen_y-51-30);
	win.draw(t);

	sprintf(buf, "cursor: x=%d  y=%d  mm", (int)click_x, (int)click_y);
	t.setString(buf);
	t.setCharacterSize(14);
	t.setFillColor(sf::Color(0,0,0, 120));
	t.setPosition(screen_x/2-bot_box_xs/2+10,screen_y-30-30);
	win.draw(t);

	if(state_is_unsynchronized)
		sprintf(buf, "Robot state is unsynchronized...");
	else if(rsync_running)
		sprintf(buf, "Syncing maps...");
	else
		sprintf(buf, "%s", click_mode_names[click_mode]);
	t.setString(buf);
	t.setCharacterSize(17);
	t.setFillColor(sf::Color(0,0,0,130));
	t.setPosition(screen_x/2-bot_box_xs/2+11,screen_y-72-30);
	win.draw(t);
	t.setFillColor(rsync_running?sf::Color(0,190,20,200):click_mode_colors[click_mode]);
	t.setPosition(screen_x/2-bot_box_xs/2+10,screen_y-73-30);
	win.draw(t);

	const float dbg_point_r = 35.0;
	if(show_dbgpoint)
	{


		sf::CircleShape circ(2.0*dbg_point_r/mm_per_pixel);
		circ.setOrigin(2.0*dbg_point_r/mm_per_pixel, 2.0*dbg_point_r/mm_per_pixel);
		circ.setFillColor(sf::Color(dbgpoint_r,dbgpoint_g,dbgpoint_b, 180));
		circ.setOutlineColor(sf::Color(255,255,255,255));

		circ.setPosition((dbgpoint_x+origin_x)/mm_per_pixel,(-1*dbgpoint_y+origin_y)/mm_per_pixel);
		win.draw(circ);
	}

	for(int i = 0; i < num_pers_dbgpoints; i++)
	{

		sf::CircleShape circ(dbg_point_r/mm_per_pixel);
		circ.setOrigin(dbg_point_r/mm_per_pixel, dbg_point_r/mm_per_pixel);
		circ.setFillColor(sf::Color(pers_dbgpoint_r[i],pers_dbgpoint_g[i],pers_dbgpoint_b[i], 120));
		circ.setOutlineColor(sf::Color(0,0,0,150));

		circ.setPosition((pers_dbgpoint_x[i]+origin_x)/mm_per_pixel,(-1*pers_dbgpoint_y[i]+origin_y)/mm_per_pixel);
		win.draw(circ);

	}

}

#define TOF_XS (160)
#define TOF_YS (60)

#define TOF_XS_NARROW (32)
#define TOF_YS_NARROW (44)

#define TOF_NARROW_Y_START (8)
#define TOF_NARROW_X_START (64)
#define N_TOF_SENSORS 10

tof_raw_dist_t latest_tof_dists[N_TOF_SENSORS];
tof_raw_ampl8_t latest_tof_ampls[N_TOF_SENSORS];
tof_raw_ambient8_t latest_tof_ambients[N_TOF_SENSORS];

#define DATA_LOW 65534
#define DATA_OVEREXP 65535

float red_dist  = 0.0;
float blue_dist = 9000.0;

int tof_raw_alpha = 255; //80;

void draw_tof_dist(sf::RenderWindow& win, int xs, int ys, uint16_t* img, int x_on_screen, int y_on_screen, float scale, bool mir_x, bool mir_y, bool rotated)
{
	static uint8_t pix[TOF_XS*TOF_YS*4];

	int yy = (mir_y?(ys-1):0);

	int inx = 0, iny = 0;
	while(1)
	{
		int xx = (mir_x?(xs-1):0);
		while(1)
		{
			int pixval;
			pixval = img[iny*xs+inx];

			if(pixval == DATA_OVEREXP)
			{
				pix[4*(yy*xs+xx)+0] = 160;
				pix[4*(yy*xs+xx)+1] = 0;
				pix[4*(yy*xs+xx)+2] = 160;
			}
			else if(pixval == DATA_LOW)
			{
				pix[4*(yy*xs+xx)+0] = 0;
				pix[4*(yy*xs+xx)+1] = 0;
				pix[4*(yy*xs+xx)+2] = 128;
			}
			else
			{
				float percolor = blue_dist/3.0;

				float mm = pixval;
				mm -= red_dist; if(mm<0.0) mm=0.0;
				float f_r = 1.0 - fabs(mm-0*percolor)/percolor;
				float f_g = 1.0 - fabs(mm-1*percolor)/percolor;
				float f_b = 1.0 - fabs(mm-2*percolor)/percolor;

				int r = f_r*256.0; if(r<0) r=0; else if(r>255) r=255;
				int g = f_g*256.0; if(g<0) g=0; else if(g>255) g=255;
				int b = f_b*256.0; if(b<0) b=0; else if(b>255) b=255;


				pix[4*(yy*xs+xx)+0] = r;
				pix[4*(yy*xs+xx)+1] = g;
				pix[4*(yy*xs+xx)+2] = b;
			}
			pix[4*(yy*xs+xx)+3] = tof_raw_alpha;

			inx++;
			if(inx==xs)
			{
				inx=0; iny++;
			}
			if( (mir_x && --xx<0) || (!mir_x && ++xx>=xs) ) break;
		}
		if( (mir_y && --yy<0) || (!mir_y && ++yy>=ys) ) break;
	}

	sf::Texture t;
	t.create(xs, ys);
	t.setSmooth(false);
	t.update(pix);
	sf::Sprite sprite;
	sprite.setTexture(t);
	sprite.setPosition((float)x_on_screen, (float)y_on_screen);
	sprite.setScale(sf::Vector2f(scale, scale));
	if(rotated)
	{
		sprite.setRotation(90.0);
		sprite.setPosition(x_on_screen+ys*scale, y_on_screen);
	}
	else
	{
		sprite.setPosition(x_on_screen, y_on_screen);
	}
	win.draw(sprite);
}

void draw_tof_ampl(sf::RenderWindow& win, int xs, int ys, uint8_t* img, int x_on_screen, int y_on_screen, float scale, bool mir_x, bool mir_y, bool rotated)
{
	static uint8_t pix[TOF_XS*TOF_YS*4];

	int yy = (mir_y?(ys-1):0);

	int inx = 0, iny = 0;
	while(1)
	{
		int xx = (mir_x?(xs-1):0);
		while(1)
		{
			int pixval;
			pixval = img[iny*xs+inx];
			if(dbg_boost) { pixval*=4; pixval+=16; if(pixval>255) pixval=255;}

			pix[4*(yy*xs+xx)+0] = pixval;
			pix[4*(yy*xs+xx)+1] = pixval;
			pix[4*(yy*xs+xx)+2] = pixval;
			pix[4*(yy*xs+xx)+3] = tof_raw_alpha;

			inx++;
			if(inx==xs)
			{
				inx=0; iny++;
			}
			if( (mir_x && --xx<0) || (!mir_x && ++xx>=xs) ) break;
		}
		if( (mir_y && --yy<0) || (!mir_y && ++yy>=ys) ) break;
	}

	sf::Texture t;
	t.create(xs, ys);
	t.setSmooth(false);
	t.update(pix);
	sf::Sprite sprite;
	sprite.setTexture(t);
	sprite.setPosition((float)x_on_screen, (float)y_on_screen);
	sprite.setScale(sf::Vector2f(scale, scale));
	if(rotated)
	{
		sprite.setRotation(90.0);
		sprite.setPosition(x_on_screen+ys*scale, y_on_screen);
	}
	else
	{
		sprite.setPosition(x_on_screen, y_on_screen);
	}
	win.draw(sprite);
}


void draw_tof_dist_together(sf::RenderWindow& win, uint16_t* img, int x_on_screen, int y_on_screen, float scale, bool mir_x, bool mir_y, bool rotated, uint16_t* img_narrow)
{
	static uint8_t pix[TOF_XS*TOF_YS*4];

	int yy = (mir_y?(TOF_YS-1):0);

	int inx = 0, iny = 0;
	while(1)
	{
		int xx = (mir_x?(TOF_XS-1):0);
		while(1)
		{
			int pixval;

			if(img_narrow != NULL &&
			   inx >= TOF_NARROW_X_START && inx < TOF_NARROW_X_START+TOF_XS_NARROW &&
			   iny >= TOF_NARROW_Y_START && iny < TOF_NARROW_Y_START+TOF_YS_NARROW)
			{
				pixval = img_narrow[(iny-TOF_NARROW_Y_START)*TOF_XS_NARROW+(inx-TOF_NARROW_X_START)];
			}
			else
			{
				pixval = img[iny*TOF_XS+inx];
			}

			if(pixval == DATA_OVEREXP)
			{
				pix[4*(yy*TOF_XS+xx)+0] = 160;
				pix[4*(yy*TOF_XS+xx)+1] = 0;
				pix[4*(yy*TOF_XS+xx)+2] = 160;
			}
			else if(pixval == DATA_LOW)
			{
				pix[4*(yy*TOF_XS+xx)+0] = 0;
				pix[4*(yy*TOF_XS+xx)+1] = 0;
				pix[4*(yy*TOF_XS+xx)+2] = 128;
			}
			else
			{
				float percolor = blue_dist/3.0;

				float mm = pixval;
				float f_r = 1.0 - fabs(mm-0*percolor)/percolor;
				float f_g = 1.0 - fabs(mm-1*percolor)/percolor;
				float f_b = 1.0 - fabs(mm-2*percolor)/percolor;

				int r = f_r*256.0; if(r<0) r=0; else if(r>255) r=255;
				int g = f_g*256.0; if(g<0) g=0; else if(g>255) g=255;
				int b = f_b*256.0; if(b<0) b=0; else if(b>255) b=255;


				pix[4*(yy*TOF_XS+xx)+0] = r;
				pix[4*(yy*TOF_XS+xx)+1] = g;
				pix[4*(yy*TOF_XS+xx)+2] = b;
			}
			pix[4*(yy*TOF_XS+xx)+3] = 255;

			inx++;
			if(inx==TOF_XS)
			{
				inx=0; iny++;
			}
			if( (mir_x && --xx<0) || (!mir_x && ++xx>=TOF_XS) ) break;
		}
		if( (mir_y && --yy<0) || (!mir_y && ++yy>=TOF_YS) ) break;
	}

	sf::Texture t;
	t.create(TOF_XS, TOF_YS);
	t.setSmooth(false);
	t.update(pix);
	sf::Sprite sprite;
	sprite.setTexture(t);
	sprite.setPosition((float)x_on_screen, (float)y_on_screen);
	sprite.setScale(sf::Vector2f(scale, scale));
	if(rotated)
	{
		sprite.setRotation(90.0);
		sprite.setPosition(x_on_screen+TOF_YS*scale, y_on_screen);
	}
	else
	{
		sprite.setPosition(x_on_screen, y_on_screen);
	}
	win.draw(sprite);
}

void draw_tof_ampl_together(sf::RenderWindow& win, uint8_t* img, int x_on_screen, int y_on_screen, float scale, bool mir_x, bool mir_y, bool rotated, uint8_t* img_narrow)
{
	static uint8_t pix[TOF_XS*TOF_YS*4];

	int yy = (mir_y?(TOF_YS-1):0);

	int inx = 0, iny = 0;

	while(1)
	{

		int xx = (mir_x?(TOF_XS-1):0);
		while(1)
		{

			int pixval;
			bool isnarrow = false;
			if(img_narrow != NULL &&
			   inx >= TOF_NARROW_X_START && inx < TOF_NARROW_X_START+TOF_XS_NARROW &&
			   iny >= TOF_NARROW_Y_START && iny < TOF_NARROW_Y_START+TOF_YS_NARROW)
			{
				pixval = img_narrow[(iny-TOF_NARROW_Y_START)*TOF_XS_NARROW+(inx-TOF_NARROW_X_START)];
				isnarrow = true;
			}
			else
			{
				pixval = img[iny*TOF_XS+inx];
			}

			if(isnarrow)
			{
				pix[4*(yy*TOF_XS+xx)+0] = pixval;
				pix[4*(yy*TOF_XS+xx)+1] = pixval;
				pix[4*(yy*TOF_XS+xx)+2] = pixval>>1;
			}
			else
			{
				pix[4*(yy*TOF_XS+xx)+0] = pixval;
				pix[4*(yy*TOF_XS+xx)+1] = pixval>>1;
				pix[4*(yy*TOF_XS+xx)+2] = pixval;
			}
			pix[4*(yy*TOF_XS+xx)+3] = 255;

			inx++;
			if(inx==TOF_XS)
			{
				inx=0; iny++;
			}

			if( (mir_x && --xx<0) || (!mir_x && ++xx>=TOF_XS) ) break;
		}

		if( (mir_y && --yy<0) || (!mir_y && ++yy>=TOF_YS) ) break;
	}

	sf::Texture t;
	t.create(TOF_XS, TOF_YS);
	t.setSmooth(false);
	t.update(pix);
	sf::Sprite sprite;
	sprite.setTexture(t);
	sprite.setPosition((float)x_on_screen, (float)y_on_screen);
	sprite.setScale(sf::Vector2f(scale, scale));
	if(rotated)
	{
		sprite.setRotation(90.0);
		sprite.setPosition(x_on_screen+TOF_YS*scale, y_on_screen);
	}
	else
	{
		sprite.setPosition(x_on_screen, y_on_screen);
	}
	win.draw(sprite);
}

bool draw_ambients = false;

void draw_tof_raws(sf::RenderWindow& win)
{
	float scale = 2.5;

	static const int order[10] = { 5,4,3,2,1,0,9,8,7,6};
//	static const int order[10] = { 0,1,2,3,4,5,6,7,8,9};

//	sf::RectangleShape rect(sf::Vector2f( 10*(scale*60.0+4)  + 6, scale*TOF_XS_NARROW+(TOF_XS+TOF_XS_NARROW)*scale+6+scale*TOF_XS +17  +6));
//	rect.setPosition(20-3, 20-20);
//	rect.setFillColor(sf::Color(255,255,255,tof_raw_alpha));
//	win.draw(rect);

//	for(int ii=0; ii<10; ii++)
	for(int ii=0; ii<1; ii++)
	{
		bool mir_x = false;
		bool mir_y = false;
		bool rotated = true;

	
		int i = order[ii];

		switch(latest_tof_dists[i].sensor_orientation)
		{
			case 1:
			rotated = true;
			mir_x = true;
			mir_y = true;
			break;
			case 2:
			rotated = true;
			mir_x = false;
			mir_y = false;
			break;
			case 3:
			rotated = false;
			mir_x = false;
			mir_y = false;
			break;
			case 4:
			rotated = false;
			mir_x = false;
			mir_y = false;

			break;
			default:
//			if(i == 0 || i == 1)
//				printf("Illegal sensor orientation %d\n", latest_tof_dists[i].sensor_orientation);
			break;
		}


		#if TOGETHER
			draw_tof_dist_together(win, latest_tof_dists[i].dist, 20+scale*60.0+4, ii*scale*160.0+20, 20, scale, mir_x, mir_y, rotated, latest_tof_dists[i].dist_narrow);
			draw_tof_ampl_together(win, latest_tof_ampls[i].ampl, 20+scale*60.0+4, ii*scale*160.0+20+2, scale, mir_x, mir_y, rotated, latest_tof_ampls[i].narrow);
		#else
			draw_tof_dist(win, TOF_XS_NARROW, TOF_YS_NARROW, latest_tof_dists[i].dist_narrow,
				20+scale*((TOF_YS-TOF_YS_NARROW)/2.0)+ii*(scale*60.0+4), 20, scale, mir_x, mir_y, rotated);
			draw_tof_dist(win, TOF_XS, TOF_YS, latest_tof_dists[i].dist,
				20+ii*(scale*60.0+4), 20+scale*TOF_XS_NARROW+2, scale, mir_x, mir_y, rotated);

			draw_tof_ampl(win, TOF_XS_NARROW, TOF_YS_NARROW, latest_tof_ampls[i].ampl_narrow, 
				20+scale*((TOF_YS-TOF_YS_NARROW)/2.0)+ii*(scale*60.0+4), 20+(TOF_XS+TOF_XS_NARROW)*scale+4, scale, mir_x, mir_y, rotated);
			draw_tof_ampl(win, TOF_XS, TOF_YS, draw_ambients?(latest_tof_ambients[i].ambient):(latest_tof_ampls[i].ampl),
				20+ii*(scale*60.0+4), 20+scale*TOF_XS_NARROW+(TOF_XS+TOF_XS_NARROW)*scale+6, scale, mir_x, mir_y, rotated);
		

			sf::Text te;
			char tbuf[32];
			sprintf(tbuf, "%u (%u%%)", latest_tof_dists[i].narrow_stray_estimate_adc, (100*latest_tof_dists[i].narrow_stray_estimate_adc+1)/16384);
			te.setFont(arial);
			te.setFillColor(sf::Color(255,255,255,tof_raw_alpha));
			te.setString(tbuf);
			te.setCharacterSize(10);
			te.setPosition(20+scale*((TOF_YS-TOF_YS_NARROW)/2.0) +ii*(scale*60.0+4), 20);
			win.draw(te);

			sprintf(tbuf, "%u (%u%%)", latest_tof_dists[i].wide_stray_estimate_adc, (100*latest_tof_dists[i].wide_stray_estimate_adc+1)/16384);
			te.setString(tbuf);
			te.setPosition(20+scale*((TOF_YS-TOF_YS_NARROW)/2.0) +ii*(scale*60.0+4), 20+scale*TOF_XS_NARROW+2);
			win.draw(te);

			sprintf(tbuf, "TOF%u", i);
			te.setString(tbuf);
			te.setCharacterSize(14);
			te.setFillColor(sf::Color(0,0,0,tof_raw_alpha));
			te.setPosition(20 + scale*20.0 +ii*(scale*60.0+4), 20-17);
			win.draw(te);
			te.setFillColor(sf::Color(255,255,255,tof_raw_alpha));
			te.setPosition(20 + scale*20.0 +ii*(scale*60.0+4)-1, 20-17-1);
			win.draw(te);

		#endif
	}
}

void draw_picture(sf::RenderWindow& win)
{
	if(pict_id < 0 || pict_xs < 1 || pict_ys < 1 || pict_xs > 500 || pict_ys > 500)
	{
		return;
	}

	static uint8_t pixels[500*500*4];
/*	if(pict_bpp == 1)
	{
		for(int i=0; i<pict_xs*pict_ys; i++)
		{
			pixels[4*i+0] = pict_data[i];
			pixels[4*i+1] = pict_data[i];
			pixels[4*i+2] = pict_data[i];
			pixels[4*i+3] = 255;
		}
	}
*/

	sf::Vector2i m = sf::Mouse::getPosition(win);


	float scale = 8.0;

	sf::Texture t;
	t.create(pict_xs, pict_ys);
	t.setSmooth(false);
	sf::Sprite sprite;

	float mx = m.x;
	float my = m.y;

	int pic_x = 15, pic_y;
#ifdef TOF_DEV
	if(pict_id==100)
	{
		pic_y = 15+scale*0*pict_ys+10;
		mx -= 15; my -= 15+scale*pict_ys+10;
	}
	else if(pict_id==101 || pict_id==110)
	{
		pic_y = 15+scale*1*pict_ys+20;
		mx -= 15; my -= 15+2*scale*pict_ys+20;
	}
	else
	{
#endif
		pic_y = 15;
		mx -= 15; my -= 15;
#ifdef TOF_DEV
	}
#endif
	sprite.setPosition(pic_x, pic_y);

	mx /= scale;
	my /= scale;

	int process_as_dist = 0;

	if(pict_id == 1 || pict_id == 2 || pict_id == 3 || pict_id == 4) // ignore map
	{
		for(int i=0; i<pict_xs*pict_ys; i++)
		{
			if(pict_data[i])
			{
				pixels[4*i+0] = 255;
				pixels[4*i+1] = 128;
				pixels[4*i+2] = 50;
			}
			else
			{
				pixels[4*i+0] = 0;
				pixels[4*i+1] = 0;
				pixels[4*i+2] = 0;
			}
			pixels[4*i+3] = 255;
		}
	}
	else if(pict_id == 101 || pict_id == 5 || pict_id == 7 || pict_id == 9) // amplitudes
	{
		for(int i=0; i<pict_xs*pict_ys; i++)
		{
			int val = pict_data[i];
			if(dbg_boost) { val*=4; if(val>255) val=255;}
			pixels[4*i+0] = val;
			pixels[4*i+1] = val;
			pixels[4*i+2] = val;
			pixels[4*i+3] = 255;
		}
	}
	else if(pict_id == 100 || pict_id == 6 || pict_id == 8 || pict_id == 10 || pict_id == 110) // distances
	{
		process_as_dist = 1;
		for(int i=0; i<pict_xs*pict_ys; i++)
		{

			int dist = ((uint16_t*)pict_data)[i];
			int r, g, b;

			if(dist == 0)
			{
				r = 0; g = 0; b = 50;
			}
			else if(dist > 6000)
			{
				r = 200; g = 200; b = 200;
			}
			else
			{
				float blue_dist=6000;
				float percolor = blue_dist/3.0;

				float mm = dist;
				float f_r = 1.0 - fabs(mm-0*percolor)/percolor;
				float f_g = 1.0 - fabs(mm-1*percolor)/percolor;
				float f_b = 1.0 - fabs(mm-2*percolor)/percolor;

				r = f_r*256.0; if(r<0) r=0; else if(r>255) r=255;
				g = f_g*256.0; if(g<0) g=0; else if(g>255) g=255;
				b = f_b*256.0; if(b<0) b=0; else if(b>255) b=255;

			}

			pixels[4*i+0] = r;
			pixels[4*i+1] = g;
			pixels[4*i+2] = b;
			pixels[4*i+3] = 255;

		}
	}

	sf::Text te2;
	char tbuf2[16];
	if(mx >= 0 && mx < pict_xs && my >= 0 && my <= pict_ys)
	{
		int imy = (int)my;
		int imx = (int)mx;
		pixels[4*(imy*pict_xs+imx)+0] = 128;
		pixels[4*(imy*pict_xs+imx)+1] = 255;
		pixels[4*(imy*pict_xs+imx)+2] = 255;
		pixels[4*(imy*pict_xs+imx)+3] = 255;

		int val;
		if(process_as_dist)
			val = ((uint16_t*)pict_data)[imy*pict_xs+imx];
		else
			val = pict_data[imy*pict_xs+imx];


		sprintf(tbuf2, "(%d,%d)=%d", imx, imy, val);
		te2.setFont(arial);
		te2.setFillColor(sf::Color(0,0,0,255));
		te2.setString(tbuf2);
		te2.setCharacterSize(12);
		te2.setPosition(pic_x+scale*mx+8, pic_y+scale*my-5);

	}

	t.update(pixels);
	sprite.setTexture(t);
	sprite.setScale(sf::Vector2f(scale, scale));
	win.draw(sprite);
	win.draw(te2);
	te2.setFillColor(sf::Color(255,255,255,255));
	te2.setPosition(pic_x+scale*mx+8-1, pic_y+scale*my-5-1);
	win.draw(te2);

	{
		sf::Text te;
		char tbuf[16];
		sprintf(tbuf, "ID=%d", pict_id);
		te.setFont(arial);
		te.setFillColor(sf::Color(0,0,0,255));
		te.setString(tbuf);
		te.setCharacterSize(9);
		te.setPosition(20,2);
		win.draw(te);
	}


#ifdef TOF_DEV
	pict_id = -1;
#endif

}


void draw_map(sf::RenderWindow& win)
{
	for(int x = 0; x < MAP_W; x++)
	{
		for(int y = 0; y < MAP_W; y++)
		{
			if(world.pages[x][y])
			{
				int startx = -MAP_MIDDLE_UNIT*MAP_UNIT_W + x*MAP_PAGE_W*MAP_UNIT_W + origin_x;
				int starty = -1*(-MAP_MIDDLE_UNIT*MAP_UNIT_W + (y+1)*MAP_PAGE_W*MAP_UNIT_W) + origin_y;

//				if(x==127 && y==128)
					draw_page(win, world.pages[x][y], startx, starty);
			}
		}
	}
}


void draw_robot(sf::RenderWindow& win)
{
	sf::ConvexShape r(7);
	r.setPoint(0, sf::Vector2f(0,0));
	r.setPoint(1, sf::Vector2f(0,robot_ys/mm_per_pixel));
	r.setPoint(2, sf::Vector2f(robot_xs/mm_per_pixel,robot_ys/mm_per_pixel));
	r.setPoint(3, sf::Vector2f(robot_xs/mm_per_pixel,0.70*robot_ys/mm_per_pixel));
	r.setPoint(4, sf::Vector2f((robot_xs+0.2*robot_ys)/mm_per_pixel,0.5*robot_ys/mm_per_pixel));
	r.setPoint(5, sf::Vector2f(robot_xs/mm_per_pixel,0.30*robot_ys/mm_per_pixel));
	r.setPoint(6, sf::Vector2f(robot_xs/mm_per_pixel,0));

	r.setOrigin((0.5*robot_xs+lidar_xoffs)/mm_per_pixel,(0.5*robot_ys+lidar_yoffs)/mm_per_pixel);

	r.setFillColor(sf::Color(200,90,50,160));

	r.setRotation(-1.0*cur_angle);
	r.setPosition((cur_x+origin_x)/mm_per_pixel,(-1.0*cur_y+origin_y)/mm_per_pixel);

	win.draw(r);

	if(static_cast<int>(dest_type) > -1 && static_cast<int>(dest_type) < 8)
	{
		const float robot_mark_radius = 200.0;
		const float robot_mark_radius2 = 40.0;
		sf::CircleShape circ(robot_mark_radius/mm_per_pixel);
		circ.setOrigin(robot_mark_radius/(mm_per_pixel), robot_mark_radius/(mm_per_pixel));
		circ.setFillColor(click_mode_colors[static_cast<int>(dest_type)]);
		circ.setOutlineThickness(1.0);
		circ.setOutlineColor(sf::Color(0,0,0,100));
		circ.setPosition((dest_x+origin_x)/mm_per_pixel,(-1*dest_y+origin_y)/mm_per_pixel);
		win.draw(circ);
		sf::CircleShape circ2(robot_mark_radius2/mm_per_pixel);
		circ2.setOrigin(robot_mark_radius2/(mm_per_pixel), robot_mark_radius2/(mm_per_pixel));
		circ2.setFillColor(click_mode_colors[static_cast<int>(dest_type)]);
		circ2.setOutlineThickness(1.0);
		circ2.setOutlineColor(sf::Color(0,0,0,150));
		circ2.setPosition((dest_x+origin_x)/mm_per_pixel,(-1*dest_y+origin_y)/mm_per_pixel);
		win.draw(circ2);

		if(dest_type == MODE_ADDCONSTRAINT || dest_type == MODE_REMCONSTRAINT)
			dest_type = MODE_INVALID;
	}
}

/*typedef struct
{
	int32_t ang; // int32_t range --> -180..+180 deg; let it overflow freely. 1 unit = 83.81903171539 ndeg
	int32_t x;   // in mm
	int32_t y;
} pos_t;

typedef struct
{
	int valid;
	int32_t x;   // in mm
	int32_t y;
} point_t;
*/

#define MAX_LIDAR_POINTS 720

typedef struct
{
	pos_t robot_pos;
	int n_points;
	point_t scan[MAX_LIDAR_POINTS];
} client_lidar_scan_t;

typedef struct
{
	pos_t robot_pos;
	int xsamples;
	int ysamples;
	int unit_size;
	int8_t data[256*256];
} client_tof3d_hmap_t;

client_lidar_scan_t lidar;
#define SONAR_POINTS 6
sonar_point_t sonar[SONAR_POINTS];
static int sonar_wr = 0;


client_tof3d_hmap_t hmap;


#define HMAP_ALPHA 255UL

#define TOF3D_WALL           8 
#define TOF3D_BIG_ITEM       7 
#define TOF3D_LOW_CEILING    6 
#define TOF3D_BIG_DROP       5
#define TOF3D_SMALL_ITEM     4 
#define TOF3D_SMALL_DROP     3
#define TOF3D_THRESHOLD      2   
#define TOF3D_FLOOR          1
#define TOF3D_UNSEEN         0


static const uint32_t hmap_colors[9] = {
/* 0 UNSEEN     */ RGBA32(128UL,128UL,128UL, 0),
/* 1 FLOOR      */ RGBA32(150UL,255UL,150UL, HMAP_ALPHA/2),
/* 2 THRESHOLD  */ RGBA32(  0UL,200UL,200UL, HMAP_ALPHA),
/* 3 SMALL_DROP */ RGBA32( 50UL,  0UL,200UL, HMAP_ALPHA),
/* 4 SMALL_ITEM */ RGBA32(  0UL,255UL,  0UL, HMAP_ALPHA),
/* 5 BIG_DROP   */ RGBA32(220UL,  0UL,220UL, HMAP_ALPHA),
/* 6 LOW_CEILING*/ RGBA32(255UL,  0UL, 50UL, HMAP_ALPHA),
/* 7 BIG_ITEM   */ RGBA32(220UL,100UL,  0UL, HMAP_ALPHA),
/* 8 WALL       */ RGBA32(200UL,200UL,  0UL, HMAP_ALPHA)
};


static int hmap_alpha_mult = 255;

void draw_tof3d_hmap(sf::RenderWindow& win, client_tof3d_hmap_t* hm)
{
	if(hm->xsamples == 0 || hm->ysamples == 0)
		return;

	if(hm->xsamples > 256 || hm->ysamples > 256)
	{
		printf("Invalid hmap size\n");
		return;
	}

	static uint32_t pixels[256*256];


	float scale = (float)hm->unit_size/mm_per_pixel;

	for(int sy=0; sy < hm->ysamples; sy++)
	{
		for(int sx=0; sx < hm->xsamples; sx++)
		{
			uint8_t val = hm->data[sy*hm->xsamples+sx];
			if(val > 8)
			{
				printf("draw_tof3d_hmap() invalid val %d at (%d, %d)\n", val, sx, sy);
				continue;
			}
			pixels[sy*hm->xsamples+sx] = hmap_colors[val];
		}
	}

	float ang = hm->robot_pos.ang;

	sf::Texture t;
	t.create(hm->xsamples, hm->ysamples);
	t.setSmooth(false);
	t.update((uint8_t*)pixels);
	sf::Sprite sprite;
	sprite.setTexture(t);
	sprite.setOrigin(hm->xsamples/2.0, hm->ysamples/2.0);
	sprite.setRotation(ang);
	sprite.setPosition((hm->robot_pos.x+origin_x)/mm_per_pixel, (-1*hm->robot_pos.y+origin_y)/mm_per_pixel);
	sprite.setScale(sf::Vector2f(scale, scale));
	sprite.setColor(sf::Color(255,255,255,hmap_alpha_mult));
	win.draw(sprite);

}

#define VOX_SEG_XS 100
#define VOX_SEG_YS 100
#define VOX_HIRES_UNIT 50 // mm
#define VOX_LORES_UNIT 100 // mm

typedef struct __attribute__((packed))
{
	mcu_multi_voxel_map_t msgs[4];
} full_voxel_map_t;

full_voxel_map_t latest_voxmap;



typedef struct
{
	int xmin;
	int xmax;
	int ymin;
	int ymax;
	int reso;
} seg_limits_t;

const seg_limits_t seg_lims[12] =
{
	{	// Seg 0
		0, VOX_SEG_XS*VOX_HIRES_UNIT-1,
		0, VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_HIRES_UNIT
	},

	{	// Seg 1
		-VOX_SEG_XS*VOX_HIRES_UNIT, -1,
		0, VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_HIRES_UNIT
	},

	{	// Seg 2
		-VOX_SEG_XS*VOX_HIRES_UNIT, -1,
		-VOX_SEG_YS*VOX_HIRES_UNIT, -1,
		VOX_HIRES_UNIT
	},

	{	// Seg 3
		0, VOX_SEG_XS*VOX_HIRES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT, -1,
		VOX_HIRES_UNIT
	},

	{	// Seg 4
		VOX_SEG_XS*VOX_HIRES_UNIT, VOX_SEG_XS*VOX_HIRES_UNIT + VOX_SEG_XS*VOX_LORES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT, VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 5
		VOX_SEG_XS*VOX_HIRES_UNIT, VOX_SEG_XS*VOX_HIRES_UNIT + VOX_SEG_XS*VOX_LORES_UNIT-1,
		VOX_SEG_YS*VOX_HIRES_UNIT, VOX_SEG_YS*VOX_HIRES_UNIT + VOX_SEG_YS*VOX_LORES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 6
		-VOX_SEG_XS*VOX_HIRES_UNIT, VOX_SEG_XS*VOX_HIRES_UNIT-1,
		VOX_SEG_YS*VOX_HIRES_UNIT, VOX_SEG_YS*VOX_HIRES_UNIT + VOX_SEG_YS*VOX_LORES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 7
		-VOX_SEG_XS*VOX_HIRES_UNIT - VOX_SEG_XS*VOX_LORES_UNIT, -VOX_SEG_XS*VOX_HIRES_UNIT-1,
		VOX_SEG_YS*VOX_HIRES_UNIT, VOX_SEG_YS*VOX_HIRES_UNIT + VOX_SEG_YS*VOX_LORES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 8
		-VOX_SEG_XS*VOX_HIRES_UNIT - VOX_SEG_XS*VOX_LORES_UNIT, -VOX_SEG_XS*VOX_HIRES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT, VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 9
		-VOX_SEG_XS*VOX_HIRES_UNIT - VOX_SEG_XS*VOX_LORES_UNIT, -VOX_SEG_XS*VOX_HIRES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT-VOX_SEG_YS*VOX_LORES_UNIT, -VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 10
		-VOX_SEG_XS*VOX_HIRES_UNIT, VOX_SEG_XS*VOX_HIRES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT-VOX_SEG_YS*VOX_LORES_UNIT, -VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_LORES_UNIT
	},

	{	// Seg 11
		VOX_SEG_XS*VOX_HIRES_UNIT, VOX_SEG_XS*VOX_HIRES_UNIT + VOX_SEG_XS*VOX_LORES_UNIT-1,
		-VOX_SEG_YS*VOX_HIRES_UNIT-VOX_SEG_YS*VOX_LORES_UNIT, -VOX_SEG_YS*VOX_HIRES_UNIT-1,
		VOX_LORES_UNIT
	}

};


void draw_voxmap_seg(sf::RenderWindow& win, uint16_t* map, int xoffs_mm, int yoffs_mm, int reso)
{
	const int xs = 100;
	const int ys = 100;

	static uint32_t pixels[100*100];

	float scale = (float)reso/mm_per_pixel;

	for(int yy=0; yy < ys; yy++)
	{
		for(int xx=0; xx < xs; xx++)
		{
			uint16_t val = map[yy*xs+xx];
			/* really one at once:
			if(val&(1<<cur_slice))
				pixels[yy*xs+xx] = voxmap_colors[cur_slice];
			else
				pixels[yy*xs+xx] = voxmap_blank_color;
			*/

			pixels[(ys-1-yy)*xs+xx] = voxmap_blank_color;
			for(int slice = 0; slice<cur_slice; slice++)
			{
				if(val&(1<<slice))
					pixels[(ys-1-yy)*xs+xx] = voxmap_colors[slice];
			}

		}
	}

	float ang = 0;

//	printf("xoffs_mm = %d,  yoffs_mm = %d,  reso = %d\n", xoffs_mm, yoffs_mm, reso);

	sf::Texture t;
	t.create(xs,ys);
	t.setSmooth(false);
	t.update((uint8_t*)pixels);
	sf::Sprite sprite;
	sprite.setTexture(t);
	sprite.setOrigin(0, ys-0);
	sprite.setRotation(ang);
	sprite.setPosition((xoffs_mm+origin_x)/mm_per_pixel, (-1*yoffs_mm+origin_y)/mm_per_pixel);
	sprite.setScale(sf::Vector2f(scale, scale));
	sprite.setColor(sf::Color(255,255,255,voxmap_alpha));
	win.draw(sprite);

}

bool animate_slice = false;
void draw_voxmap(sf::RenderWindow& win)
{
	for(int seg=0; seg<12; seg++)
	{
		draw_voxmap_seg(win, latest_voxmap.msgs[seg/3].maps[seg%3],seg_lims[seg].xmin+latest_voxmap.msgs[seg/3].ref_x, seg_lims[seg].ymin+latest_voxmap.msgs[seg/3].ref_y, (seg<4)?50:100);
	}


	static int cnt = 0;
	cnt++;

	if(animate_slice)
	{
		if((cur_slice < 15 && cnt >= 2) || (cnt >= 10))
		{
			cnt = 0;
			cur_slice++;
			if(cur_slice > 15) cur_slice = 1;
		}
	}
	else
		cur_slice = 15;
}

void draw_lidar(sf::RenderWindow& win, client_lidar_scan_t* lid)
{
	for(int i=0; i < lid->n_points; i++)
	{
		sf::RectangleShape rect(sf::Vector2f(3,3));
		rect.setOrigin(1.5,1.5);
		rect.setPosition((lid->scan[i].x+origin_x)/mm_per_pixel, (-1*lid->scan[i].y+origin_y)/mm_per_pixel);
		rect.setFillColor(sf::Color(255, 0, 0, 100));
		win.draw(rect);
	}
}

void draw_sonar(sf::RenderWindow& win)
{
	for(int i=0; i < SONAR_POINTS; i++)
	{
		int c = sonar[i].c;
		if(c==0) continue;

		sf::RectangleShape rect(sf::Vector2f(6,6));
		rect.setOrigin(3,3);
		rect.setPosition((sonar[i].x+origin_x)/mm_per_pixel, (-1*sonar[i].y+origin_y)/mm_per_pixel);
		rect.setFillColor(sf::Color(50,50,200));
		win.draw(rect);

		#if SONAR_POINTS < 10

		sf::Text t;
		char tbuf[16];
		sprintf(tbuf, "%d", sonar[i].z);
		t.setFont(arial);
		t.setFillColor(sf::Color(0,0,0,255));
		t.setString(tbuf);
		t.setCharacterSize(9);
		t.setPosition((sonar[i].x+origin_x)/mm_per_pixel, (-1*sonar[i].y+origin_y)/mm_per_pixel);
		win.draw(t);

		#endif
	}
}


sf::IpAddress serv_ip;
unsigned short serv_port;

sf::TcpSocket tcpsock;


void speedlimit_msg(uint8_t limit)
{
	uint8_t test[8] = {63 /*SPEEDLIM*/, 0, 5, limit, limit, limit, 40, 40};

	if(tcpsock.send(test, 8) != sf::Socket::Done)
	{
		printf("Send error\n");
	}
}


void go_charge_msg(uint8_t params)
{
	uint8_t test[4] = {57, 0, 1, params};

	if(tcpsock.send(test, 4) != sf::Socket::Done)
	{
		printf("Send error\n");
	}
}

void maintenance_msg(int restart_mode)
{
	const int size = 1+2+4+4;
	uint8_t test[size];
	test[0] = 62;
	test[1] = ((size-3)&0xff00)>>8;
	test[2] = (size-3)&0xff;
	I32TOBUF(0x12345678, test, 3);
	I32TOBUF(restart_mode, test, 7);

	if(tcpsock.send(test, size) != sf::Socket::Done)
	{
		printf("Send error\n");
	}
}

state_vect_t received_state_vect;
state_vect_t state_vect_to_send;

#define SENDBUF_SIZE (128*1024)

void send(int msgid, int paylen, uint8_t* buf)
{
	static uint8_t sendbuf[SENDBUF_SIZE];

	if(paylen >= SENDBUF_SIZE-5)
	{
		printf("Message too long to be sent.\n");
		return;
	}

	sendbuf[0] = (msgid&0xff00)>>8;
	sendbuf[1] = (msgid&0x00ff)>>0;
	sendbuf[2] = (paylen&0x00ff0000)>>16;
	sendbuf[3] = (paylen&0x0000ff00)>>8;
	sendbuf[4] = (paylen&0x000000ff)>>0;
	memcpy(&sendbuf[5], buf, paylen);

	if(tcpsock.send(sendbuf, 5+paylen) != sf::Socket::Done)
	{
		printf("Send error\n");
	}
}

void mode_msg(uint8_t mode)
{
	uint8_t test[1] = {mode};

	send(358, 1, test);
}


#define NUM_DECORS 8

info_state_t cur_info_state = INFO_STATE_UNDEF;

static uint8_t* rxbuf;

#define MAX_ACCEPTED_MSG_PAYLEN 16777215

int tof_raws_came = 999;

void print_test_msg1(void* m)
{
}
void print_test_msg2(void* m)
{
}
void print_test_msg3(void* m)
{
}


void print_pwr_status(void* m)
{
	memcpy(&latest_pwr, m, sizeof latest_pwr);
//	pwr_status_t* mm = m;
}
void print_tof_raw_dist(void* m)
{
	tof_raw_dist_t* mm = m;
	uint8_t idx = mm->sensor_idx;
	if(idx >= N_TOF_SENSORS)
	{
		printf("Invalid tof sensor idx=%u\n", idx);
		return;
	}

//	printf("IDX = %d, ORIEN = %d\n", mm->sensor_idx, mm->sensor_orientation);
	memcpy(&latest_tof_dists[idx], m, sizeof latest_tof_dists[0]);
	tof_raws_came = 0;
}
void print_tof_raw_ampl8(void* m)
{
	tof_raw_ampl8_t* mm = m;
	uint8_t idx = mm->sensor_idx;
	if(idx >= N_TOF_SENSORS)
	{
		printf("Invalid tof sensor idx=%u\n", idx);
		return;
	}

	memcpy(&latest_tof_ampls[idx], m, sizeof latest_tof_ampls[0]);
	tof_raws_came = 0;

}

void print_tof_raw_ambient8(void* m)
{
	tof_raw_ampl8_t* mm = m;
	uint8_t idx = mm->sensor_idx;
	if(idx >= N_TOF_SENSORS)
	{
		printf("Invalid tof sensor idx=%u\n", idx);
		return;
	}

	memcpy(&latest_tof_ambients[idx], m, sizeof latest_tof_ambients[0]);
	tof_raws_came = 0;

}

void print_tof_diagnostics(void* m)
{
}

void print_tof_raw_img(void* m)
{
}

void print_hw_pose(void* m)
{
	hw_pose_t *mm = m;

//	printf("HW pose  ang=%5.1f deg  x=%8d mm  y=%8d mm\n", ANG32TOFDEG(mm->ang), mm->x, mm->y);

	cur_angle = ANG32TOFDEG(mm->ang);
	cur_pitch = ANG_I32TOFDEG(mm->pitch);
	cur_roll = ANG_I32TOFDEG(mm->roll);
	cur_x = mm->x;
	cur_y = mm->y;
}

drive_diag_t latest_drive_diag;

void print_drive_diag(void* m)
{
	drive_diag_t *mm = m;

	memcpy(&latest_drive_diag, m, sizeof latest_drive_diag);
}

void print_mcu_voxel_map(void* m)
{
	printf("WARN: mcu_voxel_map not supported anymore, use mcu_multi_voxel_map\n");
/*	mcu_voxel_map_t *mm = m;

	int id = mm->block_id;

	printf("Voxel map block %u  z_step %u  base_z %d, running_cnt=%u, ref (%d,%d)\n", id, mm->z_step, mm->base_z, mm->running_cnt, mm->ref_x, mm->ref_y);

	if(id < 0 || id > 12)
	{
		printf("Invalid vox map block id %d\n", id);
		return;
	}
	if(id == 0)
	{
		latest_voxmap.ref_x = mm->ref_x;
		latest_voxmap.ref_y = mm->ref_y;
		latest_voxmap.running_cnt = mm->running_cnt;
	}
	else
	{
		if(mm->running_cnt != latest_voxmap.running_cnt)
		{
			printf("WARNING: running cnt changed, skipped voxmap seg 0?\n");
			return;
		}
	}

	memcpy(latest_voxmap.segs[id], mm->map, sizeof(latest_voxmap.segs[0]));
*/
}

void print_mcu_multi_voxel_map(void* m)
{
	mcu_multi_voxel_map_t *mm = m;

	int first_id = mm->first_block_id;
	int msgid = first_id/3;

	if(first_id%3 != 0 || first_id > 12 || first_id < 0)
	{
		printf("WARNING: Invalid first_id (%d) in multi_voxel_map\n", first_id);
		return;
	}

//	printf("Got multi voxel map, first id = %d. Msg id = %d\n", first_id, msgid);

	voxmap_alpha = max_voxmap_alpha;

	memcpy(&latest_voxmap.msgs[msgid], m, sizeof(latest_voxmap.msgs[0]));
}

/*
void print_(void* m)
{
}
*/


int parse_message(uint16_t id, uint32_t len)
{
	if(id < 256)
	{
		if(b2s_msgs[id].p_print)
			b2s_msgs[id].p_print(rxbuf);
	}

	switch(id)
	{
		


		case 435: // Route info
		{
			clear_route(&some_route);
			int n_elems = len/9;

			route_start_x = (int32_t)I32FROMBUF(rxbuf,0);
			route_start_y = (int32_t)I32FROMBUF(rxbuf,4);

			for(int i = 0; i < n_elems; i++)
			{
				route_unit_t* point = (route_unit_t*)malloc(sizeof(route_unit_t));
				point->backmode = rxbuf[i*9+8];
				point->loc.x = (int32_t)I32FROMBUF(rxbuf,i*9+9);
				point->loc.y = (int32_t)I32FROMBUF(rxbuf,i*9+13);
				printf("i=%d  back=%d, x=%d, y=%d\n", i, point->backmode, point->loc.x, point->loc.y);
				DL_APPEND(some_route, point);
			}
		}
		break;

#if 0

		case 436:
		{
			run_map_rsync();
		}
		break;

		case 437: // dbg_point
		{

			if(rxbuf[11] == 0)
			{
				show_dbgpoint = 1;
				dbgpoint_x = (int32_t)I32FROMBUF(rxbuf,0);
				dbgpoint_y = (int32_t)I32FROMBUF(rxbuf,4);
				dbgpoint_r = rxbuf[8];
				dbgpoint_g = rxbuf[9];
				dbgpoint_b = rxbuf[10];
			}
			else
			{
				pers_dbgpoint_x[num_pers_dbgpoints] = (int32_t)I32FROMBUF(rxbuf,0);
				pers_dbgpoint_y[num_pers_dbgpoints] = (int32_t)I32FROMBUF(rxbuf,4);
				pers_dbgpoint_r[num_pers_dbgpoints] = rxbuf[8];
				pers_dbgpoint_g[num_pers_dbgpoints] = rxbuf[9];
				pers_dbgpoint_b[num_pers_dbgpoints] = rxbuf[10];
				num_pers_dbgpoints++;
				if(num_pers_dbgpoints > 99) num_pers_dbgpoints = 0;
			}


		}
		break;
		case 439: // info state
		{
			cur_info_state = static_cast<info_state_t>(rxbuf[0]);
		}
		break;


		case 440: // Robot info
		{
			robot_xs = (double)I16FROMBUF(rxbuf, 0);
			robot_ys = (double)I16FROMBUF(rxbuf, 2);
			lidar_xoffs = (double)I16FROMBUF(rxbuf, 4);
			lidar_yoffs = (double)I16FROMBUF(rxbuf, 6);

			printf("Robot size msg: xs=%.1f ys=%.1f lidar_x=%.1f lidar_y=%.1f\n", robot_xs, robot_ys, lidar_xoffs, lidar_yoffs);
		}
		break;

		case 442: // Picture
		{
			pict_id = I16FROMBUF(rxbuf, 0);
			pict_bpp = rxbuf[2];
			pict_xs = I16FROMBUF(rxbuf, 3);
			pict_ys = I16FROMBUF(rxbuf, 5);
			printf("Picture msg: id=%u bytes_per_pixel=%u xs=%u ys=%u\n", pict_id, pict_bpp, pict_xs, pict_ys);
			int pict_size = pict_bpp*pict_xs*pict_ys;
			if(pict_size > 100000)
			{
				printf("Ignoring oversized image.\n");
				pict_id = -1;
			}
			else
				memcpy(pict_data, &rxbuf[7], pict_size);
		}
		break;

		case 443: // Movement status
		{
			int16_t mov_start_ang = I16FROMBUF(rxbuf, 0);
			int32_t mov_start_x = I32FROMBUF(rxbuf, 2);
			int32_t mov_start_y = I32FROMBUF(rxbuf, 6);

			int32_t mov_requested_x = I32FROMBUF(rxbuf, 10);
			int32_t mov_requested_y = I32FROMBUF(rxbuf, 14);
			int8_t  mov_requested_backmode = rxbuf[18];

			int16_t mov_cur_ang = I16FROMBUF(rxbuf, 19);
			int32_t mov_cur_x = I32FROMBUF(rxbuf, 21);
			int32_t mov_cur_y = I32FROMBUF(rxbuf, 25);
			uint8_t mov_status = rxbuf[29];
			uint32_t mov_obstacle_flags = I32FROMBUF(rxbuf, 30);

			if(mov_status == 0)
				sprintf(status_text, "Manual movement SUCCESS, start=(%d,%d)mm -> req=(%d,%d)mm, actual=(%d,%d)mm", mov_start_x, mov_start_y, mov_requested_x, mov_requested_y, mov_cur_x, mov_cur_y);
			else
				sprintf(status_text, "Manual movement STOPPED, start=(%d,%d)mm -> req=(%d,%d)mm, actual=(%d,%d)mm, statuscode=%u, HW obstacle flags=%08x", mov_start_x, mov_start_y, mov_requested_x, mov_requested_y, mov_cur_x, mov_cur_y, mov_status, mov_obstacle_flags);
		}
		break;

		case 444: // Route status
		{
			int16_t mov_start_ang = I16FROMBUF(rxbuf, 0);
			int32_t mov_start_x = I32FROMBUF(rxbuf, 2);
			int32_t mov_start_y = I32FROMBUF(rxbuf, 6);

			int32_t mov_requested_x = I32FROMBUF(rxbuf, 10);
			int32_t mov_requested_y = I32FROMBUF(rxbuf, 14);

			int16_t mov_cur_ang = I16FROMBUF(rxbuf, 18);
			int32_t mov_cur_x = I32FROMBUF(rxbuf, 20);
			int32_t mov_cur_y = I32FROMBUF(rxbuf, 24);

			uint8_t mov_status = rxbuf[28];
			int16_t mov_reroute_cnt = I16FROMBUF(rxbuf, 29);

			if(mov_status == 0)
				sprintf(status_text, "SUCCESSFULLY followed the route, start=(%d,%d)mm -> req=(%d,%d)mm, actual=(%d,%d)mm, needed to reroute %d times", 
					mov_start_x, mov_start_y, mov_requested_x, mov_requested_y, mov_cur_x, mov_cur_y, mov_reroute_cnt);
			else
			{
				static const char* fail_reasons[5] =
				{
					"Success",
					"Obstacles on map close to the beginning, can't get started",
					"Got a good start thanks to backing off, but obstacles on the way later",
					"Got a good start, but obstacles on the way later",
					"Unknown (newly implemented?) reason"
				};

				uint8_t reason = mov_status; if(reason >= 4) reason=4;

				sprintf(status_text, "GAVE UP routefinding, reason: %u[%s], start=(%d,%d)mm -> req=(%d,%d)mm, actual=(%d,%d)mm, did reroute %d times (reason applies to the latest reroute)", mov_status, 
					fail_reasons[reason], mov_start_x, mov_start_y, mov_requested_x, mov_requested_y, mov_cur_x, mov_cur_y, mov_reroute_cnt);
			}
		}
		break;

	#endif

		case 445: // State vector
		{
			if(len != STATE_VECT_LEN)
			{
				printf("Illegal state vector message length - do the API versions of rn1host and rn1client match?\n");
				break;
			}

			memcpy(received_state_vect.table, rxbuf, STATE_VECT_LEN);
			memcpy(state_vect_to_send.table, rxbuf, STATE_VECT_LEN);
		}
		break;

		default:
		{
			//printf("Note: unhandled message type, msgid=%u, paylen=%u\n", id, len);
		}
		break;
	}

	return 0;
}

int main(int argc, char** argv)
{
	bool f_pressed[13] = {false};
	bool return_pressed = false;
	int focus = 1;
	int online = 1;

	if(argc != 3)
	{
		printf("Usage: rn1client addr port\n");
		printf("Starting in offline mode.\n");
		online = 0;
	}


	rxbuf = new uint8_t[MAX_ACCEPTED_MSG_PAYLEN];


	init_rsync_argv();
	sprintf(status_text, "Status bar");

	if(online)
	{
		strncpy(rsync_argv[2], argv[1], 1023);
		rsync_argv[2][1023] = 0;

		serv_ip = argv[1];
		serv_port = atoi(argv[2]);

		tcpsock.setBlocking(false);
		printf("Connecting...\n");
		while(tcpsock.connect(serv_ip, serv_port) != sf::Socket::Done)
		{
			usleep(1000);
			//TODO: timeout
		}
	}

	if (!arial.loadFromFile("arial.ttf"))
	{
	    return 1;
	}

	sf::ContextSettings sets;
	sets.antialiasingLevel = 8;
	sf::RenderWindow win(sf::VideoMode(screen_x,screen_y), "PULUROBOT SLAM", sf::Style::Default, sets);
	win.setFramerateLimit(15);

	sf::Texture decors[NUM_DECORS];

	decors[INFO_STATE_IDLE].loadFromFile    ("decoration/idle.png");
	decors[INFO_STATE_THINK].loadFromFile   ("decoration/think.png");
	decors[INFO_STATE_FWD].loadFromFile     ("decoration/fwd.png");
	decors[INFO_STATE_REV].loadFromFile     ("decoration/rev.png");
	decors[INFO_STATE_LEFT].loadFromFile    ("decoration/left.png");
	decors[INFO_STATE_RIGHT].loadFromFile   ("decoration/right.png");
	decors[INFO_STATE_CHARGING].loadFromFile("decoration/charging.png");
	decors[INFO_STATE_DAIJUING].loadFromFile("decoration/party.png");



	sfml_gui gui(win, arial);

	#define BUT_WIDTH 220

	int but_start_x = screen_x-BUT_WIDTH;

	int but_localize    = gui.add_button(but_start_x, 50 + 0*35, 140, 25, "     Localize (init)", DEF_BUT_COL, DEF_BUT_FONT_SIZE+2, -1, DEF_BUT_COL_PRESSED, false);
	int but_stop        = gui.add_button(but_start_x, 50 + 1*35, 65, 25, "Stop", DEF_BUT_COL, DEF_BUT_FONT_SIZE+2, -1, DEF_BUT_COL_PRESSED, false);

	int but_route     = gui.add_button(but_start_x, 60 + 2*35, 100, 25, "Route", DEF_BUT_COL, DEF_BUT_FONT_SIZE, SYM_ROUTE, DEF_BUT_COL_PRESSED, false);
	int but_manu_fwd  = gui.add_button(but_start_x, 60 + 3*35, 100, 25, "Manual fwd", DEF_BUT_COL, DEF_BUT_FONT_SIZE, SYM_MANUAL, DEF_BUT_COL_PRESSED, false);
	int but_manu_back = gui.add_button(but_start_x+110, 60 + 3*35, 30, 25, "rev", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);
	int but_force_fwd = gui.add_button(but_start_x, 60 + 4*35, 100, 25, "Force fwd", DEF_BUT_COL, DEF_BUT_FONT_SIZE, SYM_FORCE, DEF_BUT_COL_PRESSED, false);
	int but_force_back= gui.add_button(but_start_x+110, 60 + 4*35, 30, 25, "rev", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);
	int but_pose      = gui.add_button(but_start_x, 60 + 5*35, 100, 25, "Rotate pose", DEF_BUT_COL, DEF_BUT_FONT_SIZE, SYM_POSE, DEF_BUT_COL_PRESSED, false);

	int but_findcharger = gui.add_button(but_start_x, 70 + 6*35, 140, 25, "  Find charger", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);

	int but_speedminus  = gui.add_button(but_start_x, 70 + 7*35, 25, 25, " -", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);
	int but_speedplus  = gui.add_button(but_start_x+115, 70 + 7*35, 25, 25, " +", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);

	int but_addconstraint  = gui.add_button(but_start_x, 70 + 8*35, 60, 25, "ADD", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);
	int but_remconstraint  = gui.add_button(but_start_x+65, 70 + 8*35, 60, 25, "REM", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);

	int but_state_vect[STATE_VECT_LEN];

	for(int i=0; i<STATE_VECT_LEN; i++)
	{
		but_state_vect[i] = gui.add_button(but_start_x, 70 + 9*35 + i*25, 190, 18, state_vect_names[i], DEF_BUT_COL, /*font size:*/11, -1, DEF_BUT_COL_PRESSED, SYM_STOP);
		state_vect_to_send.table[i] = received_state_vect.table[i] = 0;
	}
	

	bool right_click_on = false;
	bool left_click_on = false;
	double prev_click_x = 0.0, prev_click_y = 0.0;


	origin_x = mm_per_pixel*screen_x/2;
	origin_y = mm_per_pixel*screen_y/2;

	int cnt = 0;
	while(win.isOpen())
	{
		cnt++;
		int gui_box_xs = BUT_WIDTH;
		int gui_box_ys = screen_y-65;
		int gui_box_x = screen_x-BUT_WIDTH-15;
		int gui_box_y = 15;

		gui.buttons[but_route]->pressed =      (click_mode==MODE_ROUTE);
		gui.buttons[but_manu_fwd]->pressed =   (click_mode==MODE_MANUAL_FWD);
		gui.buttons[but_manu_back]->pressed =  (click_mode==MODE_MANUAL_BACK);
		gui.buttons[but_force_fwd]->pressed =  (click_mode==MODE_FORCE_FWD);
		gui.buttons[but_force_back]->pressed = (click_mode==MODE_FORCE_BACK);
		gui.buttons[but_pose]->pressed =       (click_mode==MODE_POSE);
		gui.buttons[but_addconstraint]->pressed = (click_mode==MODE_ADDCONSTRAINT);
		gui.buttons[but_remconstraint]->pressed = (click_mode==MODE_REMCONSTRAINT);

		state_is_unsynchronized = 0;
		for(int i=0; i<STATE_VECT_LEN; i++)
		{
			gui.buttons[but_state_vect[i]]->pressed = state_vect_to_send.table[i];
			if(received_state_vect.table[i] != state_vect_to_send.table[i])
			{
				gui.buttons[but_state_vect[i]]->symbol = SYM_PLAY;
				state_is_unsynchronized = 1;
			}
			else
			{
				gui.buttons[but_state_vect[i]]->symbol = SYM_STOP;
			}
		}


		if(poll_map_rsync() >= 0)
			load_all_pages_on_disk(&world);


		sf::Event event;
		while (win.pollEvent(event))
		{
			if(event.type == sf::Event::Closed)
				win.close();
			if(event.type == sf::Event::Resized)
			{
				sf::Vector2u size = win.getSize();
				screen_x = size.x;
				screen_y = size.y;
				but_start_x = screen_x-BUT_WIDTH;

				gui.buttons[but_route]->x = but_start_x;
				gui.buttons[but_manu_fwd]->x = but_start_x;
				gui.buttons[but_manu_back]->x = but_start_x+110;
				gui.buttons[but_force_fwd]->x = but_start_x;
				gui.buttons[but_force_back]->x = but_start_x+110;
				gui.buttons[but_pose]->x = but_start_x;
				gui.buttons[but_localize]->x = but_start_x;
				gui.buttons[but_stop]->x = but_start_x;
				gui.buttons[but_findcharger]->x = but_start_x;
				gui.buttons[but_speedminus]->x = but_start_x;
				gui.buttons[but_speedplus]->x = but_start_x+115;
				gui.buttons[but_addconstraint]->x = but_start_x;
				gui.buttons[but_remconstraint]->x = but_start_x+65;
		
				for(int i=0; i<STATE_VECT_LEN; i++)
				{
					gui.buttons[but_state_vect[i]]->x = but_start_x;
				}

				sf::FloatRect visibleArea(0, 0, screen_x, screen_y);
				win.setView(sf::View(visibleArea));
			}
			if(event.type == sf::Event::LostFocus)
				focus = 0;

			if(event.type == sf::Event::GainedFocus)
				focus = 1;

		}

		if(online)
		{
			for(int i=0; i<20; i++)
			{

				uint8_t headerbuf[5] = {0};
				size_t received = 0;

				uint32_t header_rx = 0;
				int something = 0;
				while(header_rx < 5)
				{
					sf::Socket::Status ret;

					if( (ret = tcpsock.receive(&headerbuf[header_rx], 5-header_rx, received)) == sf::Socket::Done)
					{
					//	printf("head %d\n", received);
						if(received != 0)
							something = 1;
						header_rx += received;
					}
					if(received == 0 && something == 0)
						goto NOTHING;
				}


				{

					uint16_t msgid = ((uint16_t)headerbuf[0]<<8) | ((uint16_t)headerbuf[1]<<0);
					uint32_t paylen = ((uint32_t)headerbuf[2]<<16) | ((uint32_t)headerbuf[3]<<8) | ((uint32_t)headerbuf[4]<<0);

					//printf("msgid=%u paylen=%u\n", msgid, paylen);



					if(paylen > MAX_ACCEPTED_MSG_PAYLEN)
					{
						printf("Error: msg too long.\n");
						win.close();
					}

					uint32_t total_rx = 0;
					while(total_rx < paylen)
					{
//						printf("pay\n");

						sf::Socket::Status ret;
						if( (ret = tcpsock.receive(&rxbuf[total_rx], paylen-total_rx, received)) == sf::Socket::Done)
						{
						//	printf("pay %d\n", received);
							total_rx += received;
				//			printf("    rx %d -> total %d\n", received, total_rx);
						}
					}


					if(total_rx != paylen)
						printf("error horror2  %d != %d\n", total_rx, paylen);

					parse_message(msgid, paylen);
				}
			}
			NOTHING:;
		}

		if(focus)
		{
			sf::Vector2i localPosition = sf::Mouse::getPosition(win);

			if(localPosition.x > gui_box_x-5 && localPosition.x < gui_box_x+gui_box_xs+5 && localPosition.y > gui_box_y-5 && localPosition.y < gui_box_y+gui_box_ys+5)
			{
				int but = gui.check_button_status();
				if     (but == but_route)      click_mode = MODE_ROUTE;
				else if(but == but_manu_fwd)   click_mode = MODE_MANUAL_FWD;
				else if(but == but_force_fwd)  click_mode = MODE_FORCE_FWD;
				else if(but == but_manu_back)  click_mode = MODE_MANUAL_BACK;
				else if(but == but_force_back) click_mode = MODE_FORCE_BACK;
				else if(but == but_pose)       click_mode = MODE_POSE;
				else if(but == but_addconstraint) click_mode = MODE_ADDCONSTRAINT;
				else if(but == but_remconstraint) click_mode = MODE_REMCONSTRAINT;

				if(but == but_speedplus)
				{
					gui.buttons[but_speedplus]->pressed = true;

					if(cnt&1)
					{
						if(cur_speed_limit < 10 || cur_speed_limit > 40)
							cur_speed_limit++;
						else
							cur_speed_limit = cur_speed_limit*11/10;

						if(cur_speed_limit > 70)
							cur_speed_limit = 70;
					}
				}
				else
				{
					if(gui.buttons[but_speedplus]->pressed)
					{
						gui.buttons[but_speedplus]->pressed = false;
						speedlimit_msg(cur_speed_limit);
					}
				}

				if(but == but_speedminus)
				{
					gui.buttons[but_speedminus]->pressed = true;

					if(cnt&1)
					{
						if(cur_speed_limit < 11)
							cur_speed_limit--;
						else
							cur_speed_limit = cur_speed_limit*10/11;

						if(cur_speed_limit < 1)
							cur_speed_limit = 1;
					}
					
				}
				else
				{
					if(gui.buttons[but_speedminus]->pressed)
					{
						gui.buttons[but_speedminus]->pressed = false;
						speedlimit_msg(cur_speed_limit);
					}
				}
				
				if(but == but_localize)
				{
					gui.buttons[but_localize]->pressed = true;
				}
				else
				{
					if(gui.buttons[but_localize]->pressed)
					{
						gui.buttons[but_localize]->pressed = false;
						mode_msg(3);
					}
				}

				if(but == but_stop)
				{
					gui.buttons[but_stop]->pressed = true;
				}
				else
				{
					if(gui.buttons[but_stop]->pressed)
					{
						mode_msg(8);
						gui.buttons[but_stop]->pressed = false;
					}
				}

				if(but == but_findcharger)
				{
					gui.buttons[but_findcharger]->pressed = true;
				}
				else
				{
					if(gui.buttons[but_findcharger]->pressed)
					{
						go_charge_msg(0);
						gui.buttons[but_findcharger]->pressed = false;
					}
				}

				static bool statebut_pressed[STATE_VECT_LEN];
				for(int i=0; i<STATE_VECT_LEN; i++)
				{
					if(but == but_state_vect[i])
					{
						if(!statebut_pressed[i])
						{
							statebut_pressed[i] = true;
							state_vect_to_send.table[i] = received_state_vect.table[i]?0:1;
							send(364, STATE_VECT_LEN, state_vect_to_send.table);
						}				
					}
					else
						statebut_pressed[i] = false;
				}




			}
			else if(localPosition.x > 10 && localPosition.x < screen_x-10 && localPosition.y > 10 && localPosition.y < screen_y-10)
			{
				click_x = (localPosition.x * mm_per_pixel) - origin_x;
				click_y = (-1*localPosition.y * mm_per_pixel) + origin_y;

				if(sf::Mouse::isButtonPressed(sf::Mouse::Left))
				{
//					bool shift_on = sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift);
					if(!left_click_on)
					{
						dest_type = click_mode;
						dest_x = click_x; dest_y = click_y;

						int back = 0;

						switch(click_mode)
						{
							case MODE_ROUTE: {
								clear_route(&some_route);

								int x = dest_x; int y = dest_y;

								uint8_t test[9] = {(x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff, 0};

								send(356, 9, test);
							} break;

							case MODE_MANUAL_BACK:
							back = 1;
							case MODE_MANUAL_FWD: {
								clear_route(&some_route);

								int x = dest_x; int y = dest_y;

								uint8_t test[9] = {(x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff, back};

								send(355, 9, test);

							} break;

							case MODE_FORCE_BACK:
							back = 1;
							case MODE_FORCE_FWD: {
								clear_route(&some_route);

								int x = dest_x; int y = dest_y;

								uint8_t test[12] = {55 /*DEST*/, 0, 9,   (x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff, 0b100 & back};

								if(tcpsock.send(test, 12) != sf::Socket::Done)
								{
									printf("Send error\n");
									sprintf(status_text, "Send error, connection lost?");
								}
								else
								{
									sprintf(status_text, "Sent command: move directly (force)");
								}


							} break;

							case MODE_POSE: {
								clear_route(&some_route);

								int x = dest_x; int y = dest_y;

								uint8_t test[12] = {55 /*DEST*/, 0, 9,   (x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff, 0b1000};

								if(tcpsock.send(test, 12) != sf::Socket::Done)
								{
									printf("Send error\n");
									sprintf(status_text, "Send error, connection lost?");
								}
								else
								{
									sprintf(status_text, "Sent command: rotate robot pose");
								}

							} break;

							case MODE_ADDCONSTRAINT: {
								int x = click_x; int y = click_y;
								uint8_t test[11] = {60, 0, 8,   (x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff};

								if(tcpsock.send(test, 11) != sf::Socket::Done)
								{
									printf("Send error\n");
									sprintf(status_text, "Send error, connection lost?");
								}
								else
								{
									sprintf(status_text, "Sent command: add forbidden area");
								}

							} break;

							case MODE_REMCONSTRAINT: {
								int x = click_x; int y = click_y;
								uint8_t test[11] = {61, 0, 8,   (x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff};

								if(tcpsock.send(test, 11) != sf::Socket::Done)
								{
									printf("Send error\n");
									sprintf(status_text, "Send error, connection lost?");
								}
								else
								{
									sprintf(status_text, "Sent command: remove forbidden area");
								}

							} break;
							default: break;
						}

					}

					left_click_on = true;
				}
				else
					left_click_on = false;

				if(sf::Mouse::isButtonPressed(sf::Mouse::Right))
				{
					if(right_click_on)
					{
						double dx = click_x - prev_click_x; double dy = click_y - prev_click_y;
						origin_x += dx; origin_y -= dy;
					}
					else
					{
						prev_click_x = click_x; prev_click_y = click_y;
					}

					right_click_on = true;
				}
				else
					right_click_on = false;
			}

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::PageUp))
			{
				origin_x -= (screen_x/2.0)*mm_per_pixel;
				origin_y -= (screen_y/2.0)*mm_per_pixel;
				mm_per_pixel *= 1.05;
				origin_x += (screen_x/2.0)*mm_per_pixel;
				origin_y += (screen_y/2.0)*mm_per_pixel;
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::PageDown))
			{
				origin_x -= (screen_x/2.0)*mm_per_pixel;
				origin_y -= (screen_y/2.0)*mm_per_pixel;
				mm_per_pixel *= 0.95;
				origin_x += (screen_x/2.0)*mm_per_pixel;
				origin_y += (screen_y/2.0)*mm_per_pixel;
			}

//			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F1)) { click_mode = MODE_ROUTE; }
//			else if(sf::Keyboard::isKeyPressed(sf::Keyboard::F2)) { click_mode = MODE_MANUAL; }
//			else if(sf::Keyboard::isKeyPressed(sf::Keyboard::F3)) { click_mode = MODE_FORCE; }
//			else if(sf::Keyboard::isKeyPressed(sf::Keyboard::F4)) { click_mode = MODE_POSE; }

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F5)) { if(!f_pressed[5]) 
			{
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))
					run_map_rsync();
				else
					load_all_pages_on_disk(&world);
				f_pressed[5] = true;
			}} else f_pressed[5] = false;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F6)) { if(!f_pressed[6]) 
			{
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))
				{
					maintenance_msg(6); // rn1host git pull + restart
					win.close();
				}
				f_pressed[6] = true;
			}} else f_pressed[6] = false;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F7)) { if(!f_pressed[7]) 
			{
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))
				{
					maintenance_msg(135); // reboot raspi
					win.close();
				}

				f_pressed[7] = true;
			}} else f_pressed[7] = false;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F8)) { if(!f_pressed[8]) 
			{
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))
				{
					maintenance_msg(10); // update firmware
					win.close();
				}
				f_pressed[8] = true;
			}} else f_pressed[8] = false;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F9)) { if(!f_pressed[9]) 
			{
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))
				{
					maintenance_msg(136); // shut down raspi
					win.close();
				}

				f_pressed[9] = true;
			}} else f_pressed[9] = false;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F10)) { if(!f_pressed[10]) 
			{
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))
				{
					maintenance_msg(7); // delete maps & restart rn1host
					win.close();
				}
				f_pressed[10] = true;
			}} else f_pressed[10] = false;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F11)) { if(!f_pressed[11]) 
			{
				mode_msg(7); // conf charger
				f_pressed[11] = true;
			}} else f_pressed[11] = false;

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::C))
			{
				num_pers_dbgpoints = 0;
				dbg_boost = 0;
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::B))
			{
				dbg_boost = 1;
			}

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::A))
			{
				animate_slice = true;
			}
			else
				animate_slice = false;


			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
			{
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
			{
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
			{
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
			{
			}

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Return))
			{
				if(!return_pressed)
				{
					return_pressed = 1;
				}
			}
			else
			{
				return_pressed = 0;
			}



		}

		#ifndef TOF_DEV
		win.clear(sf::Color(230,230,230));

		draw_map(win);

		draw_voxmap(win);

		draw_robot(win);

		draw_lidar(win, &lidar);
		draw_sonar(win);
		draw_tof3d_hmap(win, &hmap);
		if(hmap_alpha_mult) hmap_alpha_mult-=8; if(hmap_alpha_mult < 40) hmap_alpha_mult = 40;

		draw_hwdbg(win);
		draw_bat_status(win);

		draw_route_mm(win, &some_route);

		draw_drive_diag(win, &latest_drive_diag);

		draw_texts(win);
		sf::RectangleShape rect(sf::Vector2f( gui_box_xs, gui_box_ys));
		rect.setPosition(gui_box_x, gui_box_y);
		rect.setFillColor(sf::Color(255,255,255,160));
		win.draw(rect);
		gui.draw_all_buttons();



		{
			sf::Text t;
			char tbuf[256];
			t.setFont(arial);

			static int fx=0;

			sprintf(tbuf, "SPEED %d", cur_speed_limit);
			if(cur_speed_limit > 45)
				t.setFillColor(sf::Color(255,0,0,255));
			else
				t.setFillColor(sf::Color(200,200,0,255));
			t.setString(tbuf);
			t.setCharacterSize(14);
			t.setPosition(but_start_x+35, 70 + 7*35);
			win.draw(t);
		}

		{
			sf::Text t;
			t.setFont(arial);

			t.setString(status_text);
			t.setCharacterSize(12);
			t.setFillColor(sf::Color(255,255,255,200));
			t.setPosition(10, screen_y-15);
			win.draw(t);
			t.setFillColor(sf::Color(0,0,0,255));
			t.setPosition(8, screen_y-17);
			win.draw(t);
		}

		print_cur_cmd_status(win);

		if(static_cast<int>(cur_info_state) >= 0 && static_cast<int>(cur_info_state) < NUM_DECORS)
		{
			sf::Sprite decor_sprite;
			decor_sprite.setTexture(decors[static_cast<int>(cur_info_state)]);
			decor_sprite.setPosition(screen_x-180, (cur_info_state==INFO_STATE_DAIJUING)?(screen_y-240):(screen_y-220));
			win.draw(decor_sprite);
		}


		#endif

		if(tof_raws_came < 45)
		{
			draw_tof_raws(win);
		}
		tof_raws_came++;


		win.display();

		usleep(100);

		static int sonar_fade = 0;
		if(++sonar_fade > 10)
		{
			sonar_fade = 0;
			sonar[sonar_wr].c = 0;
			sonar_wr++; if(sonar_wr >= SONAR_POINTS) sonar_wr = 0;
		}

		if(voxmap_alpha>0) voxmap_alpha-=5;
		if(voxmap_alpha<0) voxmap_alpha=0;

	}

	delete rxbuf;
	deinit_rsync_argv();
	return 0;
}
