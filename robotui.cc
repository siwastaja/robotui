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
#include <SFML/OpenGL.hpp>

#define DEFINE_API_VARIABLES
#include "../robotsoft/api_board_to_soft.h"
#include "../robotsoft/api_soft_to_board.h"
#undef DEFINE_API_VARIABLES

#include "../robotsoft/datatypes.h"
//#include "../rn1-brain/comm.h"
#include "client_memdisk.h"
#include "uthash.h"
#include "utlist.h"
#include "sfml_gui.h"

#ifdef SFML_OPENGL_ES
#define glClearDepth glClearDepthf
#define glFrustum glFrustumf
#endif

#ifndef GL_SRGB8_ALPHA8
#define GL_SRGB8_ALPHA8 0x8C43
#endif


#define I16FROMBUF(b_, s_)  ( ((uint16_t)b_[(s_)+0]<<8) | ((uint16_t)b_[(s_)+1]<<0) )
#define I32FROMBUF(b_, s_)  ( ((uint32_t)b_[(s_)+0]<<24) | ((uint32_t)b_[(s_)+1]<<16) | ((uint32_t)b_[(s_)+2]<<8) | ((uint32_t)b_[(s_)+3]<<0) )
#define I32TOBUF(i_, b_, s_) {(b_)[(s_)] = ((i_)>>24)&0xff; (b_)[(s_)+1] = ((i_)>>16)&0xff; (b_)[(s_)+2] = ((i_)>>8)&0xff; (b_)[(s_)+3] = ((i_)>>0)&0xff; }
#define I16TOBUF(i_, b_, s_) {(b_)[(s_)] = ((i_)>>8)&0xff; (b_)[(s_)+1] = ((i_)>>0)&0xff; }

bool view_3d = false;
bool show_realtime_pc = true;
bool show_free = false;


char status_text[2000];

uint32_t robot_id = 0xacdcabba;
int cur_speed_limit = 45;

sf::Font arial;

int screen_x = 1200;
int screen_y = 700;

double click_x_mm, click_y_mm;

double mm_per_pixel = 40.0;
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

double robot_xs = 250.0;
double robot_ys = 250.0;
double lidar_xoffs = 100.0;
double lidar_yoffs = 0.0;


#define RGBA32(r_,g_,b_,a_)  ((r_) | ((g_)<<8) | ((b_)<<16) | ((a_)<<24))


float offset_z = 400.0;
float blue_z = 1200.0;

extern int view2d_min_z;
extern int view2d_max_z;



static const uint32_t forbidden_color = RGBA32(255UL,  190UL, 190UL, 255UL);
static const uint32_t visited_color = RGBA32(255UL,  255L, 255UL, 255UL);
static const uint32_t routing_allowed_color = RGBA32(255UL,  255UL, 255UL, 255UL);
static const uint32_t routing_unallowed_color = RGBA32(0UL,  0UL, 0UL, 255UL);



bool show_routing = false;

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
	int run = mm->run;

	float x1, x2, y1, y2;
	x1 = (mm->cur_x+origin_x)/mm_per_pixel;
	y1 = (-1*mm->cur_y+origin_y)/mm_per_pixel;
	x2 = (mm->target_x+origin_x)/mm_per_pixel;
	y2 = (-1*mm->target_y+origin_y)/mm_per_pixel;

	sf::RectangleShape rect(sf::Vector2f( sqrt(pow(x2-x1,2)+pow(y2-y1,2)), 2.0));
	rect.setOrigin(0, 1.0);
	rect.setPosition(x1, y1);
	rect.setRotation(atan2(y2-y1,x2-x1)*180.0/M_PI);
	rect.setFillColor(run?sf::Color(255,200,200,200):sf::Color(160,200,200,200));

	win.draw(rect);


	sf::Text t;
	char buf[256];
	t.setFont(arial);

	sprintf(buf, "%.1f deg", ANGI32TOFDEG(mm->ang_err));
	t.setString(buf);
	t.setCharacterSize(12);
	t.setFillColor(run?sf::Color(255,200,200,255):sf::Color(160,200,200,255));
	t.setPosition((x1+x2)/2 + 4, (y1+y2)/2 - 7);
	win.draw(t);

	sprintf(buf, "%d mm", mm->lin_err);
	t.setString(buf);
	t.setCharacterSize(12);
	t.setFillColor(run?sf::Color(255,200,200,255):sf::Color(160,200,200,255));
	t.setPosition((x1+x2)/2 + 4, (y1+y2)/2 + 7);
	win.draw(t);

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
	const int bot_box_ys = 83;
	sf::RectangleShape rect(sf::Vector2f( bot_box_xs, bot_box_ys));
	rect.setPosition(screen_x/2 - bot_box_xs/2, screen_y-bot_box_ys-10-30);
	rect.setFillColor(sf::Color(255,255,255,160));
	win.draw(rect);


	sprintf(buf, "z view range: %d .. %d mm", view2d_min_z, view2d_max_z);
	t.setString(buf);
	t.setCharacterSize(14);
	t.setFillColor(sf::Color(0,0,0,255));
	t.setPosition(screen_x/2-bot_box_xs/2+10,screen_y-90-30);
	win.draw(t);


	sprintf(buf, "robot: x=%d  y=%d  mm  (yaw=%.1f pitch=%.1f roll=%.1f )", (int)cur_x, (int)cur_y, cur_angle, cur_pitch, cur_roll);
	t.setString(buf);
	t.setCharacterSize(14);
	t.setFillColor(sf::Color(0,0,0,160));
	t.setPosition(screen_x/2-bot_box_xs/2+10,screen_y-51-30);
	win.draw(t);

	sprintf(buf, "cursor: x=%d  y=%d  mm", (int)click_x_mm, (int)click_y_mm);
	t.setString(buf);
	t.setCharacterSize(14);
	t.setFillColor(sf::Color(0,0,0, 120));
	t.setPosition(screen_x/2-bot_box_xs/2+10,screen_y-30-30);
	win.draw(t);

	if(state_is_unsynchronized)
		sprintf(buf, "Robot state is unsynchronized...");
	else
		sprintf(buf, "%s", click_mode_names[click_mode]);
	t.setString(buf);
	t.setCharacterSize(17);
	t.setFillColor(sf::Color(0,0,0,130));
	t.setPosition(screen_x/2-bot_box_xs/2+11,screen_y-72-30);
	win.draw(t);
	t.setFillColor(click_mode_colors[click_mode]);
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
float blue_dist = 20000.0;
//float red_dist  = 1600.0;
//float blue_dist = 2200.0;

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
			//if(dbg_boost) { pixval*=4; pixval+=16; if(pixval>255) pixval=255;}

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

float tof_raw_scale = 0.0;

void draw_tof_raws(sf::RenderWindow& win)
{
	float scale = tof_raw_scale;

	static const int order[10] = { 5,4,3,2,1,0,9,8,7,6};

//	static const int order[10] = { 6,5,4,2,1,0,9,8,7,6};

//	sf::RectangleShape rect(sf::Vector2f( 10*(scale*60.0+4)  + 6, scale*TOF_XS_NARROW+(TOF_XS+TOF_XS_NARROW)*scale+6+scale*TOF_XS +17  +6));
//	rect.setPosition(20-3, 20-20);
//	rect.setFillColor(sf::Color(255,255,255,tof_raw_alpha));
//	win.draw(rect);

	for(int ii=0; ii<10; ii++)
//	for(int ii=0; ii<1; ii++)
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
		

			float textscale = scale;
			if(textscale > 1.2) textscale = 1.2;
			sf::Text te;
			char tbuf[32];
			sprintf(tbuf, "%u (%u%%)", latest_tof_dists[i].narrow_stray_estimate_adc, (100*latest_tof_dists[i].narrow_stray_estimate_adc+1)/16384);
			te.setFont(arial);
			te.setFillColor(sf::Color(255,255,255,tof_raw_alpha));
			te.setString(tbuf);
			te.setCharacterSize(textscale*10);
			te.setPosition(20+scale*((TOF_YS-TOF_YS_NARROW)/2.0) +ii*(scale*60.0+4), 20);
			win.draw(te);

			sprintf(tbuf, "%u (%u%%)", latest_tof_dists[i].wide_stray_estimate_adc, (100*latest_tof_dists[i].wide_stray_estimate_adc+1)/16384);
			te.setString(tbuf);
			te.setPosition(20+scale*((TOF_YS-TOF_YS_NARROW)/2.0) +ii*(scale*60.0+4), 20+scale*TOF_XS_NARROW+2);
			win.draw(te);

			sprintf(tbuf, "TOF%u", i);
			te.setString(tbuf);
			te.setCharacterSize(textscale*14);
			te.setFillColor(sf::Color(0,0,0,tof_raw_alpha));
			te.setPosition(20 + scale*20.0 +ii*(scale*60.0+4), 20-17);
			win.draw(te);
			te.setFillColor(sf::Color(255,255,255,tof_raw_alpha));
			te.setPosition(20 + scale*20.0 +ii*(scale*60.0+4)-1, 20-17-1);
			win.draw(te);

		#endif
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

#define SONAR_POINTS 6
sonar_point_t sonar[SONAR_POINTS];
static int sonar_wr = 0;




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


state_vect_t received_state_vect;
state_vect_t state_vect_to_send;

#define SENDBUF_SIZE (128*1024)

int send(int msgid, int paylen, uint8_t* buf)
{
	static uint8_t sendbuf[SENDBUF_SIZE];

	if(paylen >= SENDBUF_SIZE-5)
	{
		printf("Message too long to be sent.\n");
		return 1;
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
		return 1;
	}
	return 0;
}

void mode_msg(uint8_t mode)
{
	uint8_t test[1] = {mode};

	send(358, 1, test);
}

void speedlimit_msg(uint8_t limit)
{
	uint8_t test[5] = {limit, limit, limit, 40, 40};

	send(363, 5, test);
}


void go_charge_msg(uint8_t params)
{
	uint8_t test[1] = {params};

	send(357, 1, test);
}

void maintenance_msg(int restart_mode)
{
	const int size = 4+4;
	uint8_t test[size];
	I32TOBUF(0x12345678, test, 0);
	I32TOBUF(restart_mode, test, 4);

	send(362, size, test);
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
void print_gyrocal_results(void* m)
{
}

void print_tof_slam_set(void* m)
{

	tof_slam_set_t* mm = m;
	uint8_t idx = mm->sidx;
//	printf("GOT TOF SLAM SET: IDX = %d, ORIEN = %d\n", idx, mm->sensor_orientation);

	if(idx >= N_TOF_SENSORS)
	{
		printf("Invalid tof sensor idx=%u\n", idx);
		return;
	}

	latest_tof_dists[idx].sensor_orientation = mm->sensor_orientation;

	int set = 0;

	// Wanna show long wide set?
	if(mm->flags & TOF_SLAM_SET_FLAG_SET1_WIDE)
		set = 1;	

	for(int i=0; i<160*60; i++)
	{
		uint16_t val = (mm->sets[set].ampldist[i]&0xfff);
		if(val == 0) val = DATA_LOW;
		else if(val == 1) val = DATA_OVEREXP;
		else val *= 8;
		latest_tof_dists[idx].dist[i] = val;
		latest_tof_ampls[idx].ampl[i] = ((mm->sets[set].ampldist[i]&0xf000)>>12)*16+15;
	}



	if(mm->flags & TOF_SLAM_SET_FLAG_SET1_NARROW)
	{
		for(int i=0; i<TOF_XS_NARROW*TOF_YS_NARROW; i++)
		{
			uint16_t val = (mm->sets[1].ampldist[i]&0xfff);
			if(val == 0) val = DATA_LOW;
			else if(val == 1) val = DATA_OVEREXP;
			else val *= 8;
			latest_tof_dists[idx].dist_narrow[i] = val;
			latest_tof_ampls[idx].ampl_narrow[i] = ((mm->sets[1].ampldist[i]&0xf000)>>12)*16+15;
		}
	}

	tof_raws_came = 0;

}


void print_compass_heading(void* m)
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
	cur_pitch = ANGI32TOFDEG(mm->pitch);
	cur_roll = ANGI32TOFDEG(mm->roll);
	cur_x = mm->x;
	cur_y = mm->y;
}

drive_diag_t latest_drive_diag;

void print_drive_diag(void* m)
{
	drive_diag_t *mm = m;

/*	printf("Drive diagnostics  ang_err=%5.2f deg lin_err=%d mm cur (%d, %d), targ (%d, %d), id=%d, remaining %d mm, stop_flags=%08x\nrun=%u, dbg1=%d, dbg2=%d, dbg3=%d, dbg4=%d\n"
	       "dbg5=%d  dbg6=%d, ang_speed_i=%d, lin_speed_i=%d\n",
		ANGI32TOFDEG(mm->ang_err), mm->lin_err, 
		mm->cur_x, mm->cur_y, mm->target_x, mm->target_y, mm->id, mm->remaining, mm->micronavi_stop_flags, mm->run,
		mm->dbg1, mm->dbg2, mm->dbg3, mm->dbg4, mm->dbg5, mm->dbg6, mm->ang_speed_i, mm->lin_speed_i);
*/
	memcpy(&latest_drive_diag, m, sizeof latest_drive_diag);
}

void print_mcu_voxel_map(void* m)
{
	printf("WARN: mcu_voxel_map is deprecated, ignoring\n");
}

void print_mcu_multi_voxel_map(void* m)
{
	printf("WARN: mcu_multi_voxel_map is deprecated, ignoring\n");
}

void print_chafind_results(void* m)
{
}

/*
void print_(void* m)
{
}
*/


int parse_message(uint16_t id, uint32_t len)
{
	// Handle RobotBoard2-originated firmware messages with their API-defined function pointers:
	if(id < 256)
	{
		if(b2s_msgs[id].p_print)
			b2s_msgs[id].p_print(rxbuf);

		return 0;
	}

	// Handle robotsoft-originated messages:

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

		case 436:
		{
			//run_map_rsync();
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

			printf("STATUS MESSAGE: %s\n", status_text);
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
				static const char* fail_reasons[6] =
				{
					"Success",
					"Obstacles on map close to the beginning, can't get started",
					"Got a good start thanks to backing off, but obstacles on the way later",
					"Got a good start, but obstacles on the way later",
					"Sudden micronavi stop (unmapped obstacle appeared, can't pass)",
					"Unknown (newly implemented?) reason"
				};

				uint8_t reason = mov_status; if(reason >= 5) reason=5;

				sprintf(status_text, "GAVE UP routefinding, reason: %u[%s], start=(%d,%d)mm -> req=(%d,%d)mm, actual=(%d,%d)mm, did reroute %d times (reason applies to the latest reroute)", mov_status, 
					fail_reasons[reason], mov_start_x, mov_start_y, mov_requested_x, mov_requested_y, mov_cur_x, mov_cur_y, mov_reroute_cnt);
			}

			printf("STATUS MESSAGE: %s\n", status_text);

		}
		break;

#if 0


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

		case 447:
		{
			process_realtime_pointcloud(rxbuf, len, view_3d);
		}
		break;

		case 448:
		{
			process_tcp_voxmap(rxbuf, len);
		}
		break;

		default:
		{
			printf("Note: unhandled message type, msgid=%u, paylen=%u\n", id, len);
		}
		break;
	}

	return 0;
}

typedef struct
{
	int enabled;
	int x;
	int y;
	int xs;
	int ys;
	int text_size;
	sf::Color color;
	char* text;
} popup_menu_item_t;

#define POPUP_MENU_MAX_ITEMS 8
typedef struct
{
	int enabled;
	int point_x;
	int point_y;
	int x;
	int y;
	int xs;
	int ys;
	int n_items;
	popup_menu_item_t items[POPUP_MENU_MAX_ITEMS];
} popup_menu_t;

#define MENU_ITEM_XS 180
#define MENU_ITEM_YS 45
#define MENU_ITEM_YGAP 55

popup_menu_t popup_menu =
{
	0,
	0,
	0,
	0,
	0,
	10,
	10,
	6,
	{
#define MENU_EXPLORE_3D 0
		{
			1,
			0, 0*MENU_ITEM_YGAP,
			MENU_ITEM_XS, MENU_ITEM_YS,
			16,
			sf::Color(180,180,180,255),
			"Explore 3D"
		},
#define MENU_ROUTE_HERE 1
		{
			1,
			0, 1*MENU_ITEM_YGAP,
			MENU_ITEM_XS, MENU_ITEM_YS,
			16,
			sf::Color(210,160,160,255),
			"Route here"
		},
#define MENU_TURN_HERE 2
		{
			1,
			0, 2*MENU_ITEM_YGAP,
			MENU_ITEM_XS, MENU_ITEM_YS,
			16,
			sf::Color(160,210,160,255),
			"Turn here"
		},
#define MENU_FORWARD 3
		{
			1,
			0, 3*MENU_ITEM_YGAP,
			MENU_ITEM_XS/2-8, MENU_ITEM_YS,
			16,
			sf::Color(160,160,210,255),
			"Forward"
		},
#define MENU_BACKWARD 4
		{
			1,
			MENU_ITEM_XS/2+8, 3*MENU_ITEM_YGAP,
			MENU_ITEM_XS/2-8, MENU_ITEM_YS,
			16,
			sf::Color(160,160,210,255),
			"Backward"
		},
#define MENU_CLOSE_MENU 5
		{
			1,
			0, 4*MENU_ITEM_YGAP,
			MENU_ITEM_XS, MENU_ITEM_YS,
			16,
			sf::Color(160,210,160,255),
			"Close menu"
		}

		
	}	
};

const int x_margin = 8;
const int y_margin = 8;

int is_popup_enabled(popup_menu_t* m)
{
	return m->enabled;
}

void enable_popup_menu(popup_menu_t* m, int point_x, int point_y)
{

	int max_x = 0;
//	int total_ys = 0;
	for(int i = 0; i < m->n_items; i++)
	{
		if(m->items[i].x + m->items[i].xs > max_x)
			max_x = m->items[i].x + m->items[i].xs;
	}

	m->xs = x_margin*2 + max_x;
	m->ys = y_margin*2 + m->items[m->n_items-1].y +  m->items[m->n_items-1].ys;

	const int gap_to_point_x = 25;

	if(point_x + m->xs + gap_to_point_x < screen_x) // Right side
	{
		m->x = point_x + gap_to_point_x;
	}
	else // left side
	{
		m->x = point_x - m->xs - gap_to_point_x;
	}

	m->y = point_y - m->ys/2;
	if(m->y < 5) m->y = 5;
	if(m->y > screen_y-m->ys-5) m->y = screen_y-m->ys-5;

	m->point_x = point_x;
	m->point_y = point_y;

	m->enabled = 1;
}

int test_popup_click(popup_menu_t* m, int mx, int my)
{
	for(int i=0; i< m->n_items; i++)
	{
		if(mx >= m->x+m->items[i].x && mx <= m->x+m->items[i].x+m->items[i].xs &&
		   my >= m->y+m->items[i].y && my <= m->y+m->items[i].y+m->items[i].ys)
		{
			m->enabled = 0;
			return i;
		}
	}

	return -1;

}

void draw_popup_menu(sf::RenderWindow& win, popup_menu_t* m)
{
	if(!m->enabled)
		return;

	sf::Text t;
	char buf[256];
	t.setFont(arial);

	sf::RectangleShape rect(sf::Vector2f(m->xs, m->ys));
	rect.setPosition(m->x, m->y);
	rect.setFillColor(sf::Color(200,210,255,160));
	win.draw(rect);

	for(int i=0; i<m->n_items; i++)
	{
		sf::RectangleShape but(sf::Vector2f(m->items[i].xs, m->items[i].ys));
		but.setOutlineThickness(2.0);
		but.setOutlineColor(sf::Color(0,0,0,180));

		but.setPosition(x_margin+m->x+m->items[i].x, y_margin+m->y+m->items[i].y);
		but.setFillColor(m->items[i].color);
		win.draw(but);

		sprintf(buf, "%s", m->items[i].text);
		t.setString(buf);
		t.setCharacterSize(m->items[i].text_size);
		t.setFillColor(sf::Color(0,0,0,255));
		t.setPosition(x_margin+m->x+m->items[i].x+5, y_margin+m->y+m->items[i].y+7);
		win.draw(t);
	}


	const double r1 = 15.0;
	const double r2 = 6.0;

	sf::CircleShape circ(r1);
	circ.setOrigin(r1, r1);
	circ.setFillColor(sf::Color(100,255,255,100));
	circ.setOutlineThickness(1.0);
	circ.setOutlineColor(sf::Color(0,0,0,100));
	circ.setPosition(m->point_x, m->point_y);
	win.draw(circ);

	circ.setRadius(r2);
	circ.setOrigin(r2, r2);
	circ.setFillColor(sf::Color(100,255,255,160));
	win.draw(circ);


}

float campos_x = 0.0;
float campos_y = 0.0;
float campos_z = 0.0;

double camera_yaw = 0.0;
double camera_pitch = 0.0;

bool animate_jump_in = false;
int animate_frame = 0;

float animate_dx, animate_dy, animate_dz, animate_dyaw, animate_dpitch;

#define ANIMATE_FRAMES 30

void jump_in(double x_mm, double y_mm)
{
	campos_x = x_mm;
	campos_y = y_mm;
//	campos_z = (view2d_min_z+view2d_max_z)/2;
	campos_z = view2d_min_z + (view2d_max_z-view2d_min_z)/4 + mm_per_pixel*500.0;
	if(campos_z > view2d_max_z+25000) campos_z = view2d_max_z+25000;
	camera_yaw = M_PI/2.0;
	camera_pitch = DEGTORAD(-89.0);

	animate_dx = 0.0;
	animate_dy = 0.0;

	animate_dyaw = 0.0;

	double target_z = (view2d_min_z+view2d_max_z)/2;
	double target_pitch = 0.0;
	
	animate_dz = (target_z-campos_z)/(double)ANIMATE_FRAMES;
	animate_dpitch = (target_pitch-camera_pitch)/(double)ANIMATE_FRAMES;

	view_3d = true;
	animate_jump_in = true;
	animate_frame = 0;

}

void animate()
{
	if(!animate_jump_in)
		return;

	campos_x += animate_dx;
	campos_y += animate_dy;
	campos_z += animate_dz;
	camera_yaw += animate_dyaw;
	camera_pitch += animate_dpitch;

	animate_frame++;

	if(animate_frame >= ANIMATE_FRAMES)
		animate_jump_in = false;


}



int main(int argc, char** argv)
{
	bool f_pressed[13] = {false};
	bool return_pressed = false;
	int focus = 1;
	int online = 1;

	if(argc != 3)
	{
		printf("Usage: robotui addr port\n");
		printf("Starting in offline mode.\n");
		online = 0;
	}


	rxbuf = new uint8_t[MAX_ACCEPTED_MSG_PAYLEN];


	sprintf(status_text, "Status bar");

	if(online)
	{
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
//	sets.antialiasingLevel = 8;
	// SFML drawing functions won't work on opengl 3.3, which seems to force "core" profile.
	// 3.0 is OK for us.
	sets.majorVersion = 3;
	sets.minorVersion = 0;
	sets.depthBits = 24;
	sf::RenderWindow win(sf::VideoMode(screen_x,screen_y), "PULUROBOT SLAM", sf::Style::Default, sets);

	sf::ContextSettings settings = win.getSettings();
	printf("OpenGL version: %d.%d\n", settings.majorVersion,settings.minorVersion);

	win.setActive(true);
	printf("GPU: Vendor:   %s\n", glGetString(GL_VENDOR));

	init_memdisk();
	build_fullmap();

	init_opengl(win);


	win.setActive(false);


	win.setFramerateLimit(30);

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
	int but_state_vect[STATE_VECT_LEN];

	for(int i=0; i<STATE_VECT_LEN; i++)
	{
		but_state_vect[i] = gui.add_button(but_start_x, 50 + i*29, 190, 22, state_vect_names[i], DEF_BUT_COL, /*font size:*/12, -1, DEF_BUT_COL_PRESSED, SYM_STOP);
		state_vect_to_send.table[i] = received_state_vect.table[i] = 0;
	}

	bool click_on = false;
	bool ignore_click = false;
	double prev_click_x = 0.0, prev_click_y = 0.0;
	double prev_click_x_mm = 0.0, prev_click_y_mm = 0.0;

	

	int click_start_x = 0, click_start_y = 0;

	origin_x = mm_per_pixel*screen_x/2;
	origin_y = mm_per_pixel*screen_y/2;



	win.setActive(false);
	win.pushGLStates();

	sf::Clock clock;
	float last_time = 0.0;
	int cnt = 0;
	while(win.isOpen())
	{
		cnt++;


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
				for(int i=0; i<STATE_VECT_LEN; i++)
				{
					gui.buttons[but_state_vect[i]]->x = but_start_x;
				}

				sf::FloatRect visibleArea(0, 0, screen_x, screen_y);
				win.setView(sf::View(visibleArea));

				win.popGLStates();
				win.setActive(true);

				resize_opengl(win);

				win.setActive(false);
				win.pushGLStates();

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

		bool state_button_pressed = false;
		if(focus)
		{
			sf::Vector2i localPosition = sf::Mouse::getPosition(win);


			if(localPosition.x > 2 && localPosition.x < screen_x-2 && localPosition.y > 2 && localPosition.y < screen_y-2)
			{
				int but = gui.check_button_status();
				if(but>-1) state_button_pressed = true;
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

				int click_x = localPosition.x;
				int click_y = localPosition.y;
				click_x_mm = (localPosition.x * mm_per_pixel) - origin_x;
				click_y_mm = (-1*localPosition.y * mm_per_pixel) + origin_y;


#if 0
				//if(sf::Mouse::isButtonPressed(sf::Mouse::Left))
				if(0)
				{
//					bool shift_on = sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift);
					if(!click_on)
					{
						dest_type = click_mode;
						dest_x = click_x_mm; dest_y = click_y_mm;

						int back = 0;

						switch(click_mode)
						{
							case MODE_ROUTE: {
								clear_route(&some_route);
								sprintf(status_text, "Status bar");

								int x = dest_x; int y = dest_y;

								uint8_t test[9] = {(x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff, 0};

								if(send(356, 9, test))
								{
									printf("Send error\n");
									sprintf(status_text, "Send error, connection lost?");
								}
								else
								{
									sprintf(status_text, "Sent command: find route");
								}

							} break;

							case MODE_MANUAL_BACK:
							back = 1;
							case MODE_MANUAL_FWD: {
								clear_route(&some_route);
								sprintf(status_text, "Status bar");

								int x = dest_x; int y = dest_y;

								uint8_t test[9] = {(x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff, back};

								if(send(355, 9, test))
								{
									printf("Send error\n");
									sprintf(status_text, "Send error, connection lost?");
								}
								else
								{
									sprintf(status_text, "Sent command: move directly");
								}

							} break;

							case MODE_FORCE_BACK:
							back = 1;
							case MODE_FORCE_FWD: {
								clear_route(&some_route);
								sprintf(status_text, "Status bar");

								int x = dest_x; int y = dest_y;

								uint8_t test[9] = {  (x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff, 0b100 & back};

								if(send(355, 9, test))
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
								sprintf(status_text, "Status bar");

								int x = dest_x; int y = dest_y;

								uint8_t test[9] = { (x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff, 0b1000};

								if(send(355, 9, test))
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
								int x = click_x_mm; int y = click_y_mm;
								uint8_t test[8] = {  (x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff};

								if(send(360, 8, test))
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
								int x = click_x_mm; int y = click_y_mm;
								uint8_t test[8] = {(x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff};

								if(send(361, 8, test))
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

					click_on = true;
				}
				else
					click_on = false;

#endif
				if(sf::Mouse::isButtonPressed(sf::Mouse::Left))
				{
					if(is_popup_enabled(&popup_menu))
					{
						int but = test_popup_click(&popup_menu, click_x, click_y);

						double point_x_mm = ((double)popup_menu.point_x * mm_per_pixel) - origin_x;
						double point_y_mm = (-1*(double)popup_menu.point_y * mm_per_pixel) + origin_y;

						//printf("Pressed %d\n", but);

						int back = 0;
						switch(but)
						{
							case MENU_EXPLORE_3D:
							{
								if(!view_3d)
								{
									jump_in(((double)popup_menu.point_x*mm_per_pixel) - origin_x,
										((double)(-1*popup_menu.point_y) * mm_per_pixel) + origin_y);
								}
								else
									view_3d = false;
							} break;

							case MENU_BACKWARD:
							back = 1;
							case MENU_FORWARD: 
							{
								clear_route(&some_route);
								sprintf(status_text, "Status bar");
		
								dest_x = point_x_mm; dest_y = point_y_mm;

								int x = dest_x; int y = dest_y;

								uint8_t test[9] = {(x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff, back};

								if(send(355, 9, test))
								{
									printf("Send error\n");
									sprintf(status_text, "Send error, connection lost?");
								}
								else
								{
									sprintf(status_text, "Sent command: move directly");
								}

							} break;

							case MENU_ROUTE_HERE: 
							{
								clear_route(&some_route);
								sprintf(status_text, "Status bar");
		
								dest_x = point_x_mm; dest_y = point_y_mm;

								int x = dest_x; int y = dest_y;

								uint8_t test[9] = {(x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff, 0};

								if(send(356, 9, test))
								{
									printf("Send error\n");
									sprintf(status_text, "Send error, connection lost?");
								}
								else
								{
									sprintf(status_text, "Sent command: move directly");
								}

							} break;

							default: break;

						}

						click_on = false;
						ignore_click = true;

					}
					else
					{
						if(click_on)
						{
							if(view_3d)
							{
								double dx = click_x - prev_click_x;
								double dy = click_y - prev_click_y;

								camera_yaw +=  dx*-0.004;
								camera_pitch += dy*-0.004;
								if(camera_pitch < DEGTORAD(-89.0)) camera_pitch = DEGTORAD(-89.0);
								if(camera_pitch > DEGTORAD(+89.0)) camera_pitch = DEGTORAD(+89.0);

								prev_click_x = click_x;
								prev_click_y = click_y;

							}
							else
							{
								double dx = click_x_mm - prev_click_x_mm;
								double dy = click_y_mm - prev_click_y_mm;

								origin_x += dx;
								origin_y -= dy;
							}

						}
						else
						{

							click_start_x = click_x;
							click_start_y = click_y;

							prev_click_x = click_x;
							prev_click_y = click_y;

							prev_click_x_mm = click_x_mm;
							prev_click_y_mm = click_y_mm;


						}

						click_on = true;
					}
				}
				else
				{
					if(!ignore_click && click_on && abs(click_x-click_start_x) < 2 && abs(click_y-click_start_y) < 2 && (click_x < but_start_x))
					{
						if(!is_popup_enabled(&popup_menu))
							enable_popup_menu(&popup_menu, click_x, click_y);
					}
					click_on = false;
					ignore_click = false;

				}
			}

			static bool t_pressed;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::T))
			{
				if(!t_pressed)
				{
					if(tof_raw_scale < 0.4)
						tof_raw_scale = 0.5;
					else if(tof_raw_scale > 0.4 && tof_raw_scale < 0.6)
						tof_raw_scale = 1.0;
					else if(tof_raw_scale > 0.9 && tof_raw_scale < 1.1)
						tof_raw_scale = 2.0;
					else if(tof_raw_scale > 1.9)
						tof_raw_scale = 0.0;
					t_pressed = true;
				}
			}
			else t_pressed = false;

			static bool y_pressed;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Y))
			{
				if(!y_pressed)
				{
					show_realtime_pc = !show_realtime_pc;
					y_pressed = true;
				}
			}
			else y_pressed = false;


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
				map_set_1();
				reload_map();
				reload_3d_map(campos_x, campos_y, campos_z, camera_yaw, camera_pitch);
				f_pressed[5] = true;
			}} else f_pressed[5] = false;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F6)) { if(!f_pressed[6]) 
			{
				map_set_2();
				reload_map();
				reload_3d_map(campos_x, campos_y, campos_z, camera_yaw, camera_pitch);

/*
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))
				{
					maintenance_msg(6); // rn1host git pull + restart
					win.close();
				}*/
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

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::R))
			{
				show_routing = true;
			}
			else
				show_routing = false;

			show_free = sf::Keyboard::isKeyPressed(sf::Keyboard::F);



			float speed = 100.0;

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))
				speed *= 4.0;

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::W))
			{
				campos_y += speed*sin(camera_yaw);
				campos_x += speed*cos(camera_yaw);
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::S))
			{
				campos_y += -speed*sin(camera_yaw);
				campos_x += -speed*cos(camera_yaw);
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::A))
			{
				campos_y += speed*sin(camera_yaw+(M_PI/2.0));
				campos_x += speed*cos(camera_yaw+(M_PI/2.0));
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::D))
			{
				campos_y += speed*sin(camera_yaw-(M_PI/2.0));
				campos_x += speed*cos(camera_yaw-(M_PI/2.0));
			}

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::PageUp))
			{
				campos_z += speed;
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::PageDown))
			{
				campos_z += -speed;
			}

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Num3))
			{
				view_3d = true;
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Num2))
			{
				view_3d = false;
			}

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
			{
				view2d_max_z += 50;
				reload_map();
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
			{
				view2d_max_z -= 50;
				reload_map();
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
			{
				view2d_min_z -= 50;
				reload_map();
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
			{
				view2d_min_z += 50;
				reload_map();
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

		static int use_fullmap;
		static int manage;
		manage++;
		if(manage > 10)
		{
			if(view_3d)
			{
				manage_mesh_ranges(campos_x, campos_y, campos_z, camera_yaw, camera_pitch, 0);
			}
			else
			{
				use_fullmap = manage_page_pile_ranges();
				//printf("use_fullmap=%d\n", use_fullmap);
			}

			manage = 0;
		}

		static bool prev_view_3d;
		if(view_3d)
		{
			mutex_gl.lock();

			win.popGLStates();
			win.setActive(true);

			render_3d(campos_x, campos_y, campos_z, camera_yaw, camera_pitch);


			win.setActive(false);
			win.pushGLStates();
		}
		else
		{
			if(prev_view_3d)
			{
				wait_manage_mesh();
			}

			mutex_gl.lock();

			win.clear(sf::Color(230,230,230));

			if(use_fullmap)
				draw_full_map(win);
			else
				draw_page_piles(win);


			if(show_realtime_pc)
				draw_realtime_pc_2d(win);


			draw_robot(win);

			draw_sonar(win);
			draw_route_mm(win, &some_route);
			draw_drive_diag(win, &latest_drive_diag);
		}

		prev_view_3d = view_3d;


		draw_bat_status(win);


		draw_texts(win);


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


		if(tof_raws_came < 100)
		{
			draw_tof_raws(win);
		}
		tof_raws_came++;

		float current_time = clock.getElapsedTime().asSeconds();
		float fps = 1.f / (current_time);
		clock.restart();

		{
			sf::Text t;
			char tbuf[256];
			t.setFont(arial);

			static int fx=0;

			sprintf(tbuf, "%.1f FPS", fps);
			t.setFillColor(sf::Color(255,255,255,255));
			t.setString(tbuf);
			t.setCharacterSize(12);
			t.setPosition(5, 5);
			win.draw(t);
		}



		draw_popup_menu(win, &popup_menu);

		if(!popup_menu.enabled)
			gui.draw_all_buttons();

		win.display();

		mutex_gl.unlock();

		animate();

		usleep(100);

		static int sonar_fade = 0;
		if(++sonar_fade > 10)
		{
			sonar_fade = 0;
			sonar[sonar_wr].c = 0;
			sonar_wr++; if(sonar_wr >= SONAR_POINTS) sonar_wr = 0;
		}

	}

	delete rxbuf;
	return 0;
}
