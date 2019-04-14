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

#ifndef CLIENT_MEMDISK_H
#define CLIENT_MEMDISK_H

#include <stdint.h>


void init_memdisk();

void draw_full_map(sf::RenderWindow& win);
void draw_page_piles(sf::RenderWindow& win);


void build_fullmap();
int manage_page_pile_ranges();
void reload_map();

void map_set_1();
void map_set_2();


#endif
