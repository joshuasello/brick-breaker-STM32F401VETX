/*
 * sprite.c
 *
 *  Created on: Oct 31, 2021
 *      Author: joshu
 */


/**
 * --------------------------------------------------
 * Includes
 * --------------------------------------------------*/

#include <stdint.h>

#include "sprite.h"
#include "update.h"


/**
 * --------------------------------------------------
 * Function Prototypes
 * --------------------------------------------------*/

void clear_box(screen_t* screen, unsigned int width, unsigned int height, int left, int top);


/**
 * --------------------------------------------------
 * Function Definitions
 * --------------------------------------------------*/

void clear_screen(screen_t* screen) {
	clear_box(screen, screen->width, screen->height, 0, 0);
}


void init_sprite(sprite_t* sprite, unsigned char* image, unsigned int width, unsigned int height, int left, int top, unsigned int hp) {
	sprite->image = image;
	sprite->width = width;
	sprite->height = height;
	sprite->left = left;
	sprite->top = top;
	sprite->hp = hp;
}



void draw_image(screen_t* screen, unsigned char* image, int left, int top, unsigned int width, unsigned int height) {
	unsigned char* screen_draw_position = screen->start + screen->width * top + left;
	unsigned char* pixel_position = image;

	if (width % 4 == 0) {
		for(int j = 0; j < height; j++) {
			for(int i = 0; i < width / 4; i++){
				*((uint32_t*)screen_draw_position) = *((uint32_t*)pixel_position);
				screen_draw_position += 4;
				pixel_position += 4;
			}

			screen_draw_position += (screen->width - width);
		}
	} else if (width % 2 == 0) {
		for(int j = 0; j < height; j++) {
			for(int i = 0; i < width / 2; i++){
				*((uint16_t*)screen_draw_position) = *((uint16_t*)pixel_position);
				screen_draw_position += 2;
				pixel_position += 2;
			}

			screen_draw_position += (screen->width - width);
		}
	} else {
		for(int j = 0; j < height; j++) {
			for(int i = 0; i < width; i++){
				*screen_draw_position = *pixel_position;
				++screen_draw_position;
				++pixel_position;
			}

			screen_draw_position += (screen->width - width);
		}
	}
}


void draw_sprite(sprite_t* sprite, screen_t* screen) {
	draw_image(screen, sprite->image, sprite->left, sprite->top, sprite->width, sprite->height);
}


void clear_sprite(sprite_t* sprite, screen_t* screen) {
	clear_box(screen, sprite->width, sprite->height, sprite->left, sprite->top);
}


void set_sprite_position(sprite_t* sprite, int new_left, int new_top) {
	// Sets the new position
	sprite->left = new_left;
	sprite->top = new_top;
}


void increment_sprite_position(sprite_t* sprite, int left_shift, int top_shift) {
	set_sprite_position(sprite, sprite->left + left_shift, sprite->top + top_shift);
}


void step_sprite_in_direction(sprite_t* sprite, int step, int angle) {
	increment_sprite_position(sprite, deltaX(step, angle), deltaY(step, angle));
}


unsigned int sprite_is_alive(sprite_t* sprite) {
	return sprite->hp > 0;
}


int check_deflect_off_bounary(sprite_t* sprite, int* angle, int left, int right, int top, int bottom) {
	if (!sprite_is_alive(sprite)) return 0;

	int deflected = 0;

	if (sprite->left <= left) {
		int new_left = left - (sprite->left - left); // Necessary evil
		sprite->left = new_left;
		*angle = 180 - *angle;
		deflected = 1;
	}

	if (right <= (sprite->left + sprite->width)){
		sprite->left = right - (sprite->left + sprite->width - right);
		*angle = 180 - *angle;
		deflected = 1;
	}

	if (sprite->top <= top) {
		int new_top = top - (sprite->top - top); // Necessary evil
		sprite->top = new_top;
		*angle = 360 - *angle;
		deflected = 1;
	}

	if (bottom <= (sprite->top + sprite->height)) {
		sprite->top = bottom - (sprite->top + sprite->height - bottom);
		*angle = 360 - *angle;
		deflected = 1;
	}

	// Make sure ball angle is inbetween 0 and 360
	if (*angle < 0) *angle += 360;
	if (*angle >= 360) *angle -= 360;

	return deflected;
}


int check_deflect_off_sprite(sprite_t* a, sprite_t* b, int* angle, int a_prev_left, int a_prev_top) {
	if (!sprite_is_alive(a) || !sprite_is_alive(b)) return 0;

	int deflected = 0;

	int a_left = a->left;
	int a_right = a->left + a->width;
	int a_prev_right = a_prev_left + a->width;
	int a_top = a->top;
	int a_bottom = a->top + a->height;
	int a_prev_bottom = a_prev_top + a->height;

	int b_left = b->left;
	int b_right = b->left + b->width;
	int b_top = b->top;
	int b_bottom = b->top + b->height;

	int a_h_inbetween = b_top <= a_bottom && a_top <= b_bottom;
	int a_v_inbetween = b_left <= a_right && a_left <= b_right;

	if (a_h_inbetween && b_right <= a_prev_left && a_left <= b_right) {
		a->left = b_right + (b_right - a_left);
		*angle = 180 - *angle;

		deflected = 1;
	}

	if (a_h_inbetween && a_prev_right <= b_left && b_left <= a_right){
		a->left = b_left - (a_right - b_left);
		*angle = 180 - *angle;

		deflected = 1;
	}

	if (a_v_inbetween && b_bottom <= a_prev_top && a_top <= b_bottom) {
		a->top = b_bottom + (b_bottom - a_top);
		*angle = 360 - *angle;

		deflected = 1;
	}

	if (a_v_inbetween && a_prev_bottom <= b_top && b_top <= a_bottom) {
		a->top = b_top - (a_bottom - b_top);
		*angle = 360 - *angle;

		deflected = 1;
	}

	// Make sure ball angle is inbetween 0 and 360
	if (*angle < 0) *angle += 360;
	if (*angle >= 360) *angle -= 360;

	return deflected;
}


void clear_box(screen_t* screen, unsigned int width, unsigned int height, int left, int top) {
	unsigned char* screen_draw_position = screen->start + screen->width*top + left;

	if (width % 4 == 0) {
		for(int j = 0; j < height; j++){
			for(int i = 0; i < width / 4; i++){
				*((uint32_t*)screen_draw_position) = 0;
				screen_draw_position += 4;
			}

			screen_draw_position += (screen->width - width);
		}
	} else if (width % 2 == 0) {
		for(int j = 0; j < height; j++){
			for(int i = 0; i < width / 2; i++){
				*((uint16_t*)screen_draw_position) = 0;
				screen_draw_position += 2;
			}

			screen_draw_position += (screen->width - width);
		}
	} else {
		for(int j = 0; j < height; j++){
			for(int i = 0; i < width; i++){
				*screen_draw_position = 0;
				++screen_draw_position;
			}

			screen_draw_position += (screen->width - width);
		}
	}
}
