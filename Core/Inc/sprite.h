/*
 * sprite.h
 *
 *  Created on: Oct 31, 2021
 *      Author: joshu
 */

#ifndef INC_SPRITE_H_
#define INC_SPRITE_H_

/**
 * --------------------------------------------------
 * Type Definitions
 * --------------------------------------------------*/

/**
 * Screen Structure
 */
typedef struct Screen {
	unsigned char* start;
	unsigned int width;
	unsigned int height;
} screen_t;


/**
 * Sprite Structure
 */
typedef struct Sprite {
	unsigned char* image;
	unsigned int width;
	unsigned int height;
	int left;
	int top;
	unsigned int hp;
} sprite_t;


/**
 * --------------------------------------------------
 * Function Prototypes
 * --------------------------------------------------*/

void clear_screen(screen_t* screen);

void init_sprite(sprite_t* sprite, unsigned char* image, unsigned int width, unsigned int height, int left, int top, unsigned int hp);

void draw_image(screen_t* screen, unsigned char* image, int left, int top, unsigned int width, unsigned int height);

void draw_sprite(sprite_t* sprite, screen_t* screen);

void clear_box(screen_t* screen, unsigned int width, unsigned int height, int left, int top);

void clear_sprite(sprite_t* sprite, screen_t* screen);

void set_sprite_position(sprite_t* sprite, int new_left, int new_top);

void increment_sprite_position(sprite_t* sprite, int left_shift, int top_shift);

void step_sprite_in_direction(sprite_t* sprite, int step, int angle);

unsigned int sprite_is_alive(sprite_t* sprite);

int check_deflect_off_bounary(sprite_t* sprite, int* angle, int left, int right, int top, int bottom);

int check_deflect_off_sprite(sprite_t* a, sprite_t* b, int* angle, int a_prev_left, int a_prev_top);
#endif /* INC_SPRITE_H_ */
