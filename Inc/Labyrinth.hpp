#pragma once
#include"Position.hpp"
#include<array>

extern "C"{
#include"GlobalParameter.h"
}
#define MAP_W_SIZE (MAP_X_SIZE)
#define MAP_H_SIZE (MAP_Y_SIZE)

class Labyrinth{
	private:	
	
	
	//w_wallはx軸平行の壁，h_wallはy軸平行の壁

	std::array<int,MAP_W_SIZE * (MAP_H_SIZE + 1)> w_walls;
	std::array<int,MAP_H_SIZE * (MAP_W_SIZE + 1)> h_walls;

/*
	// X_SIZE * (Y_SIZE + 1)	| の壁
	vector<int> w_wall_infomation;
	// Y_SIZE * (X_SIZE + 1)	- の壁
	vector<int> h_wall_infomation;
*/

	int* wall_mapping(int pos_x, int pos_y, int wall_dir);
	int* wall_mapping(Position pos, int wall_dir);
	int* wall_mapping(Position pos, Position pos_dir);
	
	public:
	int w_size,h_size;
	Labyrinth(int ws,int hs);
	
	void reset();
	
	void addWall(Position p1,Position p2);
	void addWall(Position p,int wall_dir);
	
	void deleteWall(Position p1,Position p2);
	void deleteWall(Position p,int wall_dir);
	
	int isThereWall(Position p1, Position p2);

	int isThereWall(Position p, int dir);
	
	bool isInLabyrinth(Position p){
		return (p.x >= 0 && p.x < w_size && p.y >= 0 && p.y < h_size);
	}

	void loadAllWallsFromBits(uint8_t* wall_bits, uint8_t ws, uint8_t hs){
		
		w_size = ws;
		h_size = hs;
		
		const int w_wall_size = w_size * (h_size + 1);
		const int h_wall_size = h_size * (w_size + 1);
		const int all_wall_size = w_wall_size + h_wall_size;

		//wall_bitsには |w_walls[0~map_xsize],h_walls[0~map_ysize]|でデータが入っているものとする
		//バイトごとに下位bitから順番に壁情報を読みだす。バイト全部読んだら隣のバイトにいく
		for(int i = 0; i < w_wall_size; i++){
			w_walls[i] = (wall_bits[i / 8]>>(i % 8)) & 0x01;
		}
		for(int i = w_wall_size; i < all_wall_size; i++){
			h_walls[i - w_wall_size] = (wall_bits[i / 8] >> (i % 8) ) & 0x01;
		}
	}

	void writeAllWallToBits(uint8_t* wall_bits, uint8_t* ws, uint8_t* hs){
		
		const int w_wall_size = w_size * (h_size + 1);
		const int h_wall_size = h_size * (w_size + 1);
		const int all_wall_size = w_wall_size + h_wall_size;

		for(int i = 0; i < w_wall_size; i++){
			wall_bits[i / 8] |= (w_walls[i]<<(i % 8)) & 0x01;
		}
		for(int i = w_wall_size; i < all_wall_size; i++){
			wall_bits[i / 8] |= (h_walls[i - w_wall_size]<<(i % 8)) & 0x01;
		}

		*ws = (uint8_t)w_size;
		*hs = (uint8_t)h_size;
	}

};

int pos_dir_to_wall_dir(Position pos_dir);
