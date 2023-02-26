#pragma once

#include"Position.hpp"

//姿勢角dirは0,1,2,3とx正軸から左回りを使う
struct Wall{
	Position pos;
	int dir;

	Wall() : 
	pos(0,0),
	dir(0)
	{}

	Wall(int x, int y,int d):
	pos(x,y),
	dir(d)
	{}
};




