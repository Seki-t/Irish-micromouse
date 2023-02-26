#pragma once
#include<cmath>
#include"GlobalParameter.h"

//dir判定のための定数
const int k_pi_range = 15;
const int k_pi180 = 180;
const int k_pi_2 = 90;
const int k_pi3_2 = 270;

//thetaは0 = -90度, 1 : 0deg, 2 : 90deg, 3 : 180deg付近である。それ以外は不定 : -1または前回値保持とする
struct Position{

	int x,y,theta;

	Position(){}
	Position(int _x,int _y, int _theta) : x{_x}, y{_y}, theta{_theta} {}

	Position(int _x,int _y) : x{_x}, y{_y}, theta{0} {}

	Position moved_position(int move_x,int move_y,int move_theta)const {
		return Position(x + move_x, y + move_y, theta + move_theta);
	}

	Position moved_position(const Position& dir_pos)const {
		return Position(x + dir_pos.x,y + dir_pos.y, theta + dir_pos.theta);
	}
	Position moved_position(Position& dir_pos)const {
		return Position(x + dir_pos.x,y + dir_pos.y, theta + dir_pos.theta);
	}

	Position moved_position(int wall_dir)const {
		return moved_position( convertDirection( wall_dir ));
	}

	//wallの0,1,2,3を

	Position convertDirection(int wall_direction)const {/*{{{*/
		switch(wall_direction){
			case 0:
				return Position(1,0);
				break;
			case 1:
				return Position(0,1);
				break;
			case 2:
				return Position(-1,0);
				break;
			case 3:
				return Position(0,-1);
				break;
		}
		return Position(0,0);
	}/*}}}*/

	Position operator +(const Position& pos)const {
		return Position(x + pos.x, y + pos.y, theta + pos.theta);
	}

	Position operator -(const Position& pos)const {
		return Position(x - pos.x, y - pos.y, theta - pos.theta);
	}

	Position& operator += (const Position& pos){
		this->x += pos.x;
		this->y += pos.y;
		this->theta += pos.theta;
		return *this;
	}

	Position& operator -= (const Position& pos){
		this->x -= pos.x;
		this->y -= pos.y;
		this->theta -= pos.theta;
		return *this;
	}

	Position& operator =(const Position& pos){
		x = pos.x;
		y = pos.y;
		theta = pos.theta;
		return *this;
	}

	bool operator !=(const Position& pos)const {
		return ((x != pos.x) || (y != pos.y) || (theta != pos.theta ));
	}

	bool operator ==(const Position& pos)const {
		return ((x == pos.x) && (y == pos.y) && (theta == pos.theta));
	}
	
	bool equalPos(Position pos)const {
		return ((x == pos.x) && (y == pos.y) );
	}

	int manhattan(Position p1, Position p2){
		return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
	}
};


struct PositionF{

	float x,y,theta;

	PositionF(){}
	PositionF(float _x,float _y, float _theta) : x{_x}, y{_y}, theta{_theta} {}

	PositionF(float _x,float _y) : x{_x}, y{_y}, theta{0} {}

	PositionF moved_position(float move_x,float move_y,float move_theta)const {
		return PositionF(x + move_x, y + move_y, theta + move_theta);
	}

	PositionF moved_position(const PositionF& dir_pos)const {
		return PositionF(x + dir_pos.x,y + dir_pos.y, theta + dir_pos.theta);
	}
	PositionF moved_position(PositionF& dir_pos)const {
		return PositionF(x + dir_pos.x,y + dir_pos.y, theta + dir_pos.theta);
	}

	PositionF operator +(const PositionF& pos)const {
		return PositionF(x + pos.x, y + pos.y, theta + pos.theta);
	}

	PositionF operator -(const PositionF& pos)const {
		return PositionF(x - pos.x, y - pos.y, theta - pos.theta);
	}

	PositionF& operator += (const PositionF& pos){
		this->x += pos.x;
		this->y += pos.y;
		this->theta += pos.theta;
		return *this;
	}

	PositionF& operator -= (const PositionF& pos){
		this->x -= pos.x;
		this->y -= pos.y;
		this->theta -= pos.theta;
		return *this;
	}

	PositionF& operator =(const PositionF& pos){
		x = pos.x;
		y = pos.y;
		theta = pos.theta;
		return *this;
	}

	bool operator !=(const PositionF& pos)const {
		return ((x != pos.x) || (y != pos.y) || (theta != pos.theta ));
	}

	bool operator ==(const PositionF& pos)const {
		return ((x == pos.x) && (y == pos.y) && (theta == pos.theta));
	}
	
	bool equalPos(PositionF pos)const {
		return ((x == pos.x) && (y == pos.y) );
	}

	Position toPosition()
	{
		const int x = (int)(MAP_X_SIZE * (this->x / FIELD_X_SIZE));
		const int y = (int)(MAP_Y_SIZE * (this->y / FIELD_Y_SIZE));

		int attitude_deg = int(180.0f * (this->theta / MY_PI)); //角度はdir形式にするので、一時変数に代入

		// 姿勢角を180 ~ -180の範囲に変換(-360 ~ 360限定なので、直す)
		//これだとここの関数内だけで修正することになるので、全体で角度情報扱ってるところでも、忘れないように
		if (attitude_deg > 180)
		{
			attitude_deg = attitude_deg - 360;
		}
		else if (attitude_deg < -180)
		{
			attitude_deg = attitude_deg + 360;
		}

		//角度のdir化
		int mouce_front_dir;

		//1 : y軸正方向, 2 : x軸正, 0 : x軸負 , 3 : y軸負, -1 : 不定
		// 0,90,180,270に近い角度の時でないと壁情報追加しないようにしたい
		if (attitude_deg > -k_pi_range && attitude_deg < k_pi_range)
		{
			mouce_front_dir = 1;
		}
		else if (attitude_deg > k_pi_2 - k_pi_range && attitude_deg < k_pi_2 + k_pi_range)
		{
			mouce_front_dir = 2;
		}
		else if (attitude_deg > -k_pi_2 - k_pi_range && attitude_deg < -k_pi_2 + k_pi_range)
		{
			mouce_front_dir = 0;
		}
		else if (attitude_deg > k_pi180 - k_pi_range || attitude_deg < -k_pi180 + k_pi_range)
		{
			mouce_front_dir = 3;
		}
		else
		{
			//角度が変化中は前と同じ角度とするため、不定を示す-1を返す
			mouce_front_dir = -1;
		}

		return Position(x,y,mouce_front_dir);	
	}
};
