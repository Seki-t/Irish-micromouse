#pragma once
#include "Position.hpp"
#include "Labyrinth.hpp"
#include "Wall.hpp"
#include"HeapArray.hpp"

extern "C"{
#include"IrSensor.h"
#include"GlobalParameter.h"
}

//基本的にFootクラスはKizunaAIクラス内のみで使う
struct Foot{
	int cost;
	Position pos;
	
	Foot(){}
	Foot(int c, Position p) : cost(c), pos(p) {}
	
	//>にはconstつけないと，stlで使えない

	bool operator > (Foot f)const{
		return cost > f.cost;
	}
	bool operator >= (Foot f)const{
		return cost >= f.cost;
	}
	bool operator < (Foot f)const{
		return cost < f.cost;
	}
	bool operator <= (Foot f)const{
		return cost <= f.cost;
	}
};


//現在持っている地図情報から，ゴールを探索するために次に向かう方向を示すAI
//地図情報から最短経路を算出するものではない．
//それは，他のクラスにやらせたい

//歩数マップ探索

// 本番ではglobalな定数フィールドで定義する
#define FIELD_SIZE (MAP_W_SIZE * MAP_H_SIZE)

//この数値は恐らく固定
#define GOAL_AREA_NUM 4

class KizunaAI
{

private:
	//このコストはAI側で持つべき
	// size = X_SIZE * Y_SIZE

	std::array<int, FIELD_SIZE> nodes_cost;
	std::array<Position, GOAL_AREA_NUM> goal_positions;
	Labyrinth laby_map;
	HeapArray<Foot> heap;

	Position now_ref_pos;
	Position now_pos;

	bool pos_move_flag;


	//private function
	size_t convertIndex(Position p)
	{
		return (p.x + p.y * laby_map.w_size);
	}

	//Map上における自己位置を更新する。移動が発生したらtrue
	bool updatePosition(PositionF now_pos_f);

	//エリアの中心(走行タスク切り替え点)にいるかどうか
	bool isOnActionChangePosition(PositionF pos);

	bool isOnWallCheckPosition(PositionF now_pos_f);

public:
	KizunaAI(std::array<Position,GOAL_AREA_NUM> gpss, Labyrinth &l_map, Position np) : laby_map(l_map), now_pos(np),pos_move_flag(false)
	{

		for (int i = 0; i < GOAL_AREA_NUM; i++)
		{
			goal_positions[i] = gpss[i];
		}
	}

	Position calcNextPos();

	//センサ情報から頭の中の地図を更新
	//地図を更新って，結局はどこに壁があるかっていう情報を地図に書いていくお仕事だから，壁情報を受け取る

	Position getNowRefPos()
	{
		return now_ref_pos;
	}

	void updateMap(Wall* new_walls, int size)
	{
		for (int i = 0; i < size; i++)
		{
			laby_map.addWall(new_walls[i].pos, new_walls[i].dir);
		}
	}

	std::array<int,FIELD_SIZE> getNodesCost()
	{
		return nodes_cost;
	}

	Labyrinth getMap()
	{
		return laby_map;
	}

	bool updateMapAndPosition(const IRSensorInfo* sensor_info,PositionF now_pos);

	bool isMoveNextPos(){return pos_move_flag;}

	int getNextDirection()
	{

		const int direction_matrix[4][4] = 
		{
			{0,1,2,3},
			{3,0,1,2},
			{2,3,0,1},
			{1,2,3,0}
		};
		return direction_matrix[now_pos.theta][now_ref_pos.theta];
	}

	bool isMovedOnTheMap()
	{
		return pos_move_flag;
	}
	
	//記憶した情報を初期状態に戻す
	void reset(){
		laby_map.reset();
	}	
};

//		{
//			Position next_pos(0,0,0);
//
//			Position migimawari_wall_directions[4] = {Position(1,0),Position(0,1),Position(-1,0),Position(0,-1)} ;
//			//
//			// const int migi_map[] = {1,0,3,1};
//
//			int start_migite_dir = (now_pos.theta + 3) % 4;// now_pos.theta = 0なら，今(1,0)を向いているので，start_migite_dir = 3になって，(0,-1)方向が右手になる
//
//			for(int i = start_migite_dir; i < 4 + start_migite_dir; i++){
//				std::cout<<"i = "<<i<<std::endl;
//				//壁があるかを聞くより，移動可能かどうかを聞いた方が，後々汎用的になるかもしれない
//				if( ( !lab.isThereWall( now_pos, i % 4 ) ) && lab.isInLabyrinth(now_pos + migimawari_wall_directions[i % 4]) ){
//					next_pos = now_pos + (migimawari_wall_directions[i % 4]);
//					next_pos.theta = i  % 4;	//今進んだ方向を次の姿勢角とする
//					break;
//				}
//			}
//			return next_pos;
//		}
