#include <algorithm>
#include <climits>
#include"HeapArray.hpp"
#include"KizunaAI.hpp"
using namespace std;

#define INF INT_MAX/3
#define REP(i,n) for(int i=0;i<n;i++)


Position KizunaAI::calcNextPos(){
	
	fill(nodes_cost.begin(),nodes_cost.end(),10000); //全costをINFに
	
	heap.reset();	
	//std::queue<foot> que;	
	
	//goalを歩数マップ0にして探索開始位置に
	
	for(Position& gps : goal_positions){
		nodes_cost[convertIndex(gps)] = 0;
		heap.push(Foot(0,gps));
	}	

	
	int excute_count = 0;
	while(heap.getSize() != 0){

		Foot p = heap.pop();
		Position selected_pos = p.pos;

		for(int i = 0; i < 4; i++){
			if( !laby_map.isThereWall(selected_pos, i) ){
				Position new_pos = selected_pos.moved_position(i);
				int idx = convertIndex(new_pos);
				
				if(nodes_cost[idx] > p.cost + 1){
					nodes_cost[idx] = p.cost + 1;
					heap.push(Foot(p.cost + 1, new_pos));
					excute_count++;
				}
			}
		}
	}
	
	Position next_pos;
	int now_cost = 1000000;
	//移動方向の決定(現在の進行方向を有線)
	
	int priority_direction[4] = {0,1,3,2};

	REP(i,4){
		
		int dir_candidate = priority_direction[(now_pos.theta + i) % 4 ];

		if( !laby_map.isThereWall(now_pos,dir_candidate)){
			
			if(nodes_cost[convertIndex(now_pos.moved_position(dir_candidate))] < now_cost){

				next_pos = now_pos.moved_position(dir_candidate);
				next_pos.theta = dir_candidate % 4;

				now_cost = nodes_cost[convertIndex(next_pos)];
			}
		}
	}

	if (next_pos.theta != now_pos.theta) {
		next_pos.x = now_pos.x;
		next_pos.y = now_pos.y;
	}

	now_ref_pos = next_pos;
	return next_pos;
}

bool KizunaAI::updatePosition(PositionF now_pos_f){
	
	Position now_pos_on_map = now_pos_f.toPosition();
	if(now_pos_on_map.theta == -1) now_pos_on_map.theta = now_pos.theta;

	//マップ上位置が前回から変化していた OR 90度旋回しているなら、移動フラグをtrueに	
	bool p_m_flag = (now_pos_on_map.x != now_pos.x) || (now_pos_on_map.y != now_pos.y) || (now_pos_on_map.theta != now_pos.theta);

	//移動先のマスに更新
	now_pos.x = now_pos_on_map.x;
	now_pos.y = now_pos_on_map.y;
	now_pos.theta = now_pos_on_map.theta;

	return p_m_flag;
}

bool KizunaAI::isOnActionChangePosition(PositionF now_pos_f){

	const int err_center_x = static_cast<int>(now_pos_f.x) % 180 - 90;
	const int err_center_y = static_cast<int>(now_pos_f.y) % 180 - 90;

	return (err_center_x * err_center_x <= 100) && (err_center_y * err_center_y <= 100);
}

bool KizunaAI::isOnWallCheckPosition(PositionF now_pos_f){

	//マスの境界線から何mm進んだところで検知を実施するか
	const float sence_start_dist = 90 - 70;
	
	const int err_center_x = static_cast<int>(now_pos_f.x) % 180;
	const int err_center_y = static_cast<int>(now_pos_f.y) % 180;

	bool should_check_wall = false;
	switch(now_pos.theta){
		//上
		case 1:
		should_check_wall = err_center_y > (180 - sence_start_dist);
		break;

		//右
		case 2:
		should_check_wall = err_center_x > (180 - sence_start_dist);
		break;

		//左
		case 0:
		should_check_wall = err_center_x < sence_start_dist;
		break;

		//下
		case 3:
		should_check_wall = err_center_y < sence_start_dist;
		break;

		default:
		should_check_wall = false;
	}

	return true;
}

//内部で多数の処理を実施しているので、関数を切り分けて分かりやすくしたい
bool KizunaAI::updateMapAndPosition(const IRSensorInfo* sensor_info,PositionF now_pos_f){

	pos_move_flag = false;

	//マップ上の自己位置を更新。位置・姿勢が変化してたらtrue
	bool isMovedOnMap = updatePosition(now_pos_f);

	//マスの境界線から何mm進んだところで検知を実施するか
	const float sence_start_dist = 90 - 70;

	//今いるマスの中心物理座標
	const float map_center_x = float(now_pos.x * MAP_CHIP_SIZE + MAP_CHIP_SIZE / 2);
	const float map_center_y = float(now_pos.y * MAP_CHIP_SIZE + MAP_CHIP_SIZE / 2);

	//壁をみるポイントにいるかどうか（現在範囲にしてるけど、ほんとは一回だけにしたい)	
	//0,90,180,270に近い角度の時でないと壁情報追加しないようにしたい(実際不要な可能性が高いため、抜いておく)
	bool should_check_wall = false;
	
	switch(now_pos.theta){
		case 1:
		should_check_wall = now_pos_f.y > (map_center_y + sence_start_dist);
		break;

		case 2:
		should_check_wall = now_pos_f.x < (map_center_x + sence_start_dist);
		break;

		case 0:
		should_check_wall = now_pos_f.x > (map_center_x - sence_start_dist);
		break;

		case 3:
		should_check_wall = now_pos_f.y < (map_center_y - sence_start_dist);
		break;

		default:
		should_check_wall = false;
	}
	
	bool should_check_wall_front = false;
	//タイヤ軸の中心が自己位置中心である。
	//そうするとその場回転した時にX-Y変わらずthetaだけ変化する	
	switch(now_pos.theta){
		case 1:
		should_check_wall_front = now_pos_f.y > (map_center_y);
		break;

		case 2:
		should_check_wall_front = now_pos_f.x < (map_center_x + sence_start_dist);
		break;

		case 0:
		should_check_wall_front = now_pos_f.x > (map_center_x - sence_start_dist);
		break;

		case 3:
		should_check_wall_front = now_pos_f.y < (map_center_y - sence_start_dist);
		break;

		default:
		should_check_wall_front = false;
	}

	//壁情報の更新をチェック
	//自分の進行方向の次のマスの壁を追加する (次のマスの壁情報が次のマスに入る前に分かってないと、なめらかに走行できない)
	//ていうか今の実装だと次のマスに切り替わった瞬間に旋回動作を始めてしまうので、中心位置を保てない
	//なめらかに走るつもりが無いなら、かならず真ん中まで移動してから次の行動計画に移らないといけない
	//移動している最中に壁のチェックはする。1マス移動終わったら次のタスクに移る
	//いつでも見るようにすると、意図しないタイミングで見てしまう恐れがあるので、限定したい
	//限定することで、閾値を狭めたい
	bool is_wall_updated = false;

	//自分の進行方向のdir番号(indexになる)に対する左壁・右壁のdir番号
	//インライン関数にしてもいいかも
	const int mouce_left_dirs[4] = {1,2,3,0};
	const int mouce_right_dir[4] = {3,0,1,2};

	//TODO : マウスの前方方向はthetaと一致するはず
	int mouce_front_dir = now_pos.theta;
	Position front_pos = now_pos.moved_position(mouce_front_dir);


	//前壁は横壁と同じタイミングで見ると、距離遠すぎるかも
	if (should_check_wall){
		//前壁の発見
		if (sensor_info->is_wall_lf && sensor_info->is_wall_rf){
			laby_map.addWall(front_pos, mouce_front_dir);
			is_wall_updated = true;
		}
		//左壁の発見
		if(sensor_info->is_wall_ls){
			laby_map.addWall(front_pos,mouce_left_dirs[mouce_front_dir]);
			is_wall_updated = true;
		} 
		//右壁の発見
		if(sensor_info->is_wall_rs){
			 laby_map.addWall(front_pos,mouce_right_dir[mouce_front_dir]);
			 is_wall_updated = true;
		}
	}

	if(should_check_wall_front){
		//前壁の発見
		if (sensor_info->is_wall_lf && sensor_info->is_wall_rf){
			laby_map.addWall(front_pos, mouce_front_dir);
			is_wall_updated = true;
		}
	}

	//壁更新かマップ移動の発生時trueを返す
	pos_move_flag = isMovedOnMap;
	return pos_move_flag;
}
/* 壁を発見したときにaddWall毎回やるけど、すでに知ってる壁なら不要では？
addWallが知ってる壁追加しても問題ない構造ならok */