extern "C"
{
    #include"MovementManager.h"
    #include"PIDController.h"
    #include"GlobalObjects.h"

    // PIDParams(Kp, Ki, Kd, Ts, Td, e, err_sum)
    PIDParams x_pid     = {0.01f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f};
    PIDParams y_pid     = {0.3f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f};
    PIDParams theta_pid = {0.5f, 0.001f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f};
}
#include"Factory.hpp"
#include<array>
#include"Labyrinth.hpp"

//スタート地点で、Y軸が前方、X軸が右方向、右回りが回転の正方向
PositionF robot_start_pos(90.0f,6.0f,0.0f);

StraightPath s_path(800.0f, 300.0f, 0.0f, 500.0f, 1000.0f);
Swivelling turn_path(6.28f, 6.28f, 100.0f);
Path* path_arr[PATH_INDEX_MAX] = {&s_path, &turn_path};


//Action Path
StraightPath go_straight_path(180.0f, 300.0f, 300.0f, 350.0f, 1000.0f);
Swivelling turn_right_path(MY_PI / 2.0f, MY_PI * 0.5f, 100.0f);
Swivelling turn_left_path(-MY_PI / 2.0f, MY_PI * 0.5f, 100.0f);
Swivelling turn_right_back_path(MY_PI, MY_PI * 0.5f, 100.0f);

PathController path_controller(&s_path,x_pid, y_pid, theta_pid, robot_start_pos);

// Search AI(0,0) - (15,15)
std::array<Position,4> gpss = {Position(7,7),Position(7,8),Position(8,7),Position(8,8)};
Labyrinth laby(16,16);
KizunaAI kizuna_ai(gpss, laby,Position(0,0,1));

ActionManager action(&movement,&measurement,&path_controller,&kizuna_ai);
// debug variant
float debug_val_u_left = 0;
float debug_val_u_right = 0;
float debug_val_dist;

Position debug_pos(0,0,1);

void FactoryInitialize(){

}

ActionManager* GetActionManager(){return &action;}

PathController* GetPathController(){ return &path_controller;}

KizunaAI* GetKizunaAI(){ return &kizuna_ai;}

Path* GetStraightPath(){return &go_straight_path;}
Path* GetTurnRightPath(){return &turn_right_path;}
Path* GetTurnLeftPath(){return &turn_left_path;}
Path* GetTurnRightBackPath(){return &turn_right_back_path;}

PositionF GetRobotStartPosition(){return robot_start_pos;}