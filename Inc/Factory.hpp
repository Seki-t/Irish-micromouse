#pragma once
#include"Position.hpp"
#include"PathController.hpp"
#include"ActionManager.hpp"
#include"KizunaAI.hpp"
#include"Path.hpp"
#define PATH_INDEX_MAX 2
void FactoryInitialize();

PathController* GetPathController();

ActionManager* GetActionManager();

KizunaAI* GetKizunaAI();

extern float debug_val_u_left;
extern float debug_val_u_right;
extern float debug_val_dist;
extern Path* path_arr[];
extern Position debug_pos;

Path* GetStraightPath();
Path* GetTurnRightPath();
Path* GetTurnLeftPath();
Path* GetTurnRightBackPath();

PositionF GetRobotStartPosition();