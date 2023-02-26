#pragma once

extern "C"
{
#include "IrSensor.h"
#include "PIDController.h"
#include "MovementManager.h"
}
#include"PathController.hpp"
#include"KizunaAI.hpp"

enum class ActionState
{
    ACTION_FOWARD,
    ACTION_STOP,
    ACTION_BACK,
    ACTION_RIGHT_TURN,
    ACTION_LEFT_TURN,
    ACTION_RIGHT_BACK_TURN,
    ACTION_LEFT_BACK_TURN
};

class ActionManager
{
    ActionState state;
    PIDParams wall_run_pid_left, wall_run_pid_right;
    IRSensorInfo ir_info;
    Movement *mover;
    Measurement* measure;
    PathController* path_con;
    KizunaAI* k_ai;
    void taskWallRun();
    void taskRightTurn();
    void taskLeftTurn();
    void taskStop();

public:
    ActionManager(Movement* _mover,Measurement* _measure, PathController* pc, KizunaAI* ai);
    void update(IRSensorInfo ir_info);
};
