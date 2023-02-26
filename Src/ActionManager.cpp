extern "C"
{
#include "GlobalParameter.h"
#include "MovementManager.h"
#include "Measurement.h"
#include "GlobalObjects.h"
#include"UartBuffer.h"
}
#include "Factory.hpp"
#include "ActionManager.hpp"
/* State = Fowardの時に実行*/
//探索走行中に自己位置も使えるなら、マスの中心を走るようthetaを制御する

ActionManager::ActionManager(Movement *_mover,Measurement* _measure, PathController *pc, KizunaAI *ai) : 
                                                                                    mover(_mover),
                                                                                    measure(_measure),
                                                                                    path_con(pc),
                                                                                    k_ai(ai)
{
    PIDInitialize(&(this->wall_run_pid_left), 0.0001, 0, 0, MORTOR_CONTROL_TS, 0);
    PIDInitialize(&(this->wall_run_pid_right), 0.0001, 0, 0, MORTOR_CONTROL_TS, 0);
    
    //起動時の初期状態のパス・位置をいれる 
    //この時点ではmeasurementは初期化されてないので、getpositionしてはいけない
    //起動時は前進から始まるので、FOWARD
    //ダミーとしてs_pathが入っているが、これは実行されることはない
    state = ActionState::ACTION_FOWARD;
    //path_con->resetStart(GetStraightPath(), GetRobotStartPosition());
}

//移動した情報を関連でオブジェクト参照して渡すのか、MainTask内で引数として渡すのかがごっちゃになっている。
//どちらかに方針を定めるべき

void ActionManager::update(IRSensorInfo ir_info)
{

    // update sensor

    MortionParams mp = MeasurementGetPosition(measure);
    PositionF now_pos_f(mp.x, mp.y, mp.theta);

    // move
    // 1 times in 100ms
    switch (state)
    {

    // task wall runの場合
    case ActionState::ACTION_FOWARD:
        g_action_direction = 'F';
        // ActionFowardじゃないと、前壁チェックしちゃダメ

        // 次の行動へ遷移
        //  path走行を終えたかどうかでなく、次のマスに移動したかどうかで遷移判断する
        if (k_ai->isMoveNextPos())
        {
            PIDReset(&wall_run_pid_left);
            PIDReset(&wall_run_pid_right);

            switch (k_ai->getNextDirection())
            {
            case 0:
                UartBufferPush('f');
                state = ActionState::ACTION_FOWARD;
                path_con->resetStart(GetStraightPath(), now_pos_f);
                break;

            case 1:
                UartBufferPush('l');
                state = ActionState::ACTION_LEFT_TURN;
                path_con->resetStart(GetTurnLeftPath(), now_pos_f);
                break;

            case 2:
                UartBufferPush('b');
                state = ActionState::ACTION_RIGHT_BACK_TURN;
                path_con->resetStart(GetTurnRightBackPath(), now_pos_f);
                break;

            case 3:
                UartBufferPush('r');
                state = ActionState::ACTION_RIGHT_TURN;
                path_con->resetStart(GetTurnRightPath(), now_pos_f);
                break;

            default:
                state = ActionState::ACTION_STOP;
                // path_con resetしないので停止するはず
            }
        }
        break;

    case ActionState::ACTION_LEFT_TURN:
        g_action_direction = 'L';
        if(path_con->isGoal())
        {
            UartBufferPush('f');
            path_con->resetStart(GetStraightPath(), now_pos_f);
            state = ActionState::ACTION_FOWARD;
        }
        break;

    case ActionState::ACTION_RIGHT_TURN:
        g_action_direction = 'R';
        if (path_con->isGoal())
        {
            UartBufferPush('f');
            path_con->resetStart(GetStraightPath(), now_pos_f);
            state = ActionState::ACTION_FOWARD;
        }
        break;

    case ActionState::ACTION_RIGHT_BACK_TURN:
        g_action_direction = 'B';
        if (path_con->isGoal())
        {
            UartBufferPush('f');
            path_con->resetStart(GetStraightPath(), now_pos_f);
            state = ActionState::ACTION_FOWARD;
        }
        break;

    default:
        g_action_direction = 'D';
        if (path_con->isGoal())
        {
            UartBufferPush('f');
            path_con->resetStart(GetStraightPath(), now_pos_f);
            state = ActionState::ACTION_FOWARD;
        }
    }

    // do
    switch (state)
    {
    case ActionState::ACTION_FOWARD:
        taskWallRun();
        break;
    default:
        MovementReference mr = path_con->calcNextReference(
            PositionF(now_pos_f.x, now_pos_f.y, now_pos_f.theta));
        MovementManagerSetReference(mover, mr.V, mr.W);
        g_mr_v = mr.V;
        break;
    }
}

void ActionManager::taskWallRun()
{

    //壁センサの目標値
    const int32_t left_wall_distance_reference = 22000;
    const int32_t right_wall_distance_reference = 22000;
    const int32_t wall_distance_min = 15000;

    const float constant_velocity = 150.0f;

    float u = 0;

    //切り替え制御するなら積分リセットなど考えないといけない
    //積分値があるから同じpid構造体は使えない
    if (ir_info.is_wall_ls && ir_info.is_wall_rs)
    {

        float u_left = PIDOutput(&wall_run_pid_left, -left_wall_distance_reference, -ir_info.left_side);
        float u_right = PIDOutput(&wall_run_pid_right, right_wall_distance_reference, ir_info.right_side);
        u = (u_left + u_right) / 2.0f;
        debug_val_u_left = u_left;
        debug_val_u_right = u_right;
    }
    if (ir_info.is_wall_ls)
    {
        u = PIDOutput(&wall_run_pid_left, -left_wall_distance_reference, -ir_info.left_side);
        PIDReset(&wall_run_pid_right);
        debug_val_u_left = u;
    }
    else if (ir_info.is_wall_rs)
    {
        u = PIDOutput(&wall_run_pid_right, right_wall_distance_reference, ir_info.right_side);
        PIDReset(&wall_run_pid_left);
        debug_val_u_right = u;
    }

    float calc_v_left = constant_velocity + DISTANCE_BETWEEN_WHEELS * u;
    float calc_v_right = constant_velocity - DISTANCE_BETWEEN_WHEELS * u;

    if (calc_v_left < 0.0f)
        calc_v_left = 0.0f;
    if (calc_v_right < 0.0f)
        calc_v_right = 0.0f;

    u = (calc_v_left - calc_v_right) / (2.0f * DISTANCE_BETWEEN_WHEELS);

    MovementManagerSetReference(mover, constant_velocity, u);
}
void ActionManager::taskStop()
{
    MovementManagerSetReference(mover, 0, 0);
}

void ActionManager::taskLeftTurn()
{
    MovementManagerSetReference(mover, 0, -3.5);
}

void ActionManager::taskRightTurn()
{
    MovementManagerSetReference(mover, 0, 3.5);
}
