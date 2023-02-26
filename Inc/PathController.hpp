#pragma once
#include"Path.hpp"
#include"PIDController.h"
#include"Measurement.h"
#include"GlobalParameter.h"

#define PATH_QUEUE_MAX 10
#define GOAL_JUDGE_DISTANCE 1000.0f
#define GOAL_JUDGE_ATTIDUE_DIFF 0.2f
extern float debug_val_dist;
class PathController{
    private:
    PositionF start_pos;
    PositionF end_pos;
    PIDParams x_controller;
    PIDParams y_controller;
    PIDParams theta_controller;
    Path* path;
    bool is_goal;
    
    bool goalCheck(PositionF now_pos, PositionF end_pos){

        PathType now_type = path->GetPathType(); 
        if(now_type == PathType::POSITION_COMPENSATE){
            float dist =  ( (now_pos.x - end_pos.x) * (now_pos.x - end_pos.x) + (now_pos.y - end_pos.y) * (now_pos.y - end_pos.y));
            debug_val_dist = dist;
            return dist < GOAL_JUDGE_DISTANCE;
        }
        else if(now_type == PathType::ATTITUDE_COMPENSATE){
            float dist = fabs(now_pos.theta - end_pos.theta);
            return dist < GOAL_JUDGE_ATTIDUE_DIFF;
        }
        return path->isGoal();
    }

    public:
    PathController(Path* _path, PIDParams x_c, PIDParams y_c, PIDParams t_c, PositionF s_p) : 
    start_pos(s_p),
    x_controller(x_c),
    y_controller(y_c),
    theta_controller(t_c),
    path(_path),
    is_goal(false)
    {
        end_pos = start_pos + path->GetEndPos();

    }

    MovementReference compansatePosition(PositionF now_pos, PositionF ref_pos){
        
        float e_x = ref_pos.x - now_pos.x;   
        float e_y = ref_pos.y - now_pos.y;   

        // upper : x, -> : y, right rotate :theta
        float e_x_robot = e_x * cosf(now_pos.theta) - e_y * sinf(now_pos.theta);
        float e_y_robot = e_x * sinf(now_pos.theta) + e_y * cosf(now_pos.theta);

        MovementReference ref;
        ref.V = PIDOutput(&y_controller, e_y_robot, 0);
        ref.W = PIDOutput(&x_controller, e_x_robot, 0);

        float calc_v_left = ref.V + DISTANCE_BETWEEN_WHEELS * ref.W;
        float calc_v_right = ref.V - DISTANCE_BETWEEN_WHEELS * ref.W;

        if (calc_v_left < 0.0f)
          calc_v_left = 0.0f;
        if (calc_v_right < 0.0f)
          calc_v_right = 0.0f;

        ref.W = (calc_v_left - calc_v_right) / (2.0f * DISTANCE_BETWEEN_WHEELS);
        return ref;
    }

    MovementReference compansateAttitude(PositionF now_pos, PositionF ref_pos){
        
        MovementReference ref;
        ref.V = 0;
        ref.W = PIDOutput(&theta_controller, ref_pos.theta, now_pos.theta);
        //if(ref.W > 0.05f && ref.W < 0.3f)ref.W = 0.3f;
        //if(ref.W < -0.05f && ref.W > -0.3f)ref.W = -0.3f;
        return ref;
    }

    MovementReference calcNextReference(PositionF now_pos){


        if(path == nullptr)return MovementReference(0,0,PositionF(0,0,0));

        path->UpdateReference(); 
        //world coord        
        PositionF ref_pos = path->GetReference().pos + start_pos;

        if(path->isGoal()){
            if(!is_goal){
                is_goal = goalCheck(now_pos, ref_pos);
            }
            if(is_goal)return path->GetReference();
        }

        MovementReference ref; 
        MovementReference comp;
        switch(path->GetPathType()){

            case PathType::POSITION_COMPENSATE:

                //robot coord
                comp = compansatePosition(now_pos, ref_pos);
                ref.V =  path->GetReference().V + comp.V;
                ref.W =  path->GetReference().W + comp.W;
                break;

            case PathType::ATTITUDE_COMPENSATE:

                comp = compansateAttitude(now_pos, ref_pos);
                //robot coord
                ref.V = 0;//path->GetReference().V;
                ref.W = path->GetReference().W + comp.W;
                //ref.W = comp.W;
                break;
            default:
                //robot coord
                ref.V =  path->GetReference().V;
                ref.W =  path->GetReference().W;
                break;
        }
        return ref;
    }


    bool isGoal(){return is_goal;}

    void resetStart(Path* _path, PositionF s_p){
        PIDReset(&x_controller);
        PIDReset(&y_controller);
        PIDReset(&theta_controller);
        path = _path;
        path->reset();
        start_pos = s_p;
        end_pos = start_pos + path->GetEndPos();
        is_goal = false;
    }

};
