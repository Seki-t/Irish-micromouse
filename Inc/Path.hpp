#pragma once
#include"Position.hpp"
#include"TrapezoidAccel.hpp"

/* 今出しているべき速度と位置(ロボット座標系)を返す */
struct MovementReference{
    float V,W;
    PositionF pos;

    MovementReference(){}
    MovementReference(float v, float w, PositionF p):
    V(v),W(w),pos(p){}
};

enum class PathType{
    POSITION_COMPENSATE,
    ATTITUDE_COMPENSATE,
    NONE
};

class Path{
    protected:
        PositionF start_position; 
        PositionF end_position;
        PositionF now_position;
        int now_time;
        MovementReference mr;
        PathType p_type;
    public:

    Path(PositionF st, PositionF en) :
        start_position(st),
        end_position(en),
        now_position(st),
        now_time(0),
        mr(0,0,PositionF(0,0,0)),
        p_type(PathType::NONE)
    {

    }

    virtual void UpdateReference() = 0;
    virtual bool isGoal() = 0;
    virtual void reset() = 0;

    MovementReference GetReference(){return mr;}
    PathType GetPathType(){return p_type;}
    PositionF GetEndPos(){return end_position;}

};

/* とりあえず、今向いている方向に走る完全に直線のみにしてみる */
    

class StraightPath : public Path{
    private:
        TrapezoidAccel<float> trape;
        float L;
        float v_start;  //reset後の状態をconstructer呼び出し後と同一状態にするために、初速だけは記憶する必要がある　
    public:

        StraightPath(float dist, float v_s, float v_e, float v_m, float a_max) :
        Path(PositionF(0,0,0),PositionF(dist,0,0)),
        trape(dist, v_s, v_e, v_m, a_max),
        L(dist),
        v_start(v_s)
        {
            mr.V = v_s;
            mr.W = 0;
            mr.pos = PositionF(0,0,0);    
            p_type = PathType::POSITION_COMPENSATE;
        }

        void UpdateReference(){

            if (!trape.isGoal())
            {
                float v_ref = trape.calcNextReference();
                mr.pos.y += 0.5f * (v_ref + mr.V) * TRAPEZOID_STEP_TIME;
                mr.V = v_ref;
            }
            else{
                mr.pos = PositionF(L,0,0);
            }
        }

        bool isGoal(){
            return trape.isGoal();
        }

        void reset(){
            trape.reset();
            mr.V = v_start;
            mr.W = 0;
            mr.pos = PositionF(0,0,0);    
        }
};


class Swivelling : public Path{

        TrapezoidAccel<float> trape;
        float theta_ref; 
    public:

        Swivelling(float theta, float w_m, float w_dot_max) : 
        Path(PositionF(0,0,0),PositionF(0,0,theta)),
        trape(theta,0.3 ,0.3 , w_m, w_dot_max),
        theta_ref(theta)
        {
            mr.V = 0;
            mr.W = 0;
            mr.pos = PositionF(0,0,0);
            p_type = PathType::ATTITUDE_COMPENSATE;
        }

        void UpdateReference(){
            if(trape.isGoal()){
            	mr = MovementReference(0, 0, PositionF(0,0,theta_ref));
            }
            else{
                float w_ref = trape.calcNextReference();
                mr.pos.theta += w_ref * TRAPEZOID_STEP_TIME;
                mr.W = w_ref;
                mr.V = 0;
            }
        }

        bool isGoal(){
            return trape.isGoal();
        }
        void reset(){
            trape.reset();
            mr.V = 0;
            mr.W = 0;
            mr.pos = PositionF(0,0,0);    
        }

};
