#pragma once

#define TRAPEZOID_STEP_TIME 0.01f

template <typename T>
class TrapezoidAccel
{
protected:
    T path_length; //経路長
    T v_s;
    T v_e;
    T v_m;
    float t1, t2, t3;
    T a;
    T now_time;

public:
    TrapezoidAccel(T _path_length, T _v_s, T _v_e, T _v_m, T _a)
        : path_length(_path_length),
          v_s(_v_s),
          v_e(_v_e),
          v_m(_v_m),
          a(_a),
          now_time(0)
    {

        t1 = (-2 * v_s + std::sqrt(2 * v_s * v_s + 2 * v_e * v_e + 4 * a * path_length)) / (2 * a);
        t3 = (-2 * v_e + std::sqrt(2 * v_s * v_s + 2 * v_e * v_e + 4 * a * path_length)) / (2 * a);

        T v_m_ideal = v_s + a * t1;

        if (v_m_ideal < v_m)
        {
            v_m = v_m_ideal;
            t2 = 0;
        }
        else
        {
            t1 = (v_m - v_s) / a;
            t3 = (v_m - v_e) / a;
            T L2 = path_length - (0.5 * a * (t1 * t1 + t3 * t3) + v_s * t1 + v_e * t3);
            t2 = L2 / v_m;
        }
    }

    T calcNextReference()
    {

        T v_ret = v_e;
        if (now_time < t1)
        {
            v_ret = v_s + a * now_time;
        }
        else if (now_time < t1 + t2)
        {
            v_ret = v_m;
        }
        else if (now_time < t1 + t2 + t3)
        {
            v_ret = v_m - a * (now_time - (t1 + t2));
        }

        now_time += TRAPEZOID_STEP_TIME;
        return v_ret;
    }

    bool isGoal()
    {
        return now_time >= (t1 + t2 + t3) - TRAPEZOID_STEP_TIME;
    }

    void reset()
    { 
        now_time = 0;
    }
};