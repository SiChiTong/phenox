#ifndef CONTROL_H_
#define CONTROL_H_

#include <Eigen/Core>

using namespace Eigen;

class PxController {

    Vector2f v_;
    Vector2f start_point_;
    Vector2f input_;
    Vector2f drift_;

    double flight_time_;

    bool bound_locked_;
    int boost_count_;
    int dbound_lock_cnt;
    bool game_bound_locked_;

    bool game_started_;

    int bound_param_;

public:

    PxController(){
        game_started_ = false;
    }
    PxController(const PxController &o) : v_(o.v_) {
    }
    ~PxController(){}

    void init(float v_init_x, float v_init_y,
        Vector2f origin, Vector2f pos);

    bool isStarted() {
        return game_started_;
    }

    float vx() {
        return v_.x();
    }
    float vy() {
        return v_.y();
    }

    void addTime(double dt);

    void setStartPoint(Vector2f pos);

    void changeVel(double direction[2], Vector2f &pos);

    int bound(Vector2f &n);

    int bound(Vector2f &n, Vector2f &n2);

    int boundHandler(int boundary_cnt, Vector2f norm1, 
                            Vector2f norm2, Vector2f pos);

    Vector2f controlStep(Vector2f &pos, double dt);

    float input_x() {
        return input_.x();
    }
    float input_y() {
        return input_.y();
    }

    void checkBoundLock();
};

#endif
