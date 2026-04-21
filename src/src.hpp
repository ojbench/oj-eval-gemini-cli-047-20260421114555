#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP
#include "math.h"
#include <vector>
#include <algorithm>

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
        my_round = -1;
        consecutive_warnings = 0;
        stuck_count = 0;
        last_pos = Vec(-1e9, -1e9);
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;
    int my_round;
    int consecutive_warnings;
    int stuck_count;
    Vec last_pos;

    bool is_safe(Vec v_next, bool token_mode) {
        int n = monitor->get_robot_number();
        for (int j = 0; j < n; ++j) {
            if (j == id) continue;
            Vec pj = monitor->get_pos_cur(j);
            Vec vj = monitor->get_v_cur(j);
            double rj = monitor->get_r(j);
            
            Vec delta_pos = pj - pos_cur; 
            Vec delta_v;
            if (token_mode) {
                delta_v = v_next - vj;
            } else {
                delta_v = v_next * 2.0 - v_cur - vj;
            }
            
            double R = r + rj + 0.1; // Larger margin
            double dist_sq = delta_pos.norm_sqr();
            
            if (dist_sq < R * R) {
                if (delta_v.dot(delta_pos) < 0) return false; 
                continue;
            }

            double dot = delta_v.dot(delta_pos);
            if (dot <= 0) continue; 
            
            double dv_sq = delta_v.norm_sqr();
            if (dv_sq < 1e-9) continue;
            
            double t = dot / dv_sq;
            double min_dist_sq;
            if (t < TIME_INTERVAL) {
                min_dist_sq = dist_sq - t * dot;
            } else {
                min_dist_sq = (delta_pos - delta_v * TIME_INTERVAL).norm_sqr();
            }
            
            if (min_dist_sq < R * R) return false;
        }
        return true;
    }

public:

    Vec get_v_next() {
        my_round++;
        if (monitor->get_warning()) {
            consecutive_warnings++;
        } else {
            consecutive_warnings = 0;
        }

        if ((pos_cur - last_pos).norm_sqr() < 1e-7) {
            stuck_count++;
        } else {
            stuck_count = 0;
        }
        last_pos = pos_cur;

        int n = monitor->get_robot_number();
        bool token_mode = (consecutive_warnings > 10 || stuck_count > 30);
        if (token_mode) {
            int active_robot = (my_round / 40) % n;
            if (id != active_robot) return Vec(0, 0);
        }

        Vec target_dir = pos_tar - pos_cur;
        double dist = target_dir.norm();
        if (dist < 1e-3) return Vec(0, 0);

        Vec v_pref = target_dir.normalize() * std::min(v_max, dist / TIME_INTERVAL);
        
        double wiggle_amp = 0.1 + 0.6 * std::min(1.0, std::max(consecutive_warnings, stuck_count) / 100.0);
        double wiggle = wiggle_amp * std::sin(my_round * 0.2 + id * 1.23);
        Vec v_steer = v_pref.rotate(wiggle + 0.2 * (id % 2 == 0 ? 1 : -1));

        if (!token_mode && consecutive_warnings < 5 && stuck_count < 5 && is_safe(v_pref, false)) return v_pref;
        
        Vec best_v = Vec(0, 0);
        double best_score = -1e18;
        if (is_safe(Vec(0, 0), token_mode)) {
            best_v = Vec(0, 0);
            best_score = -1e6; 
        }

        for (double speed_ratio : {1.0, 0.8, 0.6, 0.4, 0.2, 0.1, 0.05}) {
            double speed = v_max * speed_ratio;
            for (int angle_deg = 0; angle_deg < 360; angle_deg += 5) {
                double angle = angle_deg * PI / 180.0;
                Vec v_test(std::cos(angle) * speed, std::sin(angle) * speed);
                if (is_safe(v_test, token_mode)) {
                    double score = v_test.dot(v_steer);
                    if (score > best_score) {
                        best_score = score;
                        best_v = v_test;
                    }
                }
            }
        }
        return best_v;
    }
};


/////////////////////////////////
/// TODO: You can add any class or struct you need.
/////////////////////////////////


#endif //PPCA_SRC_HPP