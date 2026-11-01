#ifndef PID_H
#define PID_H

class Pid {
public:
    Pid(float kp, float ki, float kd,float out_max,float out_min);
    ~Pid() = default;

    void SetParas(float kp, float ki, float kd);
    void SetOutputLimits(float out_max, float out_min);
    void Reset();

    float Calc(float ref, float fdb);

    float kp() const { return kp_; }
    float ki() const { return ki_; }
    float kd() const { return kd_; }

private:
    float kp_;
    float ki_;
    float kd_;

    float out_max_;
    float out_min_;

    float integ_;
    float last_err_;
};
#endif // PID_H