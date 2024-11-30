#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <cstdint>
#include <stdint.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "arm_motor_controller/Communication/RS485.hpp"

namespace arm_motor_controller {

#define ARRAY_LEN(arr) (sizeof(arr) / sizeof(arr[0]))

// Custom command interface to trigger a homing sequence for a motor
constexpr char HW_IF_HOME[] = "home";

class Motor {
public:
    Motor(std::shared_ptr<RS485> rs485, uint8_t id, double startingPos = 0);

    void setJointLimits(double jointMin, double jointMax) {
        jointLimits[0] = jointMin;
        jointLimits[1] = jointMax;
    };

    void setMotorLimits(double motorMin, double motorMax) {
        motorLimits[0] = motorMin;
        motorLimits[1] = motorMax;
    };

    void setMotorSpeedScale(double speed) { scaledSpeed = speed; };

    virtual void setMotorHome(uint32_t speed, uint32_t config) { (void)speed, (void)config; };

    virtual int configure() { return 0; };

    virtual int enable();
    virtual int disable(bool isEmergency = false);

    virtual int exportState(std::vector<hardware_interface::StateInterface> &stateInterfaces, std::string jointName);
    virtual int exportCommand(std::vector<hardware_interface::CommandInterface> &commandInterfaces,
                              std::string jointName);

    virtual int read(double time, double period);
    virtual int write(double time, double period);

    double motorPos2Radians(double pos) {
        return (pos - motorLimits[0]) * ((jointLimits[1] - jointLimits[0]) / (motorLimits[1] - motorLimits[0])) +
               jointLimits[0];
    };
    double motorVel2Radians(double vel) { return vel / scaledSpeed; };

    double radians2MotorPos(double rad) {
        return (rad - jointLimits[0]) * ((motorLimits[1] - motorLimits[0]) / (jointLimits[1] - jointLimits[0])) +
               motorLimits[0];
    };
    double radians2MotorVel(double rad) { return rad * scaledSpeed; };

protected:
    std::shared_ptr<RS485> rs485;

    uint8_t id;

    double jointLimits[2] = {0.0, 1.0}; // ROS joint limits (min, max)
    double motorLimits[2] = {0.0, 1.0}; // Motor physical limits (min, max)
    double scaledSpeed = 1;

    double rosCurrentPos;
    double rosTargetPos;
    double rosTriggerHome;

    double rosCurrentVel;
    double rosTargetVel;
    double rosIsHomed;

    double motorPos;
    double motorVel;

    uint32_t errorCounts = 0;

private:
    double lastupdate = 0.0;
};

} // namespace arm_motor_controller

#endif // MOTOR_HPP