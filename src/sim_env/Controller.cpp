//
// Created by joshua on 7/13/17.
//
#include <sim_env/Controller.h>
#include <cmath>

using namespace sim_env;

sim_env::PIDController::PIDController(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    reset();
}

sim_env::PIDController::~PIDController() {
    // nothing to do here
}

void sim_env::PIDController::setKp(float kp) {
    _kp = kp;
}

void sim_env::PIDController::setKi(float ki) {
    _ki = ki;
}

void sim_env::PIDController::setKd(float kd) {
    _kd = kd;
}

void sim_env::PIDController::setGains(float kp, float ki, float kd) {
    setKp(kp);
    setKi(ki);
    setKd(kd);
}

void sim_env::PIDController::setTarget(float target_state) {
    reset();
    _target = target_state;
}

float sim_env::PIDController::getTarget() const {
    return _target;
}

bool sim_env::PIDController::isTargetSatisfied(float current_state, float threshold) const {
    return std::fabs(current_state - _target) < threshold;
}

float sim_env::PIDController::control(float current_state) {
    float error = _target - current_state;
    float delta_error = 0.0f;
    if (not isnan(_prev_error)) {
        delta_error = error - _prev_error;
    }
    _prev_error = error;
    _integral_part += error;
    return _kp * error + _ki * _integral_part + _kd * delta_error;
}

void sim_env::PIDController::reset() {
    _integral_part = 0.0;
    _prev_error = nanf("");
}

////////////////////// IndependentMDPIDController //////////////////////////////
IndependentMDPIDController::IndependentMDPIDController(float kp, float ki, float kd) {
    setGains(kp, ki, kd);
}

IndependentMDPIDController::~IndependentMDPIDController() {
    // nothing to do here.
}

void IndependentMDPIDController::setTarget(const Eigen::VectorXf &target_state) {
    if (target_state.size() != _controllers.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::setTarget]"
                                 "Invalid input state dimension.");
    }
    for (size_t i = 0; i < target_state.size(); ++i) {
        _controllers.at(i).setTarget(target_state[i]);
    }
}

void IndependentMDPIDController::getTarget(Eigen::VectorXf& target_state) const {
    target_state.resize(_controllers.size());
    for (size_t i = 0; i < _controllers.size(); ++i) {
        target_state[i] = _controllers.at(i).getTarget();
    }
}

void IndependentMDPIDController::control(Eigen::VectorXf &output, const Eigen::VectorXf &current_state) {
    if (_controllers.size() != current_state.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::control]"
                                 "Invalid input state dimension.");
    }
    output.resize(_controllers.size());
    for (size_t i = 0; i < _controllers.size(); ++i) {
        output[i] = _controllers.at(i).control(current_state[i]);
    }
}

void IndependentMDPIDController::reset() {
    for (auto& controller : _controllers) {
        controller.reset();
    }
}

bool IndependentMDPIDController::isTargetSatisfied(Eigen::VectorXf& current_state, float threshold) const {
    bool target_satisfied = true;
    for (size_t i = 0; i < _controllers.size(); ++i) {
        target_satisfied = target_satisfied and _controllers.at(i).isTargetSatisfied(current_state[i], threshold);
    }
    return target_satisfied;
}

void IndependentMDPIDController::setGains(float kp, float ki, float kd) {
    for (auto& controller : _controllers) {
        controller.setGains(kp, ki, kd);
    }
}

void IndependentMDPIDController::setGains(const Eigen::VectorXf& kps, const Eigen::VectorXf& kis, const Eigen::VectorXf& kds) {
    if (_controllers.size() != kps.size() or _controllers.size() != kis.size() or _controllers.size() != kds.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::setGains]"
                                         "Invalid input vector dimension.");
    }
    for (size_t i = 0; i < _controllers.size(); ++i) {
        _controllers.at(i).setGains(kps[i], kis[i], kds[i]);
    }
}

void IndependentMDPIDController::setKps(const Eigen::VectorXf& kps) {
    if (_controllers.size() != kps.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::setKps]"
                                         "Invalid input vector dimension.");
    }
    for (size_t i = 0; i < _controllers.size(); ++i) {
        _controllers.at(i).setKp(kps[i]);
    }
}

void IndependentMDPIDController::setKp(float kp) {
    for (auto& controller : _controllers) {
        controller.setKp(kp);
    }
}

void IndependentMDPIDController::setKis(const Eigen::VectorXf& kis) {
    if (_controllers.size() != kis.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::setKis]"
                                         "Invalid input vector dimension.");
    }
    for (size_t i = 0; i < _controllers.size(); ++i) {
        _controllers.at(i).setKi(kis[i]);
    }
}

void IndependentMDPIDController::setKi(float ki) {
    for (auto& controller : _controllers) {
        controller.setKi(ki);
    }
}

void IndependentMDPIDController::setKds(const Eigen::VectorXf& kds) {
    if (_controllers.size() != kds.size()) {
        throw std::runtime_error("[sim_env::IndependentMDPIDController::setKds]"
                                         "Invalid input vector dimension.");
    }
    for (size_t i = 0; i < _controllers.size(); ++i) {
        _controllers.at(i).setKd(kds[i]);
    }
}

void IndependentMDPIDController::setKd(float kd) {
    for (auto& controller : _controllers) {
        controller.setKd(kd);
    }
}

unsigned int IndependentMDPIDController::getStateDimension() {
    return (unsigned int) _controllers.size();
}

void IndependentMDPIDController::setStateDimension(unsigned int dim) {
    _controllers.resize(dim);
    reset();
}

///////////////////////////// RobotPositionController ///////////////////////////////
RobotPositionController::RobotPositionController(RobotPtr robot):
        _pid_controller(0.01, 0.0001, 0.0),
        _robot(robot) {
}

RobotPositionController::~RobotPositionController() {
}

void RobotPositionController::setTargetPosition(const Eigen::VectorXf& position) {
    if (_robot.expired()) {
        LoggerPtr logger = DefaultLogger::getInstance();
        logger->logErr("Can not access underlying robot; the pointer is not valid anymore!",
                       "[sim_env::RobotPositionController::setTargetPosition]");
    }
    RobotPtr robot = _robot.lock();
    LoggerPtr logger = robot->getWorld()->getLogger();
    std::stringstream ss;
    ss << "Setting target position " << position.transpose();
    logger->logDebug(ss.str(), "[sim_env::RobotPositionController::setTargetPosition]");
    _pid_controller.setStateDimension((unsigned int) position.size());
    _pid_controller.setTarget(position);
}

bool RobotPositionController::control(const Eigen::VectorXf &positions, const Eigen::VectorXf &velocities,
                                      float timestep, RobotConstPtr robot,
                                      Eigen::VectorXf &output) {
    Eigen::VectorXf target_position;
    _pid_controller.getTarget(target_position);
    if (target_position.size() != robot->getActiveDOFs().size()) {
        LoggerPtr logger = robot->getWorld()->getLogger();
        logger->logErr("The provided target position has different dimension from the active DOFs."
                       "[sim_env::RobotPositionController::setTargetPosition]");
        std::stringstream ss;
        ss << "target size is " << target_position.size() << " active dof size is " << robot->getNumActiveDOFs();
        logger->logErr(ss.str());
        return false;
    }
    _pid_controller.control(output, positions);
    return true;
}

///////////////////////////// RobotVelocityController ///////////////////////////////
RobotVelocityController::RobotVelocityController(RobotPtr robot):
        _pid_controller(0.01, 0.0001, 0.0),
        _robot(robot) {
}

RobotVelocityController::~RobotVelocityController() {
}

void RobotVelocityController::setTargetVelocity(const Eigen::VectorXf &velocity) {
    if (_robot.expired()) {
        LoggerPtr logger = DefaultLogger::getInstance();
        logger->logErr("Can not access underlying robot; the pointer is not valid anymore!",
                       "[sim_env::RobotVelocityController::setTargetVelocity]");
    }
    _pid_controller.setStateDimension((unsigned int) velocity.size());
    _pid_controller.setTarget(velocity);
}

bool RobotVelocityController::control(const Eigen::VectorXf &positions, const Eigen::VectorXf &velocities,
                                      float timestep, RobotConstPtr robot,
                                      Eigen::VectorXf &output) {
    Eigen::VectorXf target_velocity;
    _pid_controller.getTarget(target_velocity);
    if (target_velocity.size() != robot->getActiveDOFs().size()) {
        LoggerPtr logger = robot->getWorld()->getLogger();
        logger->logErr("The provided target position has different dimension from the active DOFs."
                               "[sim_env::RobotVelocityController::setTargetVelocity]");
        return false;
    }
    _pid_controller.control(output, velocities);
    return true;
}
