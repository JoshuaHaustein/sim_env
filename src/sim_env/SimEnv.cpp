//
// Created by joshua on 7/31/17.
//
#include "sim_env/SimEnv.h"

sim_env::Logger::~Logger() = default;

sim_env::Entity::~Entity() = default;

sim_env::Link::~Link() = default;

sim_env::Joint::~Joint() = default;

sim_env::Object::~Object() = default;

sim_env::Robot::~Robot() = default;

sim_env::World::~World() = default;

sim_env::WorldViewer::~WorldViewer() = default;

std::atomic_uint sim_env::WorldViewer::Handle::_global_id_counter(0);

sim_env::WorldViewer::Handle::Handle() {
    _id = _global_id_counter++;
}

sim_env::WorldViewer::Handle::~Handle() = default;

unsigned int sim_env::WorldViewer::Handle::getID() const {
    return _id;
}

void sim_env::Logger::logErr(const boost::format &bf, const std::string &prefix) const {
    logErr(boost::str(bf), prefix);
}

void sim_env::Logger::logInfo(const boost::format &bf, const std::string &prefix) const {
    logInfo(boost::str(bf), prefix);
}

void sim_env::Logger::logWarn(const boost::format &bf, const std::string &prefix) const {
    logWarn(boost::str(bf), prefix);
}

void sim_env::Logger::logDebug(const boost::format &bf, const std::string &prefix) const {
    logDebug(boost::str(bf), prefix);
}

void sim_env::Logger::log(const boost::format &bf, LogLevel level, const std::string &prefix) const {
    log(boost::str(bf), level, prefix);
}
