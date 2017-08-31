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


