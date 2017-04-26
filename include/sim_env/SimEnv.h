//
// Created by joshua on 4/19/17.
//

#ifndef SIM_ENV_H
#define SIM_ENV_H

// STL imports
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <mutex>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace sim_env {

    class Logger;
    typedef std::shared_ptr<Logger> LoggerPtr;
    typedef std::shared_ptr<const Logger> LoggerConstPtr;

    class Logger {
    public:
        enum class LogLevel {
            Debug=0, Info=1, Warn=2, Error=3
        };
        virtual void setLevel(LogLevel lvl) = 0;
        virtual LogLevel getLevel() = 0;
        virtual void logErr(const std::string& msg, const std::string& prefix="") = 0;
        virtual void logInfo(const std::string& msg, const std::string& prefix="") = 0;
        virtual void logWarn(const std::string& msg, const std::string& prefix="") = 0;
        virtual void logDebug(const std::string& msg, const std::string& prefix="") = 0;

    };

    class DefaultLogger : public Logger {
    public:
        DefaultLogger(const DefaultLogger& logger) = delete;
        void operator=(const DefaultLogger& logger) = delete;

        static LoggerPtr getInstance() {
            // this is thread safe in C++11
            static LoggerPtr pointer(new DefaultLogger());
            return pointer;
        }

        void setLevel(LogLevel lvl) override {
            _lvl = lvl;
        }

        LogLevel getLevel() override {
            return _lvl;
        }

        void logErr(const std::string &msg, const std::string &prefix="") override {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_lvl <= LogLevel::Error) {
                std::cout << "\033[1;31m[Error] " << prefix << " " << msg << " \033[0m " << std::endl;
            }
        }

        void logInfo(const std::string &msg, const std::string &prefix="") override {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_lvl <= LogLevel::Info) {
                std::cout << "\033[1;32m[Info] " << prefix << " " << msg << " \033[0m " << std::endl;
            }
        }

        void logWarn(const std::string &msg, const std::string &prefix="") override {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_lvl <= LogLevel::Warn) {
                std::cout << "\033[1;33m[Warning] " << prefix << " " << msg << " \033[0m " << std::endl;
            }
        }

        void logDebug(const std::string &msg, const std::string &prefix="") override {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_lvl <= LogLevel::Debug) {
                std::cout << "\033[1;35m[Debug] " << prefix << " " << msg << " \033[0m " << std::endl;
            }
        }

    private:
        LogLevel _lvl;
        DefaultLogger() : _lvl(LogLevel::Info) {}
        std::mutex _mutex;
    };

    enum class EntityType {
        Object, Robot, Joint, Link
    };

    class World;
    typedef std::shared_ptr<World> WorldPtr;
    typedef std::shared_ptr<const World> WorldConstPtr;

    class Entity;
    typedef std::shared_ptr<Entity> EntityPtr;
    typedef std::shared_ptr<const Entity> EntityConstPtr;

    class Object;
    typedef std::shared_ptr<Object> ObjectPtr;
    typedef std::shared_ptr<const Object> ObjectConstPtr;

    class Robot;
    typedef std::shared_ptr<Robot> RobotPtr;
    typedef std::shared_ptr<const Robot> RobotConstPtr;

    class Link;
    typedef std::shared_ptr<Link> LinkPtr;
    typedef std::shared_ptr<const Link> LinkConstPtr;

    class Joint;
    typedef std::shared_ptr<Joint> JointPtr;
    typedef std::shared_ptr<const Joint> JointConstPtr;

    class Collidable;
    typedef std::shared_ptr<Collidable> CollidablePtr;
    typedef std::shared_ptr<const Collidable> CollidableConstPtr;

    /**
     * Collidable is an interface for entities that support collisions checks, such as Links and Objects.
     */
    class Collidable {

        // TODO other collision checks?
        /**
         * Checks whether this Collidable collides with the given Collidable.
         * @param other the Collidable to check collision with.
         * @return True if objects are colliding, else False
         */
        virtual bool checkCollision(CollidableConstPtr other) const = 0;

        /**
         * Checks whether this Collidable collides with any of the given Collidables..
         * @param others list of Collidables to check collisions with
         * @return True if this Collidable collides with any of the given Collidables, else False
         */
        virtual bool checkCollision(const std::vector<CollidableConstPtr>& others) const = 0;
    };

    /**
     * Base class for any entity stored in a SimEnv World.
     */
    class Entity {
        /**
         * Returns the unique name of this entity.
         * @return string representing this entity's name.
         */
        virtual std::string getName() const = 0;

        /**
         * Returns the type of this entity.
         * @return the type encoded as enum SimEnvEntityType
         */
        virtual EntityType getType() const = 0;

        /**
         * Returns the transform of this object in world frame.
         * @return Eigen::Affine3d representing the pose of this object in world frame.
         */
        virtual Eigen::Affine3d getTransform() const = 0;

        /**
         * Returns the world object that this object belongs to.
         * @return the world
         */
        virtual WorldPtr getWorld() const = 0;

        /**
         * Returns the (constant) world object that this object belongs to.
         * @return the world
         */
        virtual WorldConstPtr getConstWorld() const = 0;
    };

    class Link : public Collidable, Entity  {
        // TODO what does a link provide
    };

    class Joint : public Entity {
        enum JointType {
            Revolute, Translational
        };
        virtual float getPosition() const = 0;
        virtual void setPosition(float v) = 0;
        virtual float getVelocity() const = 0;
        virtual void setVelocity(float v) = 0;
        // TODO do we need to set torques?
        virtual unsigned int getIndex() const = 0;
        virtual JointType getJointType() const = 0;
    };

    class Object : public Collidable, Entity {
    public:
        /**
         * Sets the transform of this object in world frame.
         * @param tf Eigen::Affine3d representing the new pose of this object
         */
        virtual void setTransform(const Eigen::Affine3d& tf) = 0;

        /**
         * Set the active degrees of freedom for this object.
         * @param indices list of active DOF indices
         */
        virtual void setActiveDOFs(const Eigen::VectorXi& indices) = 0;

        /**
         * Returns a list of the currently active degrees of freedom for this object.
         */
        virtual Eigen::VectorXi getActiveDOFs() = 0;

        /**
         * Returns all degree of freedom indices this object has.
         * For instance, if this object is a rigid body, this function returns [0,1,2,3,4,5],
         * where 0 - x, 1 - y, 2 - z, 3 - rx, 4 - ry, 5 - rz.
         * @return a list containing all indices for all degrees of freedom.
         */
        virtual Eigen::VectorXi getDOFIndices() = 0;

        /**
         * Get the current DoF position values of this object.
         * In case of a rigid object, this would be x,y,z,rx,ry,rz.
         * In case of a robot, this would additionally include all its joint positions.
         * @param indices a vector containing which DoFs to return. It returns all, if the vector is empty.
         * @return vector containing the requested DoF positions.
         */
        virtual Eigen::VectorXd getDOFPositions(const Eigen::VectorXi& indices=Eigen::VectorXi()) const = 0;

        /**
         * Set the current DoF position values of this object. Also see getDOFPositions.
         * @param indices a vector containing which DoFs to return. It returns all, if the vector is empty.
         */
        virtual void setDOFPositions(const Eigen::VectorXd& values, const Eigen::VectorXi& indices=Eigen::VectorXi()) = 0;

        /**
         * Get the current DoF velocity values of this object.
         * @param indices a vector containing which DoF velocities to return. It returns all, if the vector is empty.
         * @return vector containing the requested DoF velocities.
         */
        virtual Eigen::VectorXd getDOFVelocities(const Eigen::VectorXi& indices=Eigen::VectorXi()) const = 0;

        /**
         * Set the current DoF velocity values of this object. Also see getDOFPositions.
         * @param indices a vector containing which DoF velocities to return. It returns all, if the vector is empty.
         */
        virtual void setDOFVelocities(const Eigen::VectorXd& values, const Eigen::VectorXi& indices=Eigen::VectorXi()) = 0;

    };

    class Robot : public Object {
    public:

    };

    class WorldViewer {
    public:

        //TODO define all drawing functions here; provide support for setting colors and width
        virtual void drawFrame(const Eigen::Vector3d& transform) = 0;

    };

    /* Typedefs for shared pointers on this class */
    typedef std::shared_ptr<WorldViewer> WorldViewerPtr;
    typedef std::shared_ptr<const WorldViewer> WorldViewerConstPtr;

    class World {
    public:
        /**
         * Loads the world from the given file.
         * Note, that the supported file formats depend on the underlying implementation.
         * @param path A string containing a path to a world file to load.
         */
        virtual void loadWorld(const std::string& path) = 0;

        /**
         * Returns the robot with given name, if available.
         * @param name The unique name of the robot to return
         * @return Pointer to the robot with given name or nullptr if not available
         */
        virtual RobotPtr getRobot(const std::string& name) const = 0;

        /**
         * Returns the object with given name, if available.
         * @param name The unique name of the object to return
         * @return Pointer to the object with given name or nullptr if not available
         */
        virtual ObjectPtr getObject(const std::string& name) const = 0;

        /**
         * Returns all objects stored in the world.
         * @param objects List to fill with objects. The list is not cleared.
         * @param exclude_robots Flag whether to include or exclude robots.
         */
        virtual void getObjects(std::vector<ObjectPtr>& objects, bool exclude_robots=false) const = 0;

        /**
         * Returns all robots stored in the world.
         * @param objects List to fill with objects. The list is not cleared.
         */
        virtual void getRobots(std::vector<RobotPtr>& robots) const = 0;

        /**
         * If the underlying world representation supports physics simulation,
         * this function issues a physics simulation step.
         * @param steps number of physics time steps to simulate.
         */
        virtual void stepPhysics(int steps=1) const = 0;

        /**
         * Returns whether the underlying world representation supports physics simulation or not.
         * @return whether physics is supported or not.
         */
        virtual bool supportsPhysics() const = 0;

        /**
         * Sets the delta t (time) for the physics simulation.
         * @param physics_step Time in seconds to simulate per simulation step.
         */
        virtual void setPhysicsTimeStep(double physics_step) = 0;

        /**
         * Returns the currently set physics time step.
         */
        virtual void getPhysicsTimeStep() const = 0;

        /**
         * Returns an instance of graphical world viewer.
         * This function may also internally create the viewer first.
         * @return shared pointer to a world viewer showing this world.
         */
        virtual WorldViewerPtr getViewer() = 0;

        /**
         * Returns an instance of the logger used in the context of this environment.
         * This function may also internally create the logger first.
         * @return shared pointer to a logger.
         */
        virtual LoggerPtr getLogger() = 0;
    };
}

#endif //SIM_ENV_H
