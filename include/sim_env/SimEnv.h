//
// Created by joshua on 4/19/17.
//

#ifndef SIM_ENV_H
#define SIM_ENV_H

// STL imports
#include <shared_ptr>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace sim_env {

    enum class SimEnvEntityType {
        Object, Robot
    };

    class Object {
    public:
        /* Typedefs for shared pointers on this class */
        typedef std::shared_ptr<Object> Ptr;
        typedef std::shared_ptr<const Object> ConstPtr;

        /**
         * Returns the unique name of this object.
         * @return string representing this object's name.
         */
        virtual std::string getName() const = 0;

        /**
         * Returns the type of this object.
         * @return the type encoded as enum SimEnvEntityType
         */
        virtual SimEnvEntityType getType() const = 0;

        /**
         * Returns the transform of this object in world frame.
         * @return Eigen::Affine3d representing the pose of this object in world frame.
         */
        virtual Eigen::Affine3d getTransform() const = 0;

        /**
         * Sets the transform of this object in world frame.
         * @param tf Eigen::Affine3d representing the new pose of this object
         */
        virtual void setTransform(const Eigen::Affine3d& tf) = 0;

        /**
         * Returns the world object that this object belongs to.
         * @return the world
         */
        virtual World::Ptr getWorld() const = 0;

        /**
         * Returns the (constant) world object that this object belongs to.
         * @return the world
         */
        virtual World::ConstPtr getConstWorld() const = 0;

        // TODO other collision checks?
        /**
         * Checks whether this object collides with the given object.
         * @param other_object the object to check collision with.
         * @return True if objects are colliding, else False
         */
        virtual bool checkCollision(Object::ConstPtr other_object) const = 0;

        /**
         * Checks whether this object collides with any of the given objects.
         * @param object_list list of objects to check collisions with
         * @return True if this object collides with any of the given objects, else False
         */
        virtual bool checkCollision(const std::vector<Object::ConstPtr>& object_list) const = 0;

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
        /* Typedefs for shared pointers on this class */
        typedef std::shared_ptr<Robot> Ptr;
        typedef std::shared_ptr<const Robot> ConstPtr;

    };

    class WorldViewer {
    public:
        /* Typedefs for shared pointers on this class */
        typedef std::shared_ptr<WorldViewer> Ptr;
        typedef std::shared_ptr<const WorldViewer> ConstPtr;

        //TODO define all drawing functions here; provide support for setting colors and width
        virtual void drawFrame(const Eigen::Vector3d& transform) = 0;

    };

    class World {
    public:
        /* Typedefs for shared pointers on this class */
        typedef std::shared_ptr<World> Ptr;
        typedef std::shared_ptr<const World> ConstPtr;
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
        virtual Robot::Ptr getRobot(const std::string& name) const = 0;

        /**
         * Returns the object with given name, if available.
         * @param name The unique name of the object to return
         * @return Pointer to the object with given name or nullptr if not available
         */
        virtual Object::Ptr getObject(const std::string& name) const = 0;

        /**
         * Returns all objects stored in the world.
         * @param objects List to fill with objects. The list is not cleared.
         * @param exclude_robots Flag whether to include or exclude robots.
         */
        virtual void getObjects(std::vector<Object::Ptr>& objects, bool exclude_robots=False) const = 0;

        /**
         * Returns all robots stored in the world.
         * @param objects List to fill with objects. The list is not cleared.
         */
        virtual void getRobots(std::vector<Robot::Ptr>& robots) const = 0;

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
        virtual WorldViewer::Ptr getViewer() = 0;

    };
}

#endif //SIM_ENV_H
