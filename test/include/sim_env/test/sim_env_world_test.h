//
// This header contains generic tests for sim_env::World implementations.
// Created by joshua on 6/22/17.
//

#ifndef SIM_ENV_SIM_ENV_WORLD_TEST_H
#define SIM_ENV_SIM_ENV_WORLD_TEST_H
#include <memory>
#include "gtest/gtest.h"
#include <sim_env/SimEnv.h>

#if GTEST_HAS_PARAM_TEST

namespace sim_env {
    namespace test {
        // Struct that contains data that we need to run our world tests
        struct WorldTestData {
            sim_env::WorldPtr world; // the world to test
            std::vector<std::string> robot_names; // should contain all robot names that are expected to be in the scene.
            std::vector<std::string> object_names; // should contain all object names that are expected to be in the scene.
        };

        // Struct that contains data that we need to test the individual entity interfaces.
        // It is within the responisbility of the implementation developer to ensure that this struct is filled
        // with sensible data for the respective test cases.
        struct EntityTestData {
            sim_env::WorldPtr world; // the world the entity is in
            sim_env::EntityPtr entity; // the entity to test
            Eigen::Affine3f initial_transform; // to verify that loading works correctly
            std::string entity_name;
            sim_env::EntityType entity_type;
            // Object specific entries
            // TODO
            // Robot specific entries
            // TODO
            // Joint specific entries
            // TODO
            // Link specific entries
            // TODO
        };


        // This is a type declaration for a factory method for sim_env worlds.
        // In order to run these tests, you need to specifiy a function like this:
        typedef WorldTestData CreateWorldTestData();
        typedef EntityTestData CreateEntityTestData();

        // Test fixture that is parameterized by factory method
        class SimEnvWorldTest : public ::testing::TestWithParam<CreateWorldTestData*> {
        public:
            virtual ~SimEnvWorldTest() { }

            virtual void SetUp() {
                WorldTestData test_data = (*GetParam())();
                _world = test_data.world;
                _robot_names = test_data.robot_names;
                _object_names = test_data.object_names;
            }

            virtual void TearDown() {}
        protected:
            sim_env::WorldPtr _world;
            std::vector<std::string> _robot_names;
            std::vector<std::string> _object_names;
        };

        // Test fixture that is parameterized by factory method
        class SimEnvEntityTest : public ::testing::TestWithParam<CreateEntityTestData*> {
        public:
            virtual ~SimEnvEntityTest() { }

            virtual void SetUp() {
                EntityTestData test_data = (*GetParam())();
                _world = test_data.world;
                _entity = test_data.entity;
                _entity_type = test_data.entity_type;
                _entity_name = test_data.entity_name;
                _initial_transform = test_data.initial_transform;
            }

            virtual void TearDown() {}
        protected:
            sim_env::WorldPtr _world;
            sim_env::EntityPtr _entity;
            sim_env::EntityType _entity_type;
            std::string _entity_name;
            Eigen::Affine3f _initial_transform;
        };

        /************************************************  TESTs  ************************************************/
        //////////////////////////////////////////// SimEnvWorldTests ///////////////////////////////////////////
        // Tests that all robots exist
        TEST_P(SimEnvWorldTest, hasRobot) {
            for (auto &robot_name : _robot_names) {
                sim_env::RobotPtr robot = _world->getRobot(robot_name);
                ASSERT_NE(robot, nullptr);
                ASSERT_EQ(robot->getName(), robot_name);
            }
        }

        // Tests that all objects exist
        TEST_P(SimEnvWorldTest, hasObjects) {
            for (auto &object_name : _object_names) {
                sim_env::ObjectPtr object = _world->getObject(object_name);
                EXPECT_NE(object, nullptr);
                EXPECT_EQ(object->getName(), object_name);
            }
        }

        // Tests that all objects are retrievable and know their types.
        TEST_P(SimEnvWorldTest, getObjectsWorks) {
            std::vector<sim_env::ObjectPtr> objects;
            _world->getObjects(objects, true);
            std::vector<std::string> retrieved_names;
            for (auto &object : objects) {
                EXPECT_EQ(object->getType(), sim_env::EntityType::Object);
                retrieved_names.push_back(object->getName());
            }
            ASSERT_EQ(_object_names.size(), retrieved_names.size());
            std::sort(_object_names.begin(), _object_names.end());
            std::sort(retrieved_names.begin(), retrieved_names.end());
            for (size_t i = 0; i < _object_names.size(); ++i) {
                ASSERT_EQ(_object_names.at(i), retrieved_names.at(i));
            }
        }

        // Tests that all robots are retrievable and know their types.
        TEST_P(SimEnvWorldTest, getRobotsWorks) {
            std::vector<sim_env::RobotPtr> robots;
            _world->getRobots(robots);
            std::vector<std::string> retrieved_names;
            for (auto &robot : robots) {
                EXPECT_EQ(robot->getType(), sim_env::EntityType::Robot);
                retrieved_names.push_back(robot->getName());
            }
            ASSERT_EQ(_robot_names.size(), retrieved_names.size());
            std::sort(_robot_names.begin(), _robot_names.end());
            std::sort(retrieved_names.begin(), retrieved_names.end());
            for (size_t i = 0; i < _robot_names.size(); ++i) {
                ASSERT_EQ(_robot_names.at(i), retrieved_names.at(i));
            }
        }

        // Tests that all objects and robots can be retrieved with getObjects
        TEST_P(SimEnvWorldTest, getObjectsForAllWorks) {
            std::vector<sim_env::ObjectPtr> entities;
            _world->getObjects(entities, false);
            std::vector<std::string> retrieved_names;
            for (auto& object : entities) {
                retrieved_names.push_back(object->getName());
            }
            ASSERT_EQ(retrieved_names.size(), _object_names.size() + _robot_names.size());
            std::vector<std::string> entity_names;
            entity_names.insert(entity_names.end(), _object_names.begin(), _object_names.end());
            entity_names.insert(entity_names.end(), _robot_names.begin(), _robot_names.end());
            std::sort(entity_names.begin(), entity_names.end());
            std::sort(retrieved_names.begin(), retrieved_names.end());
            ASSERT_EQ(retrieved_names.size(), entity_names.size());
            for (size_t i = 0; i < retrieved_names.size(); ++i) {
                ASSERT_EQ(retrieved_names.at(i), entity_names.at(i));
            }
        }

        //////////////////////////////////////////// SimEnvEntityTests ///////////////////////////////////////////
        // Tests whether getName function of an entity
        TEST_P(SimEnvEntityTest, getNameWorks) {
            ASSERT_NE(_entity, nullptr);
            ASSERT_EQ(_entity->getName(), _entity_name);
        }

        TEST_P(SimEnvEntityTest, getTypeWorks) {
            ASSERT_NE(_entity, nullptr);
            ASSERT_EQ(_entity->getType(), _entity_type);
        }

        TEST_P(SimEnvEntityTest, getTransformWorks) {
            ASSERT_NE(_entity, nullptr);
//            ASSERT_EQ(_entity->getTransform(), _initial_transform);
            // TODO check transform
            ASSERT_TRUE(true);
        }

        TEST_P(SimEnvEntityTest, getWorldWorks) {
            ASSERT_NE(_entity, nullptr);
            ASSERT_EQ(_entity->getWorld(), _world);
            ASSERT_EQ(_entity->getConstWorld(), _world);
        }
        //////////////////////////////////////////// SimEnvJointTests ///////////////////////////////////////////
        // TODO
        //////////////////////////////////////////// SimEnvLinkTests ///////////////////////////////////////////
        // TODO
        //////////////////////////////////////////// SimEnvObjectTests ///////////////////////////////////////////
        // TODO
        //////////////////////////////////////////// SimEnvRobotTests ///////////////////////////////////////////
        // TODO more tests
    }
}



#endif // GTEST_HAS_PARAM_TEST
#endif //SIM_ENV_SIM_ENV_WORLD_TEST_H
