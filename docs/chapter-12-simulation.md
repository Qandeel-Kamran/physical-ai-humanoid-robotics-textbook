---
title: Simulation and Modeling for Physical AI
description: Understanding physics simulation, modeling, and validation techniques for humanoid robots, including sim-to-real transfer and sensor simulation.
sidebar_position: 12
wordCount: "1400-1700"
prerequisites: "Physics simulation and modeling concepts"
learningOutcomes:
  - "Develop accurate simulation environments for humanoid robots"
  - "Implement sim-to-real transfer techniques"
  - "Validate simulation models against physical systems"
subtopics:
  - "Physics simulation environments"
  - "Real-to-sim and sim-to-real transfer"
  - "Contact modeling and friction simulation"
  - "Sensor simulation and noise modeling"
  - "Validation and verification of models"
status: draft
authors:
  - "Textbook Author"
reviewers:
  - "Domain Expert"
---

# Simulation and Modeling for Physical AI

Simulation environments play a crucial role in the development and testing of humanoid robots, providing safe and efficient platforms for algorithm development, control system testing, and behavior learning. Unlike traditional robotics applications, humanoid robots require sophisticated simulation environments that can accurately model the complex interactions between the robot, environment, and humans.

The fidelity of simulation environments directly impacts the success of sim-to-real transfer, where behaviors learned in simulation are applied to real robots. Achieving high-fidelity simulation requires careful modeling of physics, contact mechanics, sensor characteristics, and environmental factors that influence robot behavior.

## Physics Simulation Environments

Physics simulation engines form the foundation of humanoid robot simulation, providing the computational framework for modeling rigid body dynamics, contact mechanics, and environmental interactions. Popular simulation platforms for humanoid robotics include Gazebo, PyBullet, MuJoCo, Isaac Gym, and Webots, each with specific strengths for different aspects of humanoid robot development.

High-fidelity physics simulation must accurately model complex phenomena such as friction, contact dynamics, and material properties. This is particularly challenging for humanoid robots due to their complex morphology, multiple contact points, and the need to maintain balance during dynamic interactions.

Modern simulation environments often include GPU-accelerated physics computation to enable large-scale parallel simulation, which is particularly valuable for reinforcement learning applications where many simulation instances run simultaneously to accelerate learning.

:::tip
When selecting a physics simulation engine for humanoid robotics, consider the trade-off between accuracy and computational efficiency. For control system development, accuracy may be more important, while for learning applications, efficiency might be prioritized.
:::

## Real-to-Sim and Sim-to-Real Transfer

The reality gap between simulation and real-world environments remains one of the most significant challenges in humanoid robotics. Sim-to-real transfer techniques aim to bridge this gap by ensuring that behaviors learned in simulation can be successfully applied to real robots.

Domain randomization is a prominent technique that addresses the reality gap by training robots in simulations with randomized parameters, making them robust to variations between simulation and reality. This approach has proven effective for enabling sim-to-real transfer of control policies for locomotion and manipulation tasks.

```python
import numpy as np
import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

class PhysicsSimulator:
    """
    Physics simulation environment for humanoid robot
    """
    def __init__(self, use_gui=True, gravity=[0, 0, -9.81]):
        if use_gui:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(*gravity)

        # Store simulation parameters
        self.sim_params = {
            'time_step': 1/240.0,
            'num_solver_iterations': 10,
            'friction_anchor': 0.01
        }

        p.setTimeStep(self.sim_params['time_step'])
        p.setPhysicsEngineParameter(numSolverIterations=self.sim_params['num_solver_iterations'])

        # Load ground plane
        self.ground_id = p.loadURDF("plane.urdf")

        # Store robot information
        self.robot_id = None
        self.joint_info = {}
        self.link_info = {}

    def load_humanoid_robot(self, urdf_path, start_position=[0, 0, 1], start_orientation=[0, 0, 0, 1]):
        """
        Load humanoid robot URDF into simulation
        """
        self.robot_id = p.loadURDF(
            urdf_path,
            start_position,
            start_orientation,
            flags=p.URDF_USE_SELF_COLLISION
        )

        # Get joint information
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            self.joint_info[joint_info[1].decode('utf-8')] = {
                'index': i,
                'type': joint_info[2],
                'lower_limit': joint_info[8],
                'upper_limit': joint_info[9],
                'max_force': joint_info[10],
                'max_velocity': joint_info[11]
            }

        return self.robot_id

    def get_robot_state(self):
        """
        Get current state of the robot
        """
        if self.robot_id is None:
            return None

        state = {
            'joint_positions': [],
            'joint_velocities': [],
            'joint_torques': [],
            'base_position': [],
            'base_orientation': [],
            'base_linear_velocity': [],
            'base_angular_velocity': []
        }

        # Get joint states
        for joint_name, info in self.joint_info.items():
            joint_state = p.getJointState(self.robot_id, info['index'])
            state['joint_positions'].append(joint_state[0])
            state['joint_velocities'].append(joint_state[1])
            state['joint_torques'].append(joint_state[3])

        # Get base (root link) state
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        base_lin_vel, base_ang_vel = p.getBaseVelocity(self.robot_id)

        state['base_position'] = list(base_pos)
        state['base_orientation'] = list(base_orn)
        state['base_linear_velocity'] = list(base_lin_vel)
        state['base_angular_velocity'] = list(base_ang_vel)

        return state

    def apply_joint_torques(self, torques):
        """
        Apply torques to robot joints
        """
        if self.robot_id is None:
            return

        # Ensure torques list matches number of controllable joints
        torque_list = list(torques)
        num_joints = len([j for j in self.joint_info.values() if j['type'] in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]])

        if len(torque_list) != num_joints:
            raise ValueError(f"Expected {num_joints} torques, got {len(torque_list)}")

        # Apply torques
        p.setJointMotorControlArray(
            self.robot_id,
            [info['index'] for info in self.joint_info.values()],
            p.TORQUE_CONTROL,
            forces=torque_list
        )

    def step_simulation(self):
        """
        Step the simulation forward
        """
        p.stepSimulation()

    def reset_robot(self, position=[0, 0, 1], orientation=[0, 0, 0, 1]):
        """
        Reset robot to initial position
        """
        if self.robot_id is not None:
            p.resetBasePositionAndOrientation(self.robot_id, position, orientation)
            # Reset joint positions to zero
            for joint_name, info in self.joint_info.items():
                if info['type'] in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                    p.resetJointState(self.robot_id, info['index'], 0, 0)

    def disconnect(self):
        """
        Disconnect from physics engine
        """
        p.disconnect(self.physics_client)

class DomainRandomization:
    """
    Domain randomization for sim-to-real transfer
    """
    def __init__(self, randomization_ranges=None):
        if randomization_ranges is None:
            # Default randomization ranges
            self.ranges = {
                'robot_mass': [0.8, 1.2],  # ±20% mass variation
                'link_damping': [0.0, 0.1],  # Joint damping
                'friction': [0.5, 1.5],  # Friction coefficient
                'restitution': [0.0, 0.2],  # Restitution coefficient
                'gravity': [0.9, 1.1],  # Gravity scaling
                'actuator_noise': [0.0, 0.05],  # Actuator noise
                'sensor_noise': [0.0, 0.02],  # Sensor noise
                'floor_friction': [0.5, 1.5],  # Floor friction
                'object_properties': [0.8, 1.2]  # Object properties scaling
            }
        else:
            self.ranges = randomization_ranges

        self.current_params = self._sample_randomization()

    def _sample_randomization(self):
        """
        Sample randomization parameters
        """
        params = {}
        for param_name, range_vals in self.ranges.items():
            if isinstance(range_vals, (list, tuple)) and len(range_vals) == 2:
                # Sample uniformly from range
                min_val, max_val = range_vals
                params[param_name] = np.random.uniform(min_val, max_val)
            else:
                # Use the value directly if it's not a range
                params[param_name] = range_vals

        return params

    def apply_randomization(self, simulator, robot_id):
        """
        Apply domain randomization to the simulation
        """
        # Update randomization parameters
        self.current_params = self._sample_randomization()

        # Apply mass randomization
        mass_scale = self.current_params['robot_mass']
        self._apply_mass_randomization(robot_id, mass_scale)

        # Apply friction randomization
        friction = self.current_params['friction']
        self._apply_friction_randomization(simulator, friction)

        # Apply gravity randomization
        gravity_scale = self.current_params['gravity']
        p.setGravity(0, 0, -9.81 * gravity_scale)

        # Apply link damping
        damping = self.current_params['link_damping']
        self._apply_damping_randomization(robot_id, damping)

        return self.current_params

    def _apply_mass_randomization(self, robot_id, scale_factor):
        """
        Apply mass scaling to robot links
        """
        # This is a simplified implementation
        # In practice, you'd need to reload the URDF with modified masses
        # or use PyBullet's mass modification functions if available
        pass

    def _apply_friction_randomization(self, simulator, friction_coeff):
        """
        Apply friction randomization
        """
        # Modify floor friction
        p.changeDynamics(
            simulator.ground_id,
            -1,
            lateralFriction=friction_coeff,
            rollingFriction=0.005,
            spinningFriction=0.005
        )

        # Modify robot-ground friction
        for link_idx in range(p.getNumJoints(robot_id)):
            p.changeDynamics(
                robot_id,
                link_idx,
                lateralFriction=friction_coeff,
                rollingFriction=0.005,
                spinningFriction=0.005
            )

    def _apply_damping_randomization(self, robot_id, damping_coeff):
        """
        Apply joint damping randomization
        """
        for joint_name, info in simulator.joint_info.items():
            if info['type'] in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                # PyBullet doesn't directly support setting joint damping
                # This would require modifying the URDF or using a custom controller
                pass

    def add_sensor_noise(self, sensor_data):
        """
        Add noise to sensor data to simulate real sensors
        """
        noise_level = self.current_params['sensor_noise']
        noise = np.random.normal(0, noise_level, size=sensor_data.shape)
        return sensor_data + noise

    def add_actuator_noise(self, commanded_torques):
        """
        Add noise to actuator commands
        """
        noise_level = self.current_params['actuator_noise']
        noise = np.random.normal(0, noise_level, size=commanded_torques.shape)
        noisy_torques = commanded_torques + noise
        return np.clip(noisy_torques, -max_torque, max_torque)  # Clip to safe limits

class SensorSimulator:
    """
    Simulate various sensors for humanoid robots
    """
    def __init__(self, robot_id, simulator):
        self.robot_id = robot_id
        self.simulator = simulator
        self.domain_randomizer = DomainRandomization()

        # Sensor parameters
        self.camera_params = {
            'image_width': 640,
            'image_height': 480,
            'fov': 60,
            'near_plane': 0.1,
            'far_plane': 10.0
        }

    def get_joint_sensors(self):
        """
        Get joint position, velocity, and effort sensors
        """
        joint_states = p.getJointStates(
            self.robot_id,
            [info['index'] for info in self.simulator.joint_info.values()]
        )

        positions = [state[0] for state in joint_states]
        velocities = [state[1] for state in joint_states]
        efforts = [state[3] for state in joint_states]

        # Add noise to simulate real sensors
        positions = self.domain_randomizer.add_sensor_noise(np.array(positions))
        velocities = self.domain_randomizer.add_sensor_noise(np.array(velocities))
        efforts = self.domain_randomizer.add_sensor_noise(np.array(efforts))

        return {
            'positions': positions,
            'velocities': velocities,
            'efforts': efforts
        }

    def get_imu_data(self):
        """
        Simulate IMU data (orientation, angular velocity, linear acceleration)
        """
        # Get base orientation and velocity
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        lin_vel, ang_vel = p.getBaseVelocity(self.robot_id)

        # Convert to IMU-like readings
        # Orientation as quaternion
        orientation = np.array(orn)

        # Angular velocity (already in the right format)
        angular_velocity = np.array(ang_vel)

        # Linear acceleration (derivative of velocity, simplified)
        # In real implementation, this would require tracking previous velocities
        gravity = np.array([0, 0, -9.81])
        # Subtract gravity to get linear acceleration
        linear_acceleration = np.array(lin_vel) * 10  # Simplified, not physically accurate
        linear_acceleration[2] += 9.81  # Add back gravity component

        # Add sensor noise
        orientation = self.domain_randomizer.add_sensor_noise(orientation)
        angular_velocity = self.domain_randomizer.add_sensor_noise(angular_velocity)
        linear_acceleration = self.domain_randomizer.add_sensor_noise(linear_acceleration)

        return {
            'orientation': orientation,
            'angular_velocity': angular_velocity,
            'linear_acceleration': linear_acceleration
        }

    def get_force_torque_sensors(self, link_index):
        """
        Get force/torque sensor data from specific link
        """
        # Get contact information
        contact_points = p.getContactPoints(bodyA=self.robot_id, linkIndexA=link_index)

        total_normal_force = 0
        total_lateral_force = 0
        total_torque = np.zeros(3)

        for contact in contact_points:
            # Extract contact forces
            normal_force = contact[9]  # Normal force magnitude
            lateral_force1 = contact[10]  # Lateral friction force 1
            lateral_force2 = contact[11]  # Lateral friction force 2

            total_normal_force += normal_force
            total_lateral_force += np.sqrt(lateral_force1**2 + lateral_force2**2)

            # Calculate torque contribution (simplified)
            contact_pos = np.array(contact[5])  # Contact position
            link_pos, _ = p.getLinkState(self.robot_id, link_index)[:2]
            lever_arm = contact_pos - np.array(link_pos)
            torque_contribution = np.cross(lever_arm, np.array([lateral_force1, lateral_force2, normal_force]))
            total_torque += torque_contribution

        # Add sensor noise
        total_normal_force += np.random.normal(0, 0.1)
        total_lateral_force += np.random.normal(0, 0.05)
        total_torque += np.random.normal(0, 0.01, size=3)

        return {
            'normal_force': total_normal_force,
            'lateral_force': total_lateral_force,
            'torque': total_torque
        }

    def get_camera_image(self, camera_pos, target_pos):
        """
        Get RGB and depth image from camera
        """
        # Calculate view matrix
        view_matrix = p.computeViewMatrix(camera_pos, target_pos, [0, 0, 1])

        # Calculate projection matrix
        aspect_ratio = self.camera_params['image_width'] / self.camera_params['image_height']
        projection_matrix = p.computeProjectionMatrixFOV(
            self.camera_params['fov'],
            aspect_ratio,
            self.camera_params['near_plane'],
            self.camera_params['far_plane']
        )

        # Render image
        _, _, rgba, depth, seg_mask = p.getCameraImage(
            width=self.camera_params['image_width'],
            height=self.camera_params['image_height'],
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        # Extract RGB and convert depth to meters
        rgb_image = np.reshape(rgba, (self.camera_params['image_height'], self.camera_params['image_width'], 4))[:, :, :3]
        depth_image = self._convert_depth_to_meters(depth, self.camera_params['near_plane'], self.camera_params['far_plane'])

        return {
            'rgb': rgb_image,
            'depth': depth_image,
            'segmentation': seg_mask
        }

    def _convert_depth_to_meters(self, depth_buffer, near_plane, far_plane):
        """
        Convert PyBullet's depth buffer to meters
        """
        depth_image = np.array(depth_buffer)
        z_buffer = depth_image
        depth_meters = (2.0 * near_plane * far_plane) / (
            near_plane + far_plane - (z_buffer - 0.5) * 2.0 * (far_plane - near_plane)
        )
        return depth_meters

class ModelValidator:
    """
    Validate simulation models against real-world behavior
    """
    def __init__(self):
        self.validation_metrics = {
            'position_error': [],
            'velocity_error': [],
            'force_error': [],
            'timing_accuracy': []
        }

    def compare_trajectories(self, sim_trajectory, real_trajectory):
        """
        Compare simulated and real robot trajectories
        """
        if len(sim_trajectory) != len(real_trajectory):
            raise ValueError("Trajectories must have the same length")

        position_errors = []
        velocity_errors = []

        for i in range(len(sim_trajectory)):
            sim_state = sim_trajectory[i]
            real_state = real_trajectory[i]

            # Calculate position error
            pos_error = np.linalg.norm(
                np.array(sim_state['position']) - np.array(real_state['position'])
            )
            position_errors.append(pos_error)

            # Calculate velocity error
            vel_error = np.linalg.norm(
                np.array(sim_state['velocity']) - np.array(real_state['velocity'])
            )
            velocity_errors.append(vel_error)

        avg_pos_error = np.mean(position_errors)
        avg_vel_error = np.mean(velocity_errors)

        self.validation_metrics['position_error'].append(avg_pos_error)
        self.validation_metrics['velocity_error'].append(avg_vel_error)

        return {
            'avg_position_error': avg_pos_error,
            'avg_velocity_error': avg_vel_error,
            'max_position_error': np.max(position_errors),
            'max_velocity_error': np.max(velocity_errors)
        }

    def validate_contact_dynamics(self, sim_forces, real_forces):
        """
        Validate contact force predictions
        """
        force_errors = []
        for sim_f, real_f in zip(sim_forces, real_forces):
            error = np.abs(sim_f - real_f)
            force_errors.append(error)

        avg_force_error = np.mean(force_errors)
        self.validation_metrics['force_error'].append(avg_force_error)

        return {
            'avg_force_error': avg_force_error,
            'std_force_error': np.std(force_errors)
        }

    def plot_validation_results(self):
        """
        Plot validation results
        """
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))

        # Position error
        axes[0, 0].plot(self.validation_metrics['position_error'])
        axes[0, 0].set_title('Position Error Over Time')
        axes[0, 0].set_xlabel('Validation Instance')
        axes[0, 0].set_ylabel('Error (m)')

        # Velocity error
        axes[0, 1].plot(self.validation_metrics['velocity_error'])
        axes[0, 1].set_title('Velocity Error Over Time')
        axes[0, 1].set_xlabel('Validation Instance')
        axes[0, 1].set_ylabel('Error (m/s)')

        # Force error
        axes[1, 0].plot(self.validation_metrics['force_error'])
        axes[1, 0].set_title('Force Error Over Time')
        axes[1, 0].set_xlabel('Validation Instance')
        axes[1, 0].set_ylabel('Error (N)')

        # Timing accuracy
        if self.validation_metrics['timing_accuracy']:
            axes[1, 1].plot(self.validation_metrics['timing_accuracy'])
            axes[1, 1].set_title('Timing Accuracy Over Time')
            axes[1, 1].set_xlabel('Validation Instance')
            axes[1, 1].set_ylabel('Accuracy')

        plt.tight_layout()
        plt.show()
```

## Contact Modeling and Friction Simulation

Accurate contact modeling is crucial for humanoid robots, as their operation involves frequent and complex interactions with the environment through feet, hands, and other body parts. Contact models must accurately represent the physics of contact, including normal forces, friction, and impact dynamics.

The challenge in contact modeling for humanoid robots stems from the need to handle multiple simultaneous contacts, varying contact surfaces, and the transition between contact and non-contact states. Different approaches to contact modeling include penalty methods, constraint-based methods, and impulse-based methods.

Friction modeling is particularly important for humanoid robots, as it affects their ability to maintain balance, manipulate objects, and walk stably. Accurate friction models must account for static and dynamic friction, as well as the Stribeck effect in some applications.

## Sensor Simulation and Noise Modeling

Sensor simulation is essential for developing and testing perception and control systems in humanoid robots. Simulated sensors must reproduce the characteristics of real sensors, including noise, latency, and physical limitations.

Common sensors for humanoid robots include cameras for vision, IMUs for orientation and acceleration, force/torque sensors for contact detection, and joint encoders for position feedback. Each sensor type requires specific modeling approaches to accurately simulate its real-world behavior.

```cpp
#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <Eigen/Dense>
#include <random>
#include <memory>

class SensorNoiseModel {
private:
    std::random_device rd;
    std::mt19937 gen;
    std::normal_distribution<double> noise_dist;

public:
    SensorNoiseModel(double mean = 0.0, double stddev = 0.01)
        : gen(rd()), noise_dist(mean, stddev) {}

    double add_noise(double measurement) {
        return measurement + noise_dist(gen);
    }

    Eigen::VectorXd add_noise_vec(const Eigen::VectorXd& measurement) {
        Eigen::VectorXd noisy = measurement;
        for (int i = 0; i < noisy.size(); ++i) {
            noisy[i] += noise_dist(gen);
        }
        return noisy;
    }

    void set_noise_parameters(double mean, double stddev) {
        noise_dist = std::normal_distribution<double>(mean, stddev);
    }
};

class SimulatedIMU {
private:
    Eigen::Vector3d true_angular_velocity;
    Eigen::Vector3d true_linear_acceleration;
    Eigen::Quaterniond true_orientation;

    SensorNoiseModel gyro_noise;
    SensorNoiseModel accel_noise;
    SensorNoiseModel mag_noise;

    double bias_drift_gyro;
    double bias_drift_accel;

public:
    SimulatedIMU() : gyro_noise(0.0, 0.01), accel_noise(0.0, 0.02), mag_noise(0.0, 0.005),
                     bias_drift_gyro(0.0), bias_drift_accel(0.0) {}

    struct IMUReading {
        Eigen::Vector3d angular_velocity;
        Eigen::Vector3d linear_acceleration;
        Eigen::Quaterniond orientation;
        double temperature;
    };

    IMUReading get_reading(const dart::dynamics::SkeletonPtr& robot, size_t link_idx) {
        // Get the link's transform and velocity
        auto link = robot->getBodyNode(link_idx);
        auto transform = link->getTransform();
        auto spatial_velocity = link->getSpatialVelocity();

        // Extract linear and angular components
        Eigen::Vector3d linear_velocity = spatial_velocity.linear();
        Eigen::Vector3d angular_velocity = spatial_velocity.angular();

        // Calculate linear acceleration (simplified - would need proper derivative in practice)
        Eigen::Vector3d gravity(0, 0, -9.81);
        Eigen::Vector3d linear_acceleration = gravity;  // Just gravity in base frame initially

        // Add sensor noise
        IMUReading reading;
        reading.angular_velocity = gyro_noise.add_noise_vec(angular_velocity);
        reading.linear_acceleration = accel_noise.add_noise_vec(linear_acceleration);
        reading.orientation = Eigen::Quaterniond(transform.rotation());  // True orientation
        reading.temperature = 25.0 + 0.1 * noise_dist(gen);  // Temperature with small variation

        return reading;
    }

    void update_bias_drift() {
        // Simulate slow bias drift over time
        bias_drift_gyro += 0.001 * (noise_dist(gen) - 0.5);  // Slow drift
        bias_drift_accel += 0.0005 * (noise_dist(gen) - 0.5);
    }
};

class ContactSensor {
private:
    std::vector<dart::collision::Contact> last_contacts;
    SensorNoiseModel force_noise;

public:
    ContactSensor() : force_noise(0.0, 0.1) {}  // 0.1N noise in force measurements

    struct ContactInfo {
        Eigen::Vector3d contact_point;
        Eigen::Vector3d normal_force;
        Eigen::Vector3d tangential_force;
        double pressure;
        bool contact_present;
    };

    std::vector<ContactInfo> get_contact_information(const dart::dynamics::SkeletonPtr& robot) {
        std::vector<ContactInfo> contacts;

        // Access the collision detector to get contact information
        auto collision_detector = robot->getCollisionDetector();
        auto collision_group = collision_detector->createCollisionGroupShared();

        // This is a simplified approach - in practice, you'd use DART's collision detection
        // For now, we'll simulate contact detection based on proximity

        for (size_t i = 0; i < robot->getNumBodyNodes(); ++i) {
            auto body_node = robot->getBodyNode(i);

            // Check if this body node is close to the ground or other objects
            Eigen::Vector3d pos = body_node->getTransform().translation();

            // Simplified ground contact detection
            if (pos[2] < 0.01) {  // Very close to ground
                ContactInfo ci;
                ci.contact_point = pos;
                ci.normal_force = Eigen::Vector3d(0, 0, 100);  // Simplified normal force
                ci.tangential_force = Eigen::Vector3d(10, 5, 0);  // Simplified tangential force
                ci.pressure = 100000;  // Simplified pressure (Pa)
                ci.contact_present = true;

                // Add noise to force measurements
                ci.normal_force = force_noise.add_noise_vec(ci.normal_force);
                ci.tangential_force = force_noise.add_noise_vec(ci.tangential_force);

                contacts.push_back(ci);
            }
        }

        return contacts;
    }
};

class SimulationManager {
private:
    dart::simulation::WorldPtr world;
    std::shared_ptr<SimulatedIMU> imu_sim;
    std::shared_ptr<ContactSensor> contact_sim;
    SensorNoiseModel process_noise;

public:
    SimulationManager() : process_noise(0.0, 0.001) {
        world = dart::simulation::World::create();
        imu_sim = std::make_shared<SimulatedIMU>();
        contact_sim = std::make_shared<ContactSensor>();

        // Set gravity
        world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
    }

    void setup_humanoid_robot(const std::string& urdf_path) {
        // Load robot from URDF (simplified - would need proper URDF parser)
        // In practice, you'd use DART's URDF loader
        auto robot = dart::utils::DartLoader().parseSkeleton(urdf_path, world.get());

        if (robot) {
            world->addSkeleton(robot);
            std::cout << "Loaded robot with " << robot->getNumDofs() << " degrees of freedom" << std::endl;
        } else {
            std::cerr << "Failed to load robot from URDF" << std::endl;
        }
    }

    void simulate_step(double dt) {
        // Set time step
        world->setTimeStep(dt);

        // Integrate the world forward by one step
        world->step();

        // Update sensor simulations
        imu_sim->update_bias_drift();
    }

    void apply_control_inputs(const std::vector<double>& torques) {
        auto robot = world->getSkeleton(0);  // Assuming first skeleton is the robot
        if (robot && torques.size() == robot->getNumDofs()) {
            robot->setForces(Eigen::VectorXd::Map(torques.data(), torques.size()));
        }
    }

    std::vector<double> get_sensor_readings(size_t robot_index = 0) {
        auto robot = world->getSkeleton(robot_index);
        if (!robot) return {};

        std::vector<double> sensor_data;

        // Get joint positions
        for (size_t i = 0; i < robot->getNumDofs(); ++i) {
            double pos = robot->getPosition(i);
            pos += process_noise.add_noise(0.0);  // Add small noise
            sensor_data.push_back(pos);
        }

        // Get joint velocities
        for (size_t i = 0; i < robot->getNumDofs(); ++i) {
            double vel = robot->getVelocity(i);
            vel += process_noise.add_noise(0.0);
            sensor_data.push_back(vel);
        }

        return sensor_data;
    }

    double get_simulation_time() const {
        return world->getTime();
    }

    dart::simulation::WorldPtr get_world() {
        return world;
    }
};
```

## Validation and Verification of Models

Model validation is critical for ensuring that simulation results accurately reflect real-world behavior. This involves comparing simulation outputs with real-world measurements and adjusting models to minimize discrepancies.

Validation approaches include system identification techniques to determine model parameters, comparison of dynamic responses, and statistical validation of stochastic behaviors. The goal is to ensure that the simulation model captures the essential dynamics and behaviors relevant to the intended application.

![Simulation environment diagram showing physics engine, sensors, and control interfaces](./assets/simulation-environment-diagram.png)

## Advanced Simulation Techniques

Modern humanoid robotics simulation employs several advanced techniques:

1. **GPU Acceleration**: Leveraging graphics processors for parallel physics simulation
2. **Reduced-Order Modeling**: Simplifying complex models while preserving essential dynamics
3. **Adaptive Simulation**: Adjusting simulation parameters based on system behavior
4. **Multi-Fidelity Simulation**: Combining different levels of model fidelity
5. **Digital Twins**: Creating real-time synchronized simulation models

## Summary

Simulation and modeling are fundamental to humanoid robot development, enabling safe and efficient algorithm development, testing, and validation. The challenge lies in creating simulation environments that are both accurate enough to yield meaningful results and efficient enough to support the computational demands of modern robot learning and control approaches. Success in simulation-to-real transfer requires careful attention to the reality gap and the implementation of techniques like domain randomization to ensure robust performance across simulation and real-world environments.