---
title: Locomotion and Bipedal Walking
description: Understanding the principles of bipedal locomotion and their implementation in humanoid robots, including walking pattern generation and terrain adaptation.
sidebar_position: 7
wordCount: "1400-1700"
prerequisites: "Understanding of dynamics and control theory"
learningOutcomes:
  - "Explain the biomechanical principles of bipedal locomotion"
  - "Implement walking pattern generation algorithms"
  - "Design terrain adaptation systems for humanoid robots"
subtopics:
  - "Principles of bipedal gait"
  - "Zero moment point and stability criteria"
  - "Walking pattern generation"
  - "Terrain adaptation and obstacle navigation"
  - "Energy-efficient walking strategies"
status: draft
authors:
  - "Textbook Author"
reviewers:
  - "Domain Expert"
---

# Locomotion and Bipedal Walking

Bipedal locomotion represents one of the most challenging aspects of humanoid robotics, requiring the integration of balance control, dynamic stability, and adaptive gait generation. Unlike wheeled or tracked robots, humanoid robots must maintain balance on two legs while navigating complex terrains, making locomotion a fundamental capability that impacts all other robot functions.

The human approach to bipedal walking involves a complex interplay of passive dynamics, active control, and adaptive mechanisms that allow for efficient and stable locomotion across various terrains and conditions. Understanding these principles is essential for developing humanoid robots capable of natural, human-like walking.

## Principles of Bipedal Gait

Human bipedal gait is characterized by a distinctive pattern of alternating single and double support phases, with the center of mass following a complex trajectory that minimizes energy expenditure while maintaining stability. The gait cycle consists of two main phases: the stance phase, where one foot is in contact with the ground, and the swing phase, where the opposite foot moves forward.

The human gait pattern has evolved to be remarkably energy-efficient, with the inverted pendulum-like motion during single support and the impulsive push-off during double support contributing to energy recovery. This passive dynamic behavior reduces the active muscular effort required for walking.

During normal walking, humans spend approximately 60% of the gait cycle in the stance phase and 40% in the swing phase. The transition between phases is smooth and coordinated, with minimal impact forces at foot contact due to the compliant nature of the human body.

:::tip
The energy efficiency of human walking comes from the pendulum-like motion of the center of mass, which recovers up to 70% of the mechanical energy through the inverted pendulum mechanism.
:::

## Zero Moment Point and Stability Criteria

The Zero Moment Point (ZMP) is a critical concept in bipedal robotics, representing the point on the ground where the net moment of the ground reaction forces is zero. For stable walking, the ZMP must remain within the support polygon defined by the feet in contact with the ground.

The ZMP criterion provides a mathematically rigorous approach to ensuring dynamic stability during walking. By planning trajectories that keep the ZMP within the support region, engineers can design walking patterns that are dynamically stable even during complex maneuvers.

```python
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

class ZMPCalculator:
    """
    Calculate Zero Moment Point for bipedal stability analysis
    """
    def __init__(self, gravity=9.81):
        self.g = gravity

    def calculate_zmp(self, com_pos, com_acc, cop_z=0.0):
        """
        Calculate ZMP given center of mass position and acceleration
        com_pos: [x, y, z] position of center of mass
        com_acc: [x, y, z] acceleration of center of mass
        cop_z: z-coordinate of contact point (foot height)
        """
        zmp_x = com_pos[0] - (com_acc[0] / self.g) * (com_pos[2] - cop_z)
        zmp_y = com_pos[1] - (com_acc[1] / self.g) * (com_pos[2] - cop_z)
        return np.array([zmp_x, zmp_y])

    def is_stable(self, zmp, support_polygon):
        """
        Check if ZMP is within support polygon
        support_polygon: list of [x, y] vertices of support polygon
        """
        # Use ray casting algorithm to check if point is inside polygon
        x, y = zmp
        n = len(support_polygon)
        inside = False

        p1x, p1y = support_polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = support_polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside

class WalkingPatternGenerator:
    """
    Generate walking patterns using preview control based on ZMP
    """
    def __init__(self, step_length=0.3, step_width=0.2, step_height=0.1,
                 z_com=0.8, sampling_time=0.01):
        self.step_length = step_length
        self.step_width = step_width
        self.step_height = step_height
        self.z_com = z_com  # Height of center of mass
        self.dt = sampling_time
        self.omega = np.sqrt(self.g / z_com)

    def generate_footsteps(self, start_pos, goal_pos, step_width=0.2):
        """
        Generate sequence of footsteps from start to goal
        """
        # Calculate number of steps needed
        distance = np.linalg.norm(goal_pos - start_pos)
        n_steps = int(distance / self.step_length) + 1

        footsteps = []
        direction = (goal_pos - start_pos) / distance  # Unit direction vector

        for i in range(n_steps + 1):
            # Alternate between left and right feet
            if i % 2 == 0:  # Right foot
                foot_offset = np.array([-step_width/2, 0.0])
            else:  # Left foot
                foot_offset = np.array([step_width/2, 0.0])

            # Position along path
            pos = start_pos + i * self.step_length * direction
            foot_pos = pos + foot_offset

            footsteps.append(foot_pos)

        return np.array(footsteps)

    def generate_com_trajectory(self, footsteps, double_support_time=0.1):
        """
        Generate CoM trajectory using preview control to track desired ZMP
        """
        # Calculate total time based on footsteps
        total_time = len(footsteps) * self.step_length / 0.5  # Assuming 0.5 m/s average speed
        n_samples = int(total_time / self.dt)

        # Initialize trajectories
        com_x = np.zeros(n_samples)
        com_y = np.zeros(n_samples)
        com_z = np.full(n_samples, self.z_com)  # Assume constant height initially

        # Generate ZMP reference based on footsteps
        zmp_ref_x = np.zeros(n_samples)
        zmp_ref_y = np.zeros(n_samples)

        # Interpolate between footsteps to create ZMP reference
        for i, foot_pos in enumerate(footsteps):
            start_idx = int(i * (n_samples // len(footsteps)))
            end_idx = int(min((i + 1) * (n_samples // len(footsteps)), n_samples))

            for j in range(start_idx, end_idx):
                # Follow foot position with appropriate timing
                if i % 2 == 0:  # Right foot support
                    zmp_ref_x[j] = foot_pos[0]
                    zmp_ref_y[j] = foot_pos[1] - self.step_width/2
                else:  # Left foot support
                    zmp_ref_x[j] = foot_pos[0]
                    zmp_ref_y[j] = foot_pos[1] + self.step_width/2

        # Generate CoM trajectory using preview control
        # This is a simplified implementation - real preview control is more complex
        for i in range(1, n_samples):
            # Simple PD control on ZMP error
            zmp_current_x = com_x[i-1] - (0 / self.g) * (self.z_com)  # Simplified
            zmp_current_y = com_y[i-1] - (0 / self.g) * (self.z_com)  # Simplified

            zmp_error_x = zmp_ref_x[i] - zmp_current_x
            zmp_error_y = zmp_ref_y[i] - zmp_current_y

            # Update CoM position (simplified dynamics)
            com_x[i] = com_x[i-1] + 0.01 * zmp_error_x  # Proportional adjustment
            com_y[i] = com_y[i-1] + 0.01 * zmp_error_y  # Proportional adjustment

        return np.column_stack([com_x, com_y, com_z]), np.column_stack([zmp_ref_x, zmp_ref_y])

    def generate_swing_trajectory(self, start_foot, end_foot, height_factor=0.05):
        """
        Generate swing foot trajectory for stepping motion
        """
        # Create 5th order polynomial trajectory for smooth motion
        duration = 0.5  # Swing phase duration
        n_points = int(duration / self.dt)

        # 5th order polynomial coefficients for smooth trajectory
        # q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        t = np.linspace(0, duration, n_points)

        # Calculate coefficients for x, y, z trajectories
        # Start and end positions and velocities
        x_start, y_start = start_foot
        x_end, y_end = end_foot
        z_start = 0  # Foot on ground
        z_end = 0    # Foot on ground

        # Coefficients for 5th order polynomial (position, velocity, acceleration = 0 at start/end)
        ax = [x_start,
              0,  # Initial velocity = 0
              0,  # Initial acceleration = 0
              (20 * (x_end - x_start)) / duration**3,
              (-30 * (x_end - x_start)) / duration**4,
              (12 * (x_end - x_start)) / duration**5]

        ay = [y_start,
              0,
              0,
              (20 * (y_end - y_start)) / duration**3,
              (-30 * (y_end - y_start)) / duration**4,
              (12 * (y_end - y_start)) / duration**5]

        az = [z_start,
              0,
              0,
              (20 * height_factor) / duration**3,
              (-30 * height_factor) / duration**4,
              (12 * height_factor) / duration**5]

        # Generate trajectories
        x_traj = np.polyval(ax[::-1], t)
        y_traj = np.polyval(ay[::-1], t)

        # For z, go up and then down
        z_up = np.polyval(az[::-1], t[:n_points//2])
        z_down = np.polyval(az[::-1], t[n_points//2:])[::-1]
        z_traj = np.concatenate([z_up, z_down[:len(z_down)]])[:n_points]

        return np.column_stack([x_traj, y_traj, z_traj])

class InvertedPendulumModel:
    """
    Simple inverted pendulum model for bipedal walking
    """
    def __init__(self, com_height=0.8, gravity=9.81):
        self.h = com_height
        self.g = gravity
        self.omega = np.sqrt(gravity / com_height)

    def step_from_impulse(self, com_pos, com_vel, impulse_duration=0.1):
        """
        Calculate CoM state after applying horizontal impulse
        """
        # Simplified model of push-off impulse
        impulse_x = 0.1  # Simplified impulse
        impulse_y = 0.05

        # Update velocity due to impulse
        new_vel_x = com_vel[0] + impulse_x
        new_vel_y = com_vel[1] + impulse_y

        # Calculate CoM trajectory after impulse
        # Solution to inverted pendulum equation: x(t) = A*cosh(ωt) + B*sinh(ωt)
        A = com_pos[0]
        B = new_vel_x / self.omega

        # Calculate new position after impulse duration
        new_pos_x = A * np.cosh(self.omega * impulse_duration) + B * np.sinh(self.omega * impulse_duration)
        new_pos_y = com_pos[1] + new_vel_y * impulse_duration  # Lateral motion is simpler

        return np.array([new_pos_x, new_pos_y, self.h]), np.array([new_vel_x, new_vel_y, 0])
```

## Walking Pattern Generation

Walking pattern generation for humanoid robots typically involves creating coordinated trajectories for the center of mass, feet, and other body parts that ensure stable locomotion. Various approaches exist, from simple pre-programmed patterns to sophisticated optimization-based methods.

The Linear Inverted Pendulum Mode (LIPM) is a popular simplification that models the robot as a point mass at a constant height, allowing for analytical solutions to the balance control problem. This model enables efficient computation of walking patterns while maintaining dynamic stability.

## Terrain Adaptation and Obstacle Navigation

Terrain adaptation is crucial for humanoid robots operating in real-world environments. This involves detecting terrain properties, adjusting gait parameters, and modifying foot placement to maintain stability and efficiency.

Advanced humanoid robots use sensor feedback to detect terrain properties such as slope, compliance, and obstacles. This information is then used to adjust walking parameters in real-time, ensuring stable locomotion across varying conditions.

```cpp
#include <Eigen/Dense>
#include <vector>
#include <queue>
#include <algorithm>

class TerrainAdaptationController {
private:
    // Sensor data
    std::vector<Eigen::Vector3d> foot_pressure_sensors;
    std::vector<Eigen::Vector3d> IMU_readings;
    Eigen::Vector3d com_position;
    Eigen::Vector3d com_velocity;

    // Terrain estimation
    double estimated_slope_x;
    double estimated_slope_y;
    double terrain_height;
    bool terrain_roughness;

    // Adaptation parameters
    double step_length_base;
    double step_width_base;
    double step_height_base;
    double walking_speed_base;

public:
    TerrainAdaptationController()
        : estimated_slope_x(0), estimated_slope_y(0), terrain_height(0),
          terrain_roughness(false), step_length_base(0.3), step_width_base(0.2),
          step_height_base(0.1), walking_speed_base(0.5) {}

    void update_sensors(const std::vector<Eigen::Vector3d>& pressures,
                       const std::vector<Eigen::Vector3d>& imu,
                       const Eigen::Vector3d& com_pos,
                       const Eigen::Vector3d& com_vel) {
        foot_pressure_sensors = pressures;
        IMU_readings = imu;
        com_position = com_pos;
        com_velocity = com_vel;
    }

    void estimate_terrain() {
        // Simple terrain estimation from sensor data
        // In practice, this would involve more sophisticated algorithms

        // Estimate slope from IMU data
        if (!IMU_readings.empty()) {
            estimated_slope_x = std::atan2(IMU_readings[0][1], IMU_readings[0][2]);
            estimated_slope_y = std::atan2(-IMU_readings[0][0], IMU_readings[0][2]);
        }

        // Estimate terrain height from pressure sensors
        double avg_pressure = 0;
        for (const auto& pressure : foot_pressure_sensors) {
            avg_pressure += pressure.norm();
        }
        avg_pressure /= foot_pressure_sensors.size();

        // Simple relationship between pressure and terrain height
        terrain_height = avg_pressure * 0.001; // Simplified conversion

        // Estimate roughness from pressure variance
        double variance = 0;
        for (const auto& pressure : foot_pressure_sensors) {
            variance += std::pow(pressure.norm() - avg_pressure, 2);
        }
        variance /= foot_pressure_sensors.size();

        terrain_roughness = variance > 100.0; // Threshold for roughness
    }

    void adapt_gait_parameters() {
        // Adjust gait parameters based on terrain estimation
        double adapted_step_length = step_length_base;
        double adapted_step_width = step_width_base;
        double adapted_step_height = step_height_base;
        double adapted_speed = walking_speed_base;

        // Adjust for slope
        double slope_magnitude = std::sqrt(estimated_slope_x*estimated_slope_x +
                                         estimated_slope_y*estimated_slope_y);
        if (slope_magnitude > 0.1) { // If slope is significant
            adapted_step_length *= (1.0 - 0.3 * slope_magnitude); // Shorter steps on slopes
            adapted_step_width *= (1.0 + 0.1 * slope_magnitude);  // Wider stance
            adapted_step_height *= (1.0 + 0.2 * slope_magnitude); // Higher steps
        }

        // Adjust for rough terrain
        if (terrain_roughness) {
            adapted_step_length *= 0.8;  // Shorter steps on rough terrain
            adapted_step_width *= 1.2;   // Wider stance for stability
            adapted_step_height *= 1.5;  // Higher steps to clear obstacles
            adapted_speed *= 0.7;        // Slower speed on rough terrain
        }

        // Update controller with adapted parameters
        update_controller_parameters(adapted_step_length, adapted_step_width,
                                   adapted_step_height, adapted_speed);
    }

    void update_controller_parameters(double step_len, double step_width,
                                    double step_height, double speed) {
        // Update the walking controller with new parameters
        step_length_base = step_len;
        step_width_base = step_width;
        step_height_base = step_height;
        walking_speed_base = speed;
    }

    Eigen::Vector2d generate_foot_placement(const Eigen::Vector2d& current_com,
                                          const Eigen::Vector2d& goal_direction) {
        // Generate appropriate foot placement based on terrain and balance
        Eigen::Vector2d foot_pos = current_com + goal_direction * step_length_base;

        // Adjust for stability based on terrain
        if (terrain_roughness) {
            // Add more conservative foot placement on rough terrain
            foot_pos += Eigen::Vector2d(0.05, 0.05) * Eigen::Vector2d::Random();
        }

        return foot_pos;
    }
};

class FootstepPlanner {
private:
    std::vector<Eigen::Vector2d> planned_footsteps;
    double step_length;
    double step_width;

public:
    FootstepPlanner(double s_len = 0.3, double s_width = 0.2)
        : step_length(s_len), step_width(s_width) {}

    std::vector<Eigen::Vector2d> plan_footsteps(const Eigen::Vector2d& start_pos,
                                              const Eigen::Vector2d& goal_pos,
                                              const std::vector<Eigen::Vector2d>& obstacles) {
        std::vector<Eigen::Vector2d> footsteps;

        // Simple straight-line planning with obstacle avoidance
        Eigen::Vector2d direction = (goal_pos - start_pos).normalized();
        double distance = (goal_pos - start_pos).norm();
        int n_steps = static_cast<int>(distance / step_length);

        Eigen::Vector2d current_pos = start_pos;

        for (int i = 0; i < n_steps; ++i) {
            Eigen::Vector2d next_pos = current_pos + direction * step_length;

            // Check for obstacles and adjust path if necessary
            if (is_obstacle_near(next_pos, obstacles, 0.3)) {
                // Simple obstacle avoidance: step around obstacle
                next_pos += Eigen::Vector2d(-direction[1], direction[0]) * step_width;
            }

            // Alternate between left and right feet
            if (i % 2 == 1) { // Right foot
                next_pos += Eigen::Vector2d(-step_width/2, 0);
            } else { // Left foot
                next_pos += Eigen::Vector2d(step_width/2, 0);
            }

            footsteps.push_back(next_pos);
            current_pos = next_pos;
        }

        // Add final step to reach goal
        if ((goal_pos - current_pos).norm() > 0.1) {
            footsteps.push_back(goal_pos);
        }

        planned_footsteps = footsteps;
        return footsteps;
    }

private:
    bool is_obstacle_near(const Eigen::Vector2d& pos,
                         const std::vector<Eigen::Vector2d>& obstacles,
                         double threshold) {
        for (const auto& obstacle : obstacles) {
            if ((pos - obstacle).norm() < threshold) {
                return true;
            }
        }
        return false;
    }
};
```

## Energy-Efficient Walking Strategies

Energy efficiency in bipedal walking can be achieved through several strategies, including the exploitation of passive dynamics, optimization of gait parameters, and the use of energy recovery mechanisms. Human walking is remarkably efficient due to the pendulum-like motion of the center of mass and the storage and release of energy in tendons.

Humanoid robots can implement energy-efficient walking by mimicking these biological strategies, using compliant actuators that can store and release energy, and optimizing gait parameters such as step length and frequency for minimum energy consumption.

![Bipedal gait cycle diagram showing key phases and center of mass movement](./assets/bipedal-gait-diagram.png)

## Advanced Locomotion Techniques

Modern humanoid robots employ several advanced techniques to improve locomotion performance:

1. **Whole-Body Walking**: Coordinates upper and lower body movements for enhanced stability
2. **Push Recovery**: Automatically recovers from disturbances during walking
3. **Stair Climbing**: Adapts gait patterns for step climbing
4. **Dynamic Walking**: Allows for faster, more natural walking patterns
5. **Multi-Contact Walking**: Uses hands or other contacts for stability on challenging terrain

## Summary

Bipedal locomotion in humanoid robots requires sophisticated integration of balance control, gait generation, and terrain adaptation. The challenge lies in creating systems that can match the efficiency, stability, and adaptability of human walking while operating within the constraints of engineered components. Success in humanoid locomotion will require continued advances in control theory, sensor integration, and mechanical design that work together to create truly human-like walking capabilities.