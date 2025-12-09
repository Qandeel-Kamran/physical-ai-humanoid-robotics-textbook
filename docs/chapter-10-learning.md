---
title: Learning and Adaptation in Physical Systems
description: Understanding learning algorithms and adaptation mechanisms for humanoid robots, including reinforcement learning, imitation learning, and transfer learning.
sidebar_position: 10
wordCount: "1600-1900"
prerequisites: "Machine learning and AI fundamentals"
learningOutcomes:
  - "Apply reinforcement learning to physical control tasks"
  - "Implement imitation learning from human demonstrations"
  - "Design systems that adapt to new physical environments"
subtopics:
  - "Reinforcement learning for physical tasks"
  - "Imitation learning from human demonstrations"
  - "Transfer learning between simulation and reality"
  - "Online learning and adaptation"
  - "Skill acquisition and refinement"
status: draft
authors:
  - "Textbook Author"
reviewers:
  - "Domain Expert"
---

# Learning and Adaptation in Physical Systems

Learning and adaptation are fundamental capabilities that enable humanoid robots to improve their performance over time and adapt to new situations. Unlike traditional robots that execute pre-programmed behaviors, humanoid robots must be capable of learning from experience, adapting to environmental changes, and refining their skills through practice.

The challenge in learning for physical systems lies in the need to balance exploration with safety, handle the high-dimensional state and action spaces characteristic of humanoid robots, and ensure that learning processes converge to stable and safe behaviors. Physical systems also face the reality of embodiment constraints, sensor noise, and the need for real-time performance.

## Reinforcement Learning for Physical Tasks

Reinforcement learning (RL) provides a framework for learning control policies through interaction with the environment. In the context of humanoid robots, RL can be used to learn complex behaviors such as walking, manipulation, and interaction with objects.

Deep reinforcement learning has shown particular promise for humanoid robotics, with deep neural networks serving as function approximators for complex state and action spaces. However, applying RL to physical systems requires careful consideration of safety constraints, sample efficiency, and the reality gap between simulation and the real world.

:::tip
When applying reinforcement learning to physical systems, it's crucial to implement safety constraints and exploration bounds to prevent dangerous behaviors during learning. Model-based approaches can improve sample efficiency by learning environment models.
:::

## Imitation Learning from Human Demonstrations

Imitation learning enables humanoid robots to acquire skills by observing and replicating human demonstrations. This approach can significantly reduce the amount of training time required compared to learning from scratch, as the robot can bootstrap its learning from human expertise.

Imitation learning typically involves learning a mapping from observed states to actions. This can be approached through behavioral cloning, where the robot learns to directly imitate demonstrated actions, or through inverse reinforcement learning, where the robot attempts to infer the reward function that motivated the demonstrated behavior.

```python
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque
import random

class HumanoidEnvironment:
    """
    Simplified environment for humanoid robot learning
    """
    def __init__(self):
        # State: [x_pos, y_pos, z_pos, roll, pitch, yaw, x_vel, y_vel, z_vel, roll_rate, pitch_rate, yaw_rate]
        self.state_dim = 12
        # Action: joint torques for simplified humanoid
        self.action_dim = 10
        self.state = np.zeros(self.state_dim)
        self.max_steps = 1000
        self.current_step = 0

    def reset(self):
        self.state = np.random.randn(self.state_dim) * 0.1
        self.current_step = 0
        return self.state

    def step(self, action):
        # Simplified dynamics update
        # In reality, this would involve complex physics simulation
        dt = 0.01  # Time step

        # Update state based on action (simplified)
        new_state = self.state.copy()

        # Apply action as change to certain state variables
        new_state[6:9] += action[:3] * dt  # Change in linear velocity
        new_state[9:12] += action[3:6] * dt  # Change in angular velocity
        new_state[0:3] += new_state[6:9] * dt  # Update position
        new_state[3:6] += new_state[9:12] * dt  # Update orientation (simplified)

        # Add some damping
        new_state[6:9] *= 0.99
        new_state[9:12] *= 0.99

        self.state = new_state
        self.current_step += 1

        # Calculate reward (example: staying upright and moving forward)
        reward = self.calculate_reward()

        done = self.current_step >= self.max_steps
        info = {}

        return self.state, reward, done, info

    def calculate_reward(self):
        """
        Calculate reward based on current state
        """
        # Reward for staying upright (z > 0.5)
        upright_reward = max(0, self.state[2] - 0.5) * 10

        # Penalty for falling over (roll or pitch too large)
        tilt_penalty = min(0, (0.3 - abs(self.state[3]))) * 5  # Roll
        tilt_penalty += min(0, (0.3 - abs(self.state[4]))) * 5  # Pitch

        # Reward for forward movement
        forward_reward = max(0, self.state[6]) * 5  # X velocity

        # Penalty for excessive joint velocities
        velocity_penalty = -np.sum(np.abs(self.state[6:12])) * 0.1

        total_reward = upright_reward + tilt_penalty + forward_reward + velocity_penalty
        return total_reward

class PolicyNetwork(nn.Module):
    """
    Neural network for policy in reinforcement learning
    """
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(PolicyNetwork, self).__init__()
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, hidden_dim)
        self.fc_mean = nn.Linear(hidden_dim, action_dim)
        self.fc_std = nn.Linear(hidden_dim, action_dim)

        # Initialize weights
        nn.init.xavier_uniform_(self.fc1.weight)
        nn.init.xavier_uniform_(self.fc2.weight)
        nn.init.xavier_uniform_(self.fc3.weight)
        nn.init.xavier_uniform_(self.fc_mean.weight)
        nn.init.xavier_uniform_(self.fc_std.weight)

    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        x = torch.relu(self.fc3(x))

        mean = torch.tanh(self.fc_mean(x))  # Actions between -1 and 1
        std = torch.sigmoid(self.fc_std(x))  # Standard deviation between 0 and 1

        return mean, std

class ValueNetwork(nn.Module):
    """
    Neural network for value function in reinforcement learning
    """
    def __init__(self, state_dim, hidden_dim=256):
        super(ValueNetwork, self).__init__()
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, hidden_dim)
        self.fc_out = nn.Linear(hidden_dim, 1)

    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        x = torch.relu(self.fc3(x))
        value = self.fc_out(x)
        return value

class PPOAgent:
    """
    Proximal Policy Optimization (PPO) agent for humanoid learning
    """
    def __init__(self, state_dim, action_dim, lr_actor=3e-4, lr_critic=1e-3, gamma=0.99, eps_clip=0.2):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.gamma = gamma
        self.eps_clip = eps_clip

        # Actor and critic networks
        self.actor = PolicyNetwork(state_dim, action_dim).to(self.device)
        self.critic = ValueNetwork(state_dim).to(self.device)

        # Optimizers
        self.optimizer_actor = optim.Adam(self.actor.parameters(), lr=lr_actor)
        self.optimizer_critic = optim.Adam(self.critic.parameters(), lr=lr_critic)

        # Training parameters
        self.buffer = []

    def select_action(self, state):
        """
        Select action using current policy
        """
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)

        with torch.no_grad():
            mean, std = self.actor(state_tensor)

        # Sample action from Gaussian distribution
        dist = torch.distributions.Normal(mean, std)
        action = dist.sample()
        log_prob = dist.log_prob(action)

        return action.cpu().numpy()[0], log_prob.cpu().numpy()[0]

    def evaluate(self, state, action):
        """
        Evaluate action probability and state value
        """
        mean, std = self.actor(state)
        dist = torch.distributions.Normal(mean, std)

        action_logprobs = dist.log_prob(action)
        dist_entropy = dist.entropy()

        state_values = self.critic(state)

        return action_logprobs, torch.squeeze(state_values), dist_entropy

    def update(self, states, actions, rewards, logprobs, state_values, terminals):
        """
        Update actor and critic networks
        """
        # Convert to tensors
        states = torch.FloatTensor(states).to(self.device)
        actions = torch.FloatTensor(actions).to(self.device)
        rewards = torch.FloatTensor(rewards).to(self.device)
        logprobs = torch.FloatTensor(logprobs).to(self.device)
        state_values = torch.FloatTensor(state_values).to(self.device)
        terminals = torch.BoolTensor(terminals).to(self.device)

        # Calculate discounted rewards
        discounted_rewards = []
        running_reward = 0

        for i in reversed(range(len(rewards))):
            running_reward = rewards[i] + self.gamma * running_reward * (not terminals[i])
            discounted_rewards.insert(0, running_reward)

        discounted_rewards = torch.FloatTensor(discounted_rewards).to(self.device)

        # Normalize discounted rewards
        discounted_rewards = (discounted_rewards - discounted_rewards.mean()) / (discounted_rewards.std() + 1e-7)

        # Calculate advantages
        advantages = discounted_rewards - state_values.detach()
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-7)

        # Optimize policy for K epochs
        for _ in range(3):  # K epochs
            # Evaluate actions at current policy
            logprobs_new, state_values_new, entropy = self.evaluate(states, actions)

            # Calculate ratios
            ratios = torch.exp(logprobs_new - logprobs.detach())

            # Calculate surrogates
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1 - self.eps_clip, 1 + self.eps_clip) * advantages

            # Actor loss
            actor_loss = -torch.min(surr1, surr2).mean() - 0.01 * entropy.mean()

            # Critic loss
            critic_loss = nn.MSELoss()(state_values_new, discounted_rewards)

            # Update networks
            self.optimizer_actor.zero_grad()
            actor_loss.backward()
            self.optimizer_actor.step()

            self.optimizer_critic.zero_grad()
            critic_loss.backward()
            self.optimizer_critic.step()

    def save_model(self, filepath):
        """
        Save trained model
        """
        torch.save({
            'actor_state_dict': self.actor.state_dict(),
            'critic_state_dict': self.critic.state_dict(),
            'optimizer_actor_state_dict': self.optimizer_actor.state_dict(),
            'optimizer_critic_state_dict': self.optimizer_critic.state_dict()
        }, filepath)

    def load_model(self, filepath):
        """
        Load trained model
        """
        checkpoint = torch.load(filepath, map_location=self.device)
        self.actor.load_state_dict(checkpoint['actor_state_dict'])
        self.critic.load_state_dict(checkpoint['critic_state_dict'])
        self.optimizer_actor.load_state_dict(checkpoint['optimizer_actor_state_dict'])
        self.optimizer_critic.load_state_dict(checkpoint['optimizer_critic_state_dict'])

class ImitationLearningAgent:
    """
    Behavioral cloning agent for learning from demonstrations
    """
    def __init__(self, state_dim, action_dim, learning_rate=1e-3):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Network for behavioral cloning
        self.network = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, action_dim),
            nn.Tanh()  # Actions between -1 and 1
        ).to(self.device)

        self.optimizer = optim.Adam(self.network.parameters(), lr=learning_rate)
        self.criterion = nn.MSELoss()

        # Storage for demonstration data
        self.demo_states = []
        self.demo_actions = []

    def add_demonstration(self, states, actions):
        """
        Add demonstration data
        """
        self.demo_states.extend(states)
        self.demo_actions.extend(actions)

    def train(self, epochs=100, batch_size=64):
        """
        Train the imitation learning model
        """
        if len(self.demo_states) == 0:
            print("No demonstration data available!")
            return

        states = torch.FloatTensor(self.demo_states).to(self.device)
        actions = torch.FloatTensor(self.demo_actions).to(self.device)

        dataset = torch.utils.data.TensorDataset(states, actions)
        dataloader = torch.utils.data.DataLoader(dataset, batch_size=batch_size, shuffle=True)

        for epoch in range(epochs):
            total_loss = 0
            for batch_states, batch_actions in dataloader:
                self.optimizer.zero_grad()

                predicted_actions = self.network(batch_states)
                loss = self.criterion(predicted_actions, batch_actions)

                loss.backward()
                self.optimizer.step()

                total_loss += loss.item()

            if epoch % 20 == 0:
                print(f"Epoch {epoch}, Loss: {total_loss/len(dataloader):.4f}")

    def predict_action(self, state):
        """
        Predict action for given state
        """
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)

        with torch.no_grad():
            action = self.network(state_tensor)

        return action.cpu().numpy()[0]

    def finetune_with_rl(self, env, rl_agent, steps=10000):
        """
        Fine-tune imitation learning with reinforcement learning
        """
        print("Fine-tuning with reinforcement learning...")

        state = env.reset()
        total_reward = 0

        for step in range(steps):
            # Use imitation policy to get action
            action = self.predict_action(state)

            # Add some exploration
            action += np.random.normal(0, 0.1, size=action.shape)

            next_state, reward, done, info = env.step(action)

            # Add to RL agent's buffer
            rl_agent.buffer.append((state, action, reward, done))

            state = next_state
            total_reward += reward

            if done:
                state = env.reset()

            # Update RL agent periodically
            if len(rl_agent.buffer) >= 1000:
                # This would involve updating the RL agent with collected experience
                # Implementation would depend on specific RL algorithm
                rl_agent.buffer = []  # Reset buffer after update

            if step % 1000 == 0:
                print(f"Step {step}, Average Reward: {total_reward/max(1, step):.2f}")
```

## Transfer Learning Between Simulation and Reality

The reality gap between simulation and real-world environments poses a significant challenge for humanoid robotics. While simulation provides a safe and efficient environment for learning, behaviors learned in simulation often fail to transfer to the real world due to model inaccuracies and environmental differences.

Domain randomization is a technique that aims to bridge this gap by training agents in simulations with randomized parameters, making them robust to variations between simulation and reality. This approach has shown success in enabling sim-to-real transfer for robotic manipulation and locomotion tasks.

```cpp
#include <torch/torch.h>
#include <vector>
#include <random>
#include <memory>

class DomainRandomization {
private:
    std::mt19937 rng;
    std::vector<double> mass_range;
    std::vector<double> friction_range;
    std::vector<double> actuator_noise_range;
    std::vector<double> sensor_noise_range;

public:
    DomainRandomization() : rng(std::random_device{}()) {
        // Define ranges for randomization
        mass_range = {0.8, 1.2};  // ±20% mass variation
        friction_range = {0.5, 1.5};  // Friction range
        actuator_noise_range = {0.0, 0.05};  // 5% actuator noise
        sensor_noise_range = {0.0, 0.02};    // 2% sensor noise
    }

    void randomize_environment(torch::Tensor& env_params) {
        // Randomize environment parameters
        std::uniform_real_distribution<double> mass_dist(mass_range[0], mass_range[1]);
        std::uniform_real_distribution<double> friction_dist(friction_range[0], friction_range[1]);
        std::uniform_real_distribution<double> noise_dist(actuator_noise_range[0], actuator_noise_range[1]);

        // Apply randomization to environment parameters
        auto options = torch::TensorOptions().dtype(torch::kFloat32);

        // Mass randomization
        auto mass_rand = torch::rand(env_params.size(0), options) * (mass_range[1] - mass_range[0]) + mass_range[0];
        env_params.select(1, 0) *= mass_rand;  // Assuming column 0 is mass

        // Friction randomization
        auto friction_rand = torch::rand(env_params.size(0), options) * (friction_range[1] - friction_range[0]) + friction_range[0];
        env_params.select(1, 1) *= friction_rand;  // Assuming column 1 is friction

        // Add noise to other parameters
        auto noise = torch::randn({env_params.size(0), env_params.size(1)}, options);
        noise.select(1, 2) *= (actuator_noise_range[1] - actuator_noise_range[0]);  // Actuator noise
        noise.select(1, 3) *= (sensor_noise_range[1] - sensor_noise_range[0]);      // Sensor noise
        env_params += noise * 0.1;  // Small noise addition
    }

    torch::Tensor randomize_observations(torch::Tensor& obs, double noise_level = 0.02) {
        // Add realistic noise to observations
        auto noise = torch::randn_like(obs) * noise_level;
        return obs + noise;
    }

    torch::Tensor randomize_actions(torch::Tensor& actions, double noise_level = 0.05) {
        // Add realistic noise to actions (actuator noise)
        auto noise = torch::randn_like(actions) * noise_level;
        auto noisy_actions = actions + noise;

        // Clamp actions to reasonable bounds
        return torch::clamp(noisy_actions, -1.0, 1.0);
    }
};

class SkillTransferModule {
private:
    std::shared_ptr<torch::jit::script::Module> source_policy;
    std::shared_ptr<torch::jit::script::Module> target_policy;
    std::unique_ptr<DomainRandomization> domain_randomizer;
    torch::optim::Adam optimizer;
    double adaptation_lr;

public:
    SkillTransferModule(std::shared_ptr<torch::jit::script::Module> source,
                       int state_dim, int action_dim, double lr = 1e-4)
        : source_policy(source), adaptation_lr(lr) {

        // Initialize target policy with same architecture as source
        target_policy = std::make_shared<torch::jit::script::Module>(*source);

        // Initialize domain randomizer
        domain_randomizer = std::make_unique<DomainRandomization>();

        // Setup optimizer for target policy
        std::vector<torch::Tensor> target_params;
        for (const auto& param : target_policy->parameters()) {
            target_params.push_back(param.value());
        }
        optimizer = torch::optim::Adam(target_params, lr);
    }

    void adapt_to_new_domain(const std::vector<torch::Tensor>& obs_batch,
                           const std::vector<torch::Tensor>& action_batch,
                           int adaptation_steps = 100) {
        /*
        Adapt the policy to a new domain using limited real-world data
        */
        torch::Tensor total_loss = torch::zeros({1}, torch::kFloat32);

        for (int step = 0; step < adaptation_steps; ++step) {
            optimizer.zero_grad();

            // Randomly select a batch
            int idx = std::rand() % obs_batch.size();
            auto obs = obs_batch[idx];
            auto expert_action = action_batch[idx];

            // Get action from current target policy
            std::vector<torch::jit::IValue> inputs;
            inputs.push_back(obs);
            auto policy_action_tensor = target_policy->forward(inputs).toTensor();

            // Calculate behavioral cloning loss
            auto loss = torch::mse_loss(policy_action_tensor, expert_action);

            // Backpropagate
            loss.backward();
            optimizer.step();

            total_loss += loss.item<float>();
        }

        std::cout << "Adaptation completed. Average loss: " << total_loss.item<float>() / adaptation_steps << std::endl;
    }

    torch::Tensor execute_policy(const torch::Tensor& state) {
        /*
        Execute the adapted policy
        */
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(state);

        auto result = target_policy->forward(inputs).toTensor();
        return result;
    }

    void domain_randomization_training(torch::Tensor& env_params) {
        /*
        Apply domain randomization during training
        */
        domain_randomizer->randomize_environment(env_params);
    }
};
```

## Online Learning and Adaptation

Online learning enables humanoid robots to continuously adapt their behaviors based on real-time experience. This is particularly important for humanoid robots that must operate in dynamic environments where conditions can change unexpectedly.

Online adaptation can take various forms, from simple parameter tuning to complex skill modification. The key challenge is to enable adaptation without disrupting ongoing tasks or compromising safety.

## Skill Acquisition and Refinement

Skill acquisition in humanoid robots involves learning complex motor behaviors that can be reused and combined to accomplish various tasks. These skills might include walking, grasping, reaching, or more complex behaviors like opening doors or pouring liquids.

Hierarchical skill learning structures skills at multiple levels of abstraction, allowing for efficient learning and reuse. Primitive skills can be combined to form more complex behaviors, and abstract skills can be refined into specific implementations.

![Learning architecture diagram showing different learning paradigms and their applications](./assets/learning-architecture-diagram.png)

## Advanced Learning Techniques

Modern humanoid robots employ several advanced learning techniques:

1. **Meta-Learning**: Learning to learn quickly from few examples
2. **Multi-Task Learning**: Learning multiple related tasks simultaneously
3. **Curriculum Learning**: Gradually increasing task difficulty
4. **Multi-Agent Learning**: Learning in environments with multiple agents
5. **Safe Exploration**: Exploring new behaviors while maintaining safety

## Summary

Learning and adaptation are essential capabilities that enable humanoid robots to improve their performance and adapt to new situations over time. The field continues to evolve with new algorithms and techniques that address the unique challenges of learning in physical systems, including safety, sample efficiency, and the reality gap between simulation and the real world. Success in humanoid learning will require continued advances in algorithms, computational efficiency, and safety that work together to create truly adaptive robotic systems.