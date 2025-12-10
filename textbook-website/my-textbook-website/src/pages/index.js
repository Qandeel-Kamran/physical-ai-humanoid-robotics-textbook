import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/chapter-01-introduction">
            Read the Textbook - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="A comprehensive textbook on Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4 padding-horiz--md">
                <h2>Module 1: The Robotic Nervous System (ROS 2)</h2>
                <p>Master ROS 2 architecture, communication patterns, and robot modeling. Learn to build distributed robotic systems using nodes, topics, services, and actions.</p>
                <ul>
                  <li>Explain the ROS 2 computation graph and its components</li>
                  <li>Create publishers, subscribers, and service clients using rclpy</li>
                  <li>Define robot structure using URDF and visualize in RViz2</li>
                </ul>
              </div>
              <div className="col col--4 padding-horiz--md">
                <h2>Module 2: Digital Twins - Simulation & Sensors</h2>
                <p>Build digital twins for robotic systems using Gazebo and Unity. Simulate sensors, physics, and environments for testing before deploying to physical hardware.</p>
                <ul>
                  <li>Create Gazebo simulation environments with physics and sensors</li>
                  <li>Integrate Unity for photorealistic sensor simulation</li>
                  <li>Test navigation and perception algorithms in simulation</li>
                </ul>
              </div>
              <div className="col col--4 padding-horiz--md">
                <h2>Module 3: NVIDIA Isaac - Perception & Navigation</h2>
                <p>Leverage NVIDIA Isaac Sim for GPU-accelerated robotics. Implement VSLAM, Nav2 navigation stacks, and reinforcement learning for autonomous behaviors.</p>
                <ul>
                  <li>Set up and configure NVIDIA Isaac Sim environments</li>
                  <li>Implement Visual SLAM for robot localization</li>
                  <li>Deploy Nav2 navigation stack for autonomous navigation</li>
                </ul>
              </div>
            </div>
            <div className="row padding-vert--md">
              <div className="col col--6">
                <h2>Module 4: VLA & Humanoid Robotics</h2>
                <p>Integrate Vision-Language-Action models with humanoid robots. Master humanoid kinematics, manipulation, and conversational AI for natural human-robot interaction.</p>
                <ul>
                  <li>Calculate forward and inverse kinematics for humanoid robots</li>
                  <li>Implement manipulation primitives for pick-and-place tasks</li>
                  <li>Integrate conversational AI with robot action planning</li>
                </ul>
              </div>
              <div className="col col--4 col--offset-1">
                <h2>Quick Links</h2>
                <ul>
                  <li><Link to="/docs/setup-workstation">Workstation Setup</Link></li>
                  <li><Link to="/docs/setup-edge-kit">Edge Kit Setup</Link></li>
                  <li><Link to="/docs/setup-cloud">Cloud Setup</Link></li>
                  <li><Link to="/docs/glossary">Glossary</Link></li>
                  <li><Link to="/docs/module-1-ros2">Module 1: ROS 2</Link></li>
                  <li><Link to="/docs/module-2-digital-twin">Module 2: Digital Twin</Link></li>
                  <li><Link to="/docs/module-3-isaac-sim">Module 3: Isaac Sim</Link></li>
                  <li><Link to="/docs/module-4-vla-humanoids">Module 4: VLA & Humanoids</Link></li>
                </ul>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}