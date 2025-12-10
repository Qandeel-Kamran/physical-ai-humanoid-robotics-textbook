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
      title={`Welcome to ${siteConfig.title}`}
      description="A comprehensive textbook on Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4 padding-horiz--md">
                <h2>Chapter 1-6: Foundations</h2>
                <p>Explore the fundamentals of Physical AI, biomechanics, sensorimotor integration, cognitive architectures, actuation systems, and control theory that form the foundation of humanoid robotics.</p>
                <ul>
                  <li>Understand the principles of Physical AI and embodiment</li>
                  <li>Learn biomechanical principles underlying human movement</li>
                  <li>Master sensorimotor integration for robotic systems</li>
                </ul>
              </div>
              <div className="col col--4 padding-horiz--md">
                <h2>Chapter 7-12: Locomotion & Control</h2>
                <p>Discover advanced locomotion techniques, manipulation strategies, perception systems, learning algorithms, human-robot interaction, and simulation methodologies.</p>
                <ul>
                  <li>Implement dynamic locomotion for humanoid robots</li>
                  <li>Design dexterous manipulation systems</li>
                  <li>Develop perception and learning capabilities</li>
                </ul>
              </div>
              <div className="col col--4 padding-horiz--md">
                <h2>Chapter 13-18: Applications & Ethics</h2>
                <p>Examine hardware design, energy efficiency, safety protocols, real-world applications, future directions, ethical considerations, and concluding thoughts on humanoid robotics.</p>
                <ul>
                  <li>Explore hardware implementations and design challenges</li>
                  <li>Understand safety and energy efficiency requirements</li>
                  <li>Consider ethical implications and future directions</li>
                </ul>
              </div>
            </div>
            <div className="row padding-vert--md">
              <div className="col col--6">
                <h2>About This Textbook</h2>
                <p>This comprehensive textbook provides a complete guide to Physical AI and Humanoid Robotics, covering theoretical foundations, practical implementations, and future directions. Each chapter builds upon previous concepts to provide a cohesive understanding of this complex and rapidly evolving field.</p>
                <ul>
                  <li>Comprehensive coverage from fundamentals to advanced topics</li>
                  <li>Practical examples and implementation strategies</li>
                  <li>Insights into current research and future directions</li>
                </ul>
              </div>
              <div className="col col--4 col--offset-1">
                <h2>Quick Links</h2>
                <ul>
                  <li><Link to="/docs/chapter-01-introduction">Introduction</Link></li>
                  <li><Link to="/docs/chapter-07-locomotion">Locomotion</Link></li>
                  <li><Link to="/docs/chapter-11-hri">Human-Robot Interaction</Link></li>
                  <li><Link to="/docs/chapter-18-ethics">Ethics</Link></li>
                  <li><Link to="/docs/textbook-summary">Summary</Link></li>
                </ul>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}