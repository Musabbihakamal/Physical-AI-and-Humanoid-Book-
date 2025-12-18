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
            to="/docs/physical-ai-book">
            Read the Physical AI & Humanoid Robotics Book 📚
          </Link>
        </div>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/docs/physical-ai/intro">
            Start Learning Now →
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
      title={`Physical AI & Humanoid Robotics Book`}
      description="Comprehensive Educational System for Physical AI & Humanoid Robotics - World-class reference for robotics + AI">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <h2>Complete Technical Book</h2>
                <p>9 comprehensive chapters covering Physical AI, ROS 2, simulation environments, and humanoid robotics following ROS 2 Rolling/Humble conventions and Python PEP8 style.</p>
              </div>
              <div className="col col--4">
                <h2>AI-Powered Learning</h2>
                <p>Glossary Maker, Code Explainer, Quiz Creator, and Chapter Generator with integrated RAG chatbot for enhanced learning. All content is technically accurate, reproducible, and safe for minors.</p>
              </div>
              <div className="col col--4">
                <h2>Real-World Applications</h2>
                <p>Implementation examples with ROS 2, NVIDIA Isaac Sim, Gazebo, Unity, Isaac ROS, and advanced robotics frameworks. All code follows platform-agnostic instructions for Linux, Jetson, and Cloud.</p>
              </div>
            </div>

            <div className="row" style={{marginTop: '2rem'}}>
              <div className="col col--12">
                <h2 style={{textAlign: 'center'}}>Learning Modules</h2>
                <div className="row" style={{gap: '1rem', justifyContent: 'center'}}>
                  <div className="col col--3" style={{margin: 0, padding: '1rem'}}>
                    <div className={`${styles.moduleCard} ${styles.moduleCard1}`}>
                      <div className={styles.moduleIcon}>
                        <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                          <path d="M12 2C13.1 2 14 2.9 14 4C14 5.1 13.1 6 12 6C10.9 6 10 5.1 10 4C10 2.9 10.9 2 12 2Z" fill="#4b86b4"/>
                          <path d="M21 9V7C21 5.9 20.1 5 19 5H16.1C15.8 4.4 15.2 4 14.5 4H9.4C8.8 4 8.2 4.4 7.9 5H5C3.9 5 3 5.9 3 7V9C1.9 9 1 9.9 1 11V15C1 16.1 1.9 17 3 17H5C5.6 19.4 7.8 21 10.5 21H13.5C16.2 21 18.4 19.4 19 17H21C22.1 17 23 16.1 23 15V11C23 9.9 22.1 9 21 9ZM19 15C19 15 18.9 15.5 18.6 15.8C18.2 16.2 17.6 16.5 17 16.5H7C6.4 16.5 5.8 16.2 5.4 15.8C5.1 15.5 5 15 5 15V11H19V15Z" fill="#2a4d69"/>
                          <path d="M8 11H6V13H8V11Z" fill="#4b86b4"/>
                          <path d="M18 11H16V13H18V11Z" fill="#4b86b4"/>
                          <path d="M15 7H9V9H15V7Z" fill="#4b86b4"/>
                        </svg>
                      </div>
                      <h3>Module 1: The Robotic Nervous System (ROS 2)</h3>
                      <p>Master ROS 2 fundamentals, architecture, and robot control systems.</p>
                      <Link className="button button--primary button--block" to="/docs/physical-ai/module-1/ch1-introduction-to-physical-ai">
                        Start Module
                      </Link>
                    </div>
                  </div>

                  <div className="col col--3" style={{margin: 0, padding: '1rem'}}>
                    <div className={`${styles.moduleCard} ${styles.moduleCard2}`}>
                      <div className={styles.moduleIcon}>
                        <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                          <path d="M12 2C13.1 2 14 2.9 14 4C14 5.1 13.1 6 12 6C10.9 6 10 5.1 10 4C10 2.9 10.9 2 12 2Z" fill="#4b86b4"/>
                          <path d="M21 9V7C21 5.9 20.1 5 19 5H16.1C15.8 4.4 15.2 4 14.5 4H9.4C8.8 4 8.2 4.4 7.9 5H5C3.9 5 3 5.9 3 7V9C1.9 9 1 9.9 1 11V15C1 16.1 1.9 17 3 17H5C5.6 19.4 7.8 21 10.5 21H13.5C16.2 21 18.4 19.4 19 17H21C22.1 17 23 16.1 23 15V11C23 9.9 22.1 9 21 9ZM19 15C19 15 18.9 15.5 18.6 15.8C18.2 16.2 17.6 16.5 17 16.5H7C6.4 16.5 5.8 16.2 5.4 15.8C5.1 15.5 5 15 5 15V11H19V15Z" fill="#2a4d69"/>
                          <path d="M8 11H6V13H8V11Z" fill="#4b86b4"/>
                          <path d="M18 11H16V13H18V11Z" fill="#4b86b4"/>
                          <path d="M15 7H9V9H15V7Z" fill="#4b86b4"/>
                        </svg>
                      </div>
                      <h3>Module 2: Digital Twin Simulation (Gazebo & Unity)</h3>
                      <p>Explore simulation environments, physics engines, and digital twin technologies.</p>
                      <Link className="button button--primary button--block" to="/docs/physical-ai/module-2/ch1-gazebo-simulation-environment-setup">
                        Start Module
                      </Link>
                    </div>
                  </div>

                  <div className="col col--3" style={{margin: 0, padding: '1rem'}}>
                    <div className={`${styles.moduleCard} ${styles.moduleCard3}`}>
                      <div className={styles.moduleIcon}>
                        <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                          <path d="M12 2C13.1 2 14 2.9 14 4C14 5.1 13.1 6 12 6C10.9 6 10 5.1 10 4C10 2.9 10.9 2 12 2Z" fill="#4b86b4"/>
                          <path d="M21 9V7C21 5.9 20.1 5 19 5H16.1C15.8 4.4 15.2 4 14.5 4H9.4C8.8 4 8.2 4.4 7.9 5H5C3.9 5 3 5.9 3 7V9C1.9 9 1 9.9 1 11V15C1 16.1 1.9 17 3 17H5C5.6 19.4 7.8 21 10.5 21H13.5C16.2 21 18.4 19.4 19 17H21C22.1 17 23 16.1 23 15V11C23 9.9 22.1 9 21 9ZM19 15C19 15 18.9 15.5 18.6 15.8C18.2 16.2 17.6 16.5 17 16.5H7C6.4 16.5 5.8 16.2 5.4 15.8C5.1 15.5 5 15 5 15V11H19V15Z" fill="#2a4d69"/>
                          <path d="M8 11H6V13H8V11Z" fill="#4b86b4"/>
                          <path d="M18 11H16V13H18V11Z" fill="#4b86b4"/>
                          <path d="M15 7H9V9H15V7Z" fill="#4b86b4"/>
                        </svg>
                      </div>
                      <h3>Module 3: The AI-Robot Brain (NVIDIA Isaac™)</h3>
                      <p>Discover perception, navigation, and AI training for robotics systems.</p>
                      <Link className="button button--primary button--block" to="/docs/physical-ai/module-3/ch1-nvidia-isaac-sim-foundations">
                        Start Module
                      </Link>
                    </div>
                  </div>

                  <div className="col col--3" style={{margin: 0, padding: '1rem'}}>
                    <div className={`${styles.moduleCard} ${styles.moduleCard4}`}>
                      <div className={styles.moduleIcon}>
                        <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                          <path d="M12 2C13.1 2 14 2.9 14 4C14 5.1 13.1 6 12 6C10.9 6 10 5.1 10 4C10 2.9 10.9 2 12 2Z" fill="#4b86b4"/>
                          <path d="M21 9V7C21 5.9 20.1 5 19 5H16.1C15.8 4.4 15.2 4 14.5 4H9.4C8.8 4 8.2 4.4 7.9 5H5C3.9 5 3 5.9 3 7V9C1.9 9 1 9.9 1 11V15C1 16.1 1.9 17 3 17H5C5.6 19.4 7.8 21 10.5 21H13.5C16.2 21 18.4 19.4 19 17H21C22.1 17 23 16.1 23 15V11C23 9.9 22.1 9 21 9ZM19 15C19 15 18.9 15.5 18.6 15.8C18.2 16.2 17.6 16.5 17 16.5H7C6.4 16.5 5.8 16.2 5.4 15.8C5.1 15.5 5 15 5 15V11H19V15Z" fill="#2a4d69"/>
                          <path d="M8 11H6V13H8V11Z" fill="#4b86b4"/>
                          <path d="M18 11H16V13H18V11Z" fill="#4b86b4"/>
                          <path d="M15 7H9V9H15V7Z" fill="#4b86b4"/>
                        </svg>
                      </div>
                      <h3>Module 4: Vision-Language-Action & Capstone</h3>
                      <p>Integrate multi-modal systems and complete your humanoid robotics project.</p>
                      <Link className="button button--primary button--block" to="/docs/physical-ai/module-4/ch1-introduction-to-vla">
                        Start Module
                      </Link>
                    </div>
                  </div>
                </div>
              </div>
            </div>

            <div className="row" style={{marginTop: '2rem', backgroundColor: '#f0f8ff', padding: '2rem', borderRadius: '8px'}}>
              <div className="col col--12">
                <h2 style={{textAlign: 'center'}}>Constitution-Governed Content</h2>
                <p style={{textAlign: 'center', fontSize: '1.1em', lineHeight: '1.6'}}>
                  All content follows our <strong>Physical AI Book Constitution</strong> ensuring technical accuracy,
                  safety for minors, reproducibility, and consistency across all chapters.
                  Code examples follow ROS 2 conventions and Python PEP8 style with platform-agnostic instructions.
                </p>
                <div style={{display: 'flex', justifyContent: 'center', gap: '1rem', marginTop: '1rem'}}>
                  <Link className="button button--primary" to="/docs/physical-ai/intro">Begin Learning</Link>
                  <Link className="button button--secondary" to="/docs/physical-ai-book">View Complete Book</Link>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}