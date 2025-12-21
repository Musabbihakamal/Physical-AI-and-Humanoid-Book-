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
            Start Reading
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

            <div className="row" style={{marginTop: '2rem'}}>
              <div className="col col--12">
                <h2 style={{textAlign: 'center', fontFamily: "'Montserrat', 'Helvetica', sans-serif"}}>Learning Modules</h2>
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

          </div>
        </section>
      </main>
    </Layout>
  );
}