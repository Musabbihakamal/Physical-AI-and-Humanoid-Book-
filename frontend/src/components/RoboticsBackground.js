import React from 'react';
import styles from './RoboticsBackground.module.css';

const RoboticsBackground = () => {
  return (
    <div className={styles.roboticsBackground}>
      <div className={styles.roboticsOverlay}></div>
      <div className={styles.roboticsContent}>
        <h1 className={styles.roboticsTitle}>Physical AI & Humanoid Robotics</h1>
        <p className={styles.roboticsSubtitle}>Advanced Robotics Education Platform</p>
      </div>
    </div>
  );
};

export default RoboticsBackground;