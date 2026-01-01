// frontend/src/components/auth/personalization-toggle.js
import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './personalization-toggle.module.css';

const PersonalizationToggle = ({
  currentLevel,
  onLevelChange,
  showOverride = true,
  className = ''
}) => {
  const [selectedLevel, setSelectedLevel] = useState(currentLevel || 'INTERMEDIATE');
  const [isOverride, setIsOverride] = useState(false);

  const experienceLevels = [
    { key: 'BEGINNER', label: 'Beginner', description: 'Extra explanations, simpler examples' },
    { key: 'INTERMEDIATE', label: 'Intermediate', description: 'Focus on implementation' },
    { key: 'EXPERT', label: 'Expert', description: 'Advanced topics, optional challenges' }
  ];

  useEffect(() => {
    if (!isOverride) {
      setSelectedLevel(currentLevel || 'INTERMEDIATE');
    }
  }, [currentLevel, isOverride]);

  const handleLevelChange = (level) => {
    setSelectedLevel(level);
    if (showOverride) {
      setIsOverride(true);
    }
    if (onLevelChange) {
      onLevelChange(level, isOverride);
    }
  };

  return (
    <div className={clsx(styles.personalizationToggle, className)}>
      <div className={styles.header}>
        <h3 className={styles.title}>Content Personalization</h3>
        {showOverride && (
          <div className={styles.overrideToggle}>
            <label className={styles.switch}>
              <input
                type="checkbox"
                checked={isOverride}
                onChange={(e) => {
                  const newValue = e.target.checked;
                  setIsOverride(newValue);
                  if (!newValue && currentLevel) {
                    setSelectedLevel(currentLevel);
                  }
                }}
              />
              <span className={styles.slider}></span>
            </label>
            <span className={styles.overrideLabel}>
              Override profile setting
            </span>
          </div>
        )}
      </div>

      <div className={styles.levelSelector}>
        {experienceLevels.map((level) => (
          <button
            key={level.key}
            className={clsx(
              styles.levelButton,
              selectedLevel === level.key && styles.levelButtonActive
            )}
            onClick={() => handleLevelChange(level.key)}
            type="button"
          >
            <div className={styles.levelHeader}>
              <span className={styles.levelLabel}>{level.label}</span>
              {selectedLevel === level.key && (
                <span className={styles.checkmark}>✓</span>
              )}
            </div>
            <div className={styles.levelDescription}>
              {level.description}
            </div>
          </button>
        ))}
      </div>

      <div className={styles.currentSelection}>
        {isOverride ? (
          <span>Showing content for: <strong>{selectedLevel.toLowerCase()}</strong> level (overridden)</span>
        ) : (
          <span>Showing content for: <strong>{selectedLevel.toLowerCase()}</strong> level (from profile)</span>
        )}
      </div>
    </div>
  );
};

export default PersonalizationToggle;