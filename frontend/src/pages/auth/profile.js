// frontend/src/pages/auth/profile.js
import React, { useState, useEffect } from 'react';
import { useHistory } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import styles from './auth.module.css';
import { AuthProvider, useAuth } from '../../contexts/AuthContext';
import { API_BASE_URL } from '../../constants/apiConfig';

// Wrapper component to provide Auth context
function ProfileWithAuth() {
  const { siteConfig } = useDocusaurusContext();
  const { user, logout, updateProfile } = useAuth();
  const history = useHistory();
  const [formData, setFormData] = useState({
    full_name: '',
    experience_level: 'BEGINNER',
    technical_background: '',
    preferred_difficulty: 'MEDIUM',
    learning_goals: [],
    hardware_access: [],
    language_preference: 'en'
  });
  const [newGoal, setNewGoal] = useState('');
  const [newHardware, setNewHardware] = useState('');
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  // Load user profile data when component mounts
  useEffect(() => {
    if (user) {
      setFormData({
        full_name: user.full_name || user.name || '',
        experience_level: user.profile?.experience_level || 'BEGINNER',
        technical_background: user.profile?.technical_background || '',
        preferred_difficulty: user.profile?.preferred_difficulty || 'MEDIUM',
        learning_goals: user.profile?.learning_goals || [],
        hardware_access: user.profile?.hardware_access || [],
        language_preference: user.profile?.language_preference || 'en'
      });
    }
  }, [user]);

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
    // Clear error when user starts typing
    if (error) setError('');
    if (success) setSuccess('');
  };

  const handleAddGoal = () => {
    if (newGoal.trim() && !formData.learning_goals.includes(newGoal.trim())) {
      setFormData(prev => ({
        ...prev,
        learning_goals: [...prev.learning_goals, newGoal.trim()]
      }));
      setNewGoal('');
    }
  };

  const handleRemoveGoal = (goalToRemove) => {
    setFormData(prev => ({
      ...prev,
      learning_goals: prev.learning_goals.filter(goal => goal !== goalToRemove)
    }));
  };

  const handleAddHardware = () => {
    if (newHardware.trim() && !formData.hardware_access.includes(newHardware.trim())) {
      setFormData(prev => ({
        ...prev,
        hardware_access: [...prev.hardware_access, newHardware.trim()]
      }));
      setNewHardware('');
    }
  };

  const handleRemoveHardware = (hardwareToRemove) => {
    setFormData(prev => ({
      ...prev,
      hardware_access: prev.hardware_access.filter(hw => hw !== hardwareToRemove)
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    setIsLoading(true);
    setError('');
    setSuccess('');

    try {
      // Check if backend is accessible
      try {
        const response = await fetch(`${API_BASE_URL}/health`);
        if (!response.ok) {
          throw new Error('Backend server is not accessible. Please ensure the backend server is running.');
        }
      } catch (connectionErr) {
        console.error('Backend connection check failed:', connectionErr);
        throw new Error('Cannot connect to backend server. Please ensure the backend server is running on port 8000.');
      }

      // Update user profile using auth context
      const result = await updateProfile({
        full_name: formData.full_name,
        experience_level: formData.experience_level,
        technical_background: formData.technical_background,
        preferred_difficulty: formData.preferred_difficulty,
        learning_goals: formData.learning_goals,
        hardware_access: formData.hardware_access,
        language_preference: formData.language_preference
      });

      if (result.success) {
        setSuccess('Profile updated successfully!');

        // Update the local user state with the new data
        // The auth context should have updated the user state automatically
      } else {
        throw new Error(result.error || 'Failed to update profile');
      }
    } catch (err) {
      setError(err.message || 'Profile update failed. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  const handleLogout = () => {
    logout();
    history.push('/auth/signin');
  };

  if (!user) {
    return (
      <Layout title={`Profile - ${siteConfig.title}`} description="Please sign in to view your profile">
        <main className={styles.authContainer}>
          <div className={styles.authCard}>
            <div className={styles.formSection}>
              <h1>Profile Access</h1>
              <p>Please sign in to access your profile.</p>
              <button
                className={styles.primaryButton}
                onClick={() => history.push('/auth/signin')}
              >
                Sign In
              </button>
            </div>
          </div>
        </main>
      </Layout>
    );
  }

  return (
    <Layout
      title={`Profile - ${siteConfig.title}`}
      description="Manage your profile and preferences">
      <main className={styles.authContainer}>
        <div className={styles.authCard}>
          {/* Left side with user info */}
          <div className={styles.imageSection}>
            <div className={styles.imageSectionContent}>
              <h2>Your Profile</h2>
              <p>Manage your information and preferences</p>
              <div className={styles.userInfo}>
                <p><strong>Email:</strong> {user.email}</p>
                <p><strong>Member since:</strong> {user.created_at ? new Date(user.created_at).toLocaleDateString() : 'Unknown'}</p>
              </div>
            </div>
          </div>

          {/* Right side with form */}
          <div className={styles.formSection}>
            <h1>Update Your Profile</h1>
            <p className={styles.subtitle}>Customize your experience and learning preferences</p>

            {error && (
              <div className={clsx(styles.alert, styles['alert--danger'])}>
                {error}
              </div>
            )}

            {success && (
              <div className={clsx(styles.alert, styles['alert--success'])}>
                {success}
              </div>
            )}

            <form onSubmit={handleSubmit} className={styles.form}>
              <div className={styles.formGroup}>
                <label htmlFor="full_name" className={styles.formLabel}>Full Name</label>
                <div className={styles.inputWithIcon}>
                  <svg className={styles.inputIcon} width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
                    <path fillRule="evenodd" d="M10 9a3 3 0 100-6 3 3 0 000 6zm-7 9a7 7 0 1114 0H3z" clipRule="evenodd" />
                  </svg>
                  <input
                    type="text"
                    id="full_name"
                    name="full_name"
                    className={styles.formControl}
                    value={formData.full_name}
                    onChange={handleChange}
                    placeholder="Enter your full name"
                    autoComplete="name"
                  />
                </div>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="experience_level" className={styles.formLabel}>Experience Level</label>
                <select
                  id="experience_level"
                  name="experience_level"
                  className={styles.formControl}
                  value={formData.experience_level}
                  onChange={handleChange}
                >
                  <option value="BEGINNER">Beginner - Just starting out</option>
                  <option value="INTERMEDIATE">Intermediate - Some experience</option>
                  <option value="EXPERT">Expert - Advanced knowledge</option>
                </select>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="technical_background" className={styles.formLabel}>Technical Background</label>
                <textarea
                  id="technical_background"
                  name="technical_background"
                  className={styles.formControl}
                  value={formData.technical_background}
                  onChange={handleChange}
                  placeholder="Describe your technical background, programming experience, etc."
                  rows="3"
                />
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="preferred_difficulty" className={styles.formLabel}>Preferred Difficulty</label>
                <select
                  id="preferred_difficulty"
                  name="preferred_difficulty"
                  className={styles.formControl}
                  value={formData.preferred_difficulty}
                  onChange={handleChange}
                >
                  <option value="EASY">Easy</option>
                  <option value="MEDIUM">Medium</option>
                  <option value="HARD">Hard</option>
                </select>
              </div>

              <div className={styles.formGroup}>
                <label className={styles.formLabel}>Learning Goals</label>
                <div className={styles.inputWithIcon}>
                  <input
                    type="text"
                    value={newGoal}
                    onChange={(e) => setNewGoal(e.target.value)}
                    placeholder="Add a learning goal (e.g., Learn AI concepts)"
                    className={styles.formControl}
                    onKeyPress={(e) => {
                      if (e.key === 'Enter') {
                        e.preventDefault();
                        handleAddGoal();
                      }
                    }}
                  />
                  <button
                    type="button"
                    className={styles.addButton}
                    onClick={handleAddGoal}
                  >
                    Add
                  </button>
                </div>
                <div className={styles.tagList}>
                  {formData.learning_goals.map((goal, index) => (
                    <span key={index} className={styles.tag}>
                      {goal}
                      <button
                        type="button"
                        className={styles.tagRemove}
                        onClick={() => handleRemoveGoal(goal)}
                        aria-label="Remove goal"
                      >
                        ×
                      </button>
                    </span>
                  ))}
                </div>
              </div>

              <div className={styles.formGroup}>
                <label className={styles.formLabel}>Hardware Access</label>
                <div className={styles.inputWithIcon}>
                  <input
                    type="text"
                    value={newHardware}
                    onChange={(e) => setNewHardware(e.target.value)}
                    placeholder="Add hardware you have access to (e.g., Raspberry Pi)"
                    className={styles.formControl}
                    onKeyPress={(e) => {
                      if (e.key === 'Enter') {
                        e.preventDefault();
                        handleAddHardware();
                      }
                    }}
                  />
                  <button
                    type="button"
                    className={styles.addButton}
                    onClick={handleAddHardware}
                  >
                    Add
                  </button>
                </div>
                <div className={styles.tagList}>
                  {formData.hardware_access.map((hw, index) => (
                    <span key={index} className={styles.tag}>
                      {hw}
                      <button
                        type="button"
                        className={styles.tagRemove}
                        onClick={() => handleRemoveHardware(hw)}
                        aria-label="Remove hardware"
                      >
                        ×
                      </button>
                    </span>
                  ))}
                </div>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="language_preference" className={styles.formLabel}>Language Preference</label>
                <select
                  id="language_preference"
                  name="language_preference"
                  className={styles.formControl}
                  value={formData.language_preference}
                  onChange={handleChange}
                >
                  <option value="en">English</option>
                  <option value="es">Spanish</option>
                  <option value="fr">French</option>
                  <option value="de">German</option>
                </select>
              </div>

              <button
                type="submit"
                className={styles.primaryButton}
                disabled={isLoading}
              >
                {isLoading ? (
                  <div className={styles.loadingSpinner}>
                    <svg className={styles.spinner} width="20" height="20" viewBox="0 0 20 20" fill="none">
                      <circle cx="10" cy="10" r="8" stroke="currentColor" strokeWidth="2" fill="none" opacity="0.3" />
                      <path d="M10 2a8 8 0 018 8" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
                    </svg>
                    Updating profile...
                  </div>
                ) : (
                  'Update Profile'
                )}
              </button>
            </form>

            <div className={styles.authLinks}>
              <div className="margin-bottom--lg">
                <button
                  className={styles.secondaryButton}
                  onClick={() => history.push('/')}
                  type="button"
                >
                  Back to Home
                </button>
              </div>

              <div className="margin-bottom--lg">
                <button
                  className={clsx(styles.secondaryButton, styles.dangerButton)}
                  onClick={handleLogout}
                  type="button"
                >
                  Sign Out
                </button>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}

// Main component that wraps content with AuthProvider
export default function Profile() {
  return (
    <AuthProvider>
      <ProfileWithAuth />
    </AuthProvider>
  );
}