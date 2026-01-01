import React, { useState } from 'react';
import { useHistory } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import styles from './signup-template.module.css';
import { AuthProvider, useAuth } from '../../contexts/AuthContext';
import { GOOGLE_OAUTH_URL, GITHUB_OAUTH_URL, API_BASE_URL } from '../../constants/apiConfig';

// Wrapper component to provide Auth context
function SignupWithAuth() {
  const { siteConfig } = useDocusaurusContext();
  const { register } = useAuth();
  const history = useHistory();
  const [formData, setFormData] = useState({
    name: '',
    email: '',
    password: '',
    confirmPassword: '',
    experience_level: 'BEGINNER',
    technical_background: '',
    preferred_difficulty: 'MEDIUM',
    learning_goals: [],
    hardware_access: [],
    language_preference: 'en'
  });
  const [error, setError] = useState('');
  const [showPassword, setShowPassword] = useState(false);
  const [showConfirmPassword, setShowConfirmPassword] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [passwordStrength, setPasswordStrength] = useState(0);

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));

    // Calculate password strength when password changes
    if (name === 'password') {
      calculatePasswordStrength(value);
    }

    // Clear error when user starts typing
    if (error) setError('');
  };

  const calculatePasswordStrength = (password) => {
    let strength = 0;
    if (password.length >= 8) strength += 1;
    if (/[A-Z]/.test(password)) strength += 1;
    if (/[a-z]/.test(password)) strength += 1;
    if (/[0-9]/.test(password)) strength += 1;
    if (/[^A-Za-z0-9]/.test(password)) strength += 1;
    setPasswordStrength(strength);
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    // Validation
    if (!formData.name || !formData.email || !formData.password || !formData.confirmPassword) {
      setError('Please fill in all fields');
      return;
    }

    // Name validation
    if (formData.name.trim().split(' ').length < 2) {
      setError('Please enter your full name (first and last name)');
      return;
    }

    // Email validation
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(formData.email)) {
      setError('Please enter a valid email address');
      return;
    }

    // Password validation
    if (formData.password.length < 8) {
      setError('Password must be at least 8 characters');
      return;
    }

    if (formData.password !== formData.confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    // Password strength validation
    if (passwordStrength < 3) {
      setError('Please choose a stronger password. Use uppercase, lowercase, numbers, and special characters.');
      return;
    }

    setIsLoading(true);
    setError('');

    try {
      const result = await register({
        email: formData.email,
        password: formData.password,
        full_name: formData.name,
        experience_level: formData.experience_level,
        technical_background: formData.technical_background,
        preferred_difficulty: formData.preferred_difficulty,
        learning_goals: formData.learning_goals,
        hardware_access: formData.hardware_access,
        language_preference: formData.language_preference
      });

      if (result.success) {
        // Registration successful, redirect to home or dashboard
        if (typeof history.push === 'function') {
          history.push('/');
        } else {
          // Fallback to window.location if history.push is not available
          window.location.href = '/';
        }
      } else {
        setError(result.error || 'Registration failed. Please try again.');
      }
    } catch (err) {
      setError(err.message || 'Registration failed. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  const togglePasswordVisibility = () => {
    setShowPassword(!showPassword);
  };

  const toggleConfirmPasswordVisibility = () => {
    setShowConfirmPassword(!showConfirmPassword);
  };

  const getPasswordStrengthText = () => {
    if (passwordStrength === 0) return 'Very Weak';
    if (passwordStrength === 1) return 'Weak';
    if (passwordStrength === 2) return 'Fair';
    if (passwordStrength === 3) return 'Good';
    if (passwordStrength >= 4) return 'Strong';
  };

  const getPasswordStrengthColor = () => {
    if (passwordStrength === 0) return '#e53e3e'; // red
    if (passwordStrength === 1) return '#dd6b20'; // orange
    if (passwordStrength === 2) return '#d69e2e'; // yellow
    if (passwordStrength === 3) return '#38a169'; // green
    if (passwordStrength >= 4) return '#38a169'; // green
  };

  // OAuth Login Functions
  const handleGoogleLogin = async () => {
    try {
      setIsLoading(true);
      setError('');

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

      // Call the backend to get the Google OAuth URL
      const response = await fetch(GOOGLE_OAUTH_URL, {
        method: 'GET',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (response.ok) {
        const data = await response.json();

        // Redirect to Google OAuth URL
        if (data.auth_url) {
          window.location.href = data.auth_url;
        } else {
          throw new Error('Invalid response from server - no auth_url provided');
        }
      } else if (response.status === 404) {
        throw new Error('OAuth endpoints not found. Backend server may not be running or OAuth is not properly configured.');
      } else {
        const errorData = await response.json();
        throw new Error(errorData.detail || `Failed to initiate Google login (${response.status})`);
      }
    } catch (err) {
      console.error('Google OAuth error:', err);
      // Handle the specific "Failed to fetch" error with more helpful message
      if (err.message.includes('Failed to fetch')) {
        setError('Cannot connect to the authentication server. Please make sure the backend server is running and accessible.');
      } else {
        setError(err.message || 'Google login failed. Please try again or use email/password instead.');
      }
    } finally {
      setIsLoading(false);
    }
  };

  const handleGitHubLogin = async () => {
    try {
      setIsLoading(true);
      setError('');

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

      // Call the backend to get the GitHub OAuth URL
      const response = await fetch(GITHUB_OAUTH_URL, {
        method: 'GET',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (response.ok) {
        const data = await response.json();

        // Redirect to GitHub OAuth URL
        if (data.auth_url) {
          window.location.href = data.auth_url;
        } else {
          throw new Error('Invalid response from server - no auth_url provided');
        }
      } else if (response.status === 404) {
        throw new Error('OAuth endpoints not found. Backend server may not be running or OAuth is not properly configured.');
      } else {
        const errorData = await response.json();
        throw new Error(errorData.detail || `Failed to initiate GitHub login (${response.status})`);
      }
    } catch (err) {
      console.error('GitHub OAuth error:', err);
      // Handle the specific "Failed to fetch" error with more helpful message
      if (err.message.includes('Failed to fetch')) {
        setError('Cannot connect to the authentication server. Please make sure the backend server is running and accessible.');
      } else {
        setError(err.message || 'GitHub login failed. Please try again or use email/password instead.');
      }
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Layout
      title={`Create Account - ${siteConfig.title}`}
      description="Create an account to access the Physical AI & Humanoid Robotics learning platform">
      <main className={styles.authContainer}>
        <div className={styles.authCard}>
          {/* Left side with image and text */}
          <div className={styles.imageSection}>
            <div className={styles.imageSectionContent}>
              <h2>Join Our Community of Innovators</h2>
              <p>Connect with like-minded professionals and access exclusive content on Physical AI & Humanoid Robotics</p>
            </div>
          </div>

          {/* Right side with form */}
          <div className={styles.formSection}>
            <h1>Create Your Account</h1>
            <p className={styles.subtitle}>Join our community to access exclusive content</p>

            {error && (
              <div className={clsx(styles.alert, styles['alert--danger'])}>
                {error}
              </div>
            )}

            <form onSubmit={handleSubmit} className={styles.form}>
              <div className={styles.formGroup}>
                <label htmlFor="name" className={styles.formLabel}>Full Name</label>
                <div className={styles.inputWithIcon}>
                  <svg className={styles.inputIcon} width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
                    <path fillRule="evenodd" d="M10 9a3 3 0 100-6 3 3 0 000 6zm-7 9a7 7 0 1114 0H3z" clipRule="evenodd" />
                  </svg>
                  <input
                    type="text"
                    id="name"
                    name="name"
                    className={styles.formControl}
                    value={formData.name}
                    onChange={handleChange}
                    placeholder="Enter your full name"
                    autoComplete="name"
                  />
                </div>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="email" className={styles.formLabel}>Email Address</label>
                <div className={styles.inputWithIcon}>
                  <svg className={styles.inputIcon} width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
                    <path d="M2.003 5.884L10 9.882l7.997-3.998A2 2 0 0016 4H4a2 2 0 00-1.997 1.884z" />
                    <path d="M18 8.118l-8 4-8-4V14a2 2 0 002 2h12a2 2 0 002-2V8.118z" />
                  </svg>
                  <input
                    type="email"
                    id="email"
                    name="email"
                    className={styles.formControl}
                    value={formData.email}
                    onChange={handleChange}
                    placeholder="Enter your email address"
                    autoComplete="email"
                  />
                </div>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="password" className={styles.formLabel}>Password</label>
                <div className={styles.inputWithIcon}>
                  <svg className={styles.inputIcon} width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
                    <path fillRule="evenodd" d="M5 9V7a5 5 0 0110 0v2a2 2 0 012 2v5a2 2 0 01-2 2H5a2 2 0 01-2-2v-5a2 2 0 012-2zm8-2v2H7V7a3 3 0 016 0z" clipRule="evenodd" />
                  </svg>
                  <input
                    type={showPassword ? "text" : "password"}
                    id="password"
                    name="password"
                    className={styles.formControl}
                    value={formData.password}
                    onChange={handleChange}
                    placeholder="Create a strong password"
                    autoComplete="new-password"
                  />
                  <button
                    type="button"
                    className={styles.passwordToggle}
                    onClick={togglePasswordVisibility}
                    aria-label={showPassword ? "Hide password" : "Show password"}
                  >
                    {showPassword ? (
                      <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
                        <path d="M10 12a2 2 0 100-4 2 2 0 000 4z" />
                        <path fillRule="evenodd" d="M.458 10C1.732 5.943 5.522 3 10 3s8.268 2.943 9.542 7c-1.274 4.057-5.064 7-9.542 7S1.732 14.057.458 10zM14 10a4 4 0 11-8 0 4 4 0 018 0z" clipRule="evenodd" />
                      </svg>
                    ) : (
                      <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
                        <path fillRule="evenodd" d="M3.707 2.293a1 1 0 00-1.414 1.414l14 14a1 1 0 001.414-1.414l-1.473-1.473A10.014 10.014 0 0019.542 10C18.268 5.943 14.478 3 10 3a9.958 9.958 0 00-4.512 1.074l-1.78-1.781zm4.261 4.26l1.514 1.515a2.003 2.003 0 012.45 2.45l1.514 1.514a4 4 0 00-5.478-5.478z" clipRule="evenodd" />
                        <path d="M12.454 16.697L9.75 13.992a4 4 0 01-3.742-3.741L2.335 6.578A9.98 9.98 0 00.458 10c1.274 4.057 5.065 7 9.542 7 .847 0 1.669-.105 2.454-.303z" />
                      </svg>
                    )}
                  </button>
                </div>
                <div className={styles.passwordStrength}>
                  <div className={styles.passwordStrengthBar}>
                    <div
                      className={styles.passwordStrengthFill}
                      style={{
                        width: `${(passwordStrength / 5) * 100}%`,
                        backgroundColor: getPasswordStrengthColor()
                      }}
                    ></div>
                  </div>
                  <small className={styles.helpText} style={{ color: getPasswordStrengthColor() }}>
                    Password strength: {getPasswordStrengthText()}
                  </small>
                </div>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="confirmPassword" className={styles.formLabel}>Confirm Password</label>
                <div className={styles.inputWithIcon}>
                  <svg className={styles.inputIcon} width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
                    <path fillRule="evenodd" d="M5 9V7a5 5 0 0110 0v2a2 2 0 012 2v5a2 2 0 01-2 2H5a2 2 0 01-2-2v-5a2 2 0 012-2zm8-2v2H7V7a3 3 0 016 0z" clipRule="evenodd" />
                  </svg>
                  <input
                    type={showConfirmPassword ? "text" : "password"}
                    id="confirmPassword"
                    name="confirmPassword"
                    className={styles.formControl}
                    value={formData.confirmPassword}
                    onChange={handleChange}
                    placeholder="Confirm your password"
                    autoComplete="new-password"
                  />
                  <button
                    type="button"
                    className={styles.passwordToggle}
                    onClick={toggleConfirmPasswordVisibility}
                    aria-label={showConfirmPassword ? "Hide password" : "Show password"}
                  >
                    {showConfirmPassword ? (
                      <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
                        <path d="M10 12a2 2 0 100-4 2 2 0 000 4z" />
                        <path fillRule="evenodd" d="M.458 10C1.732 5.943 5.522 3 10 3s8.268 2.943 9.542 7c-1.274 4.057-5.064 7-9.542 7S1.732 14.057.458 10zM14 10a4 4 0 11-8 0 4 4 0 018 0z" clipRule="evenodd" />
                      </svg>
                    ) : (
                      <svg width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
                        <path fillRule="evenodd" d="M3.707 2.293a1 1 0 00-1.414 1.414l14 14a1 1 0 001.414-1.414l-1.473-1.473A10.014 10.014 0 0019.542 10C18.268 5.943 14.478 3 10 3a9.958 9.958 0 00-4.512 1.074l-1.78-1.781zm4.261 4.26l1.514 1.515a2.003 2.003 0 012.45 2.45l1.514 1.514a4 4 0 00-5.478-5.478z" clipRule="evenodd" />
                        <path d="M12.454 16.697L9.75 13.992a4 4 0 01-3.742-3.741L2.335 6.578A9.98 9.98 0 00.458 10c1.274 4.057 5.065 7 9.542 7 .847 0 1.669-.105 2.454-.303z" />
                      </svg>
                    )}
                  </button>
                </div>
              </div>

              <div className={styles.formGroup}>
                <label className={styles.checkbox}>
                  <input type="checkbox" required />
                  <span className={styles.checkboxCheckmark}></span>
                  <span className={styles.checkboxText}>I agree to the <a href="/tos">Terms of Service</a> and <a href="/privacy">Privacy Policy</a></span>
                </label>
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
                    Creating account...
                  </div>
                ) : (
                  'Create Account'
                )}
              </button>
            </form>

            <div className={styles.authLinks}>
              <div className="margin-bottom--lg">
                <button
                  className={styles.secondaryButton}
                  onClick={() => {
                    if (typeof history.push === 'function') {
                      history.push('/auth/signin');
                    } else {
                      window.location.href = '/auth/signin';
                    }
                  }}
                  type="button"
                >
                  Sign In to Existing Account
                </button>
              </div>
              <p className="margin-bottom--sm text--center">
                Already have an account? <a href="/auth/signin">Sign in</a>
              </p>
              <div className={styles.socialLogin}>
                <p>Or sign up with</p>
                <div className={styles.socialButtons}>
                  <button
                    className={clsx(styles.socialButton, styles.googleButton)}
                    onClick={handleGoogleLogin}
                    type="button"
                  >
                    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                      <path d="M22.56 12.25C22.56 11.47 22.49 10.72 22.36 10H12V14.26H17.92C17.66 15.63 16.88 16.79 15.71 17.57V20.34H19.28C21.36 18.42 22.56 15.6 22.56 12.25Z" fill="#4285F4"/>
                      <path d="M12 23C14.97 23 17.46 22.02 19.28 20.34L15.71 17.57C14.73 18.23 13.48 18.64 12 18.64C9.14 18.64 6.72 16.69 5.85 14.05H2.18V16.86C4.04 20.53 7.72 23 12 23Z" fill="#34A853"/>
                      <path d="M5.85 14.05C5.63 13.38 5.5 12.68 5.5 12C5.5 11.32 5.63 10.62 5.85 9.95V7.14H2.18C1.43 8.64 1 10.28 1 12C1 13.72 1.43 15.36 2.18 16.86L5.85 14.05Z" fill="#FBBC05"/>
                      <path d="M12 5.36C13.62 5.36 15.06 5.93 16.21 7.03L19.39 3.85C17.44 2.03 14.96 1 12 1C7.72 1 4.04 3.47 2.18 7.14L5.85 9.95C6.72 7.31 9.14 5.36 12 5.36Z" fill="#EA4335"/>
                    </svg>
                    Google
                  </button>
                  <button
                    className={clsx(styles.socialButton, styles.githubButton)}
                    onClick={handleGitHubLogin}
                    type="button"
                  >
                    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                      <path d="M12 0C5.37 0 0 5.37 0 12C0 17.3 3.43 21.8 8.2 23.39C8.8 23.5 9 23.12 9 22.78V20.66C5.72 21.33 5.05 19.2 5.05 19.2C4.47 17.75 3.64 17.38 3.64 17.38C2.5 16.63 3.71 16.65 3.71 16.65C4.95 16.73 5.55 17.88 5.55 17.88C6.65 19.67 8.38 19.16 9.09 18.86C9.2 18.08 9.52 17.5 9.88 17.2C7.15 16.9 4.27 15.86 4.27 11.47C4.27 10.39 4.65 9.49 5.27 8.79C5.16 8.51 4.81 7.49 5.43 5.85C5.43 5.85 6.28 5.58 8 6.77C8.77 6.56 9.6 6.46 10.43 6.45C11.26 6.46 12.09 6.56 12.86 6.77C14.6 5.58 15.44 5.85 15.44 5.85C16.06 7.49 15.71 8.51 15.6 8.79C16.22 9.49 16.6 10.39 16.6 11.47C16.6 15.88 13.71 16.89 10.97 17.19C11.43 17.58 11.85 18.33 11.85 19.5V22.78C11.85 23.12 12.05 23.5 12.65 23.39C17.4 21.8 20.8 17.3 20.8 12C20.8 5.37 15.43 0 12 0Z" fill="currentColor"/>
                    </svg>
                    GitHub
                  </button>
                </div>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}

// Main component that wraps content with AuthProvider
export default function Signup() {
  return (
    <AuthProvider>
      <SignupWithAuth />
    </AuthProvider>
  );
}