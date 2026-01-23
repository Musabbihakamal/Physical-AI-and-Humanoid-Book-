import React, { useState } from 'react';
import { Redirect } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import styles from './auth.module.css';

export default function ForgotPassword() {
  const { siteConfig } = useDocusaurusContext();
  const [email, setEmail] = useState('');
  const [submitted, setSubmitted] = useState(false);
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();

    // Validation
    if (!email) {
      setError('Please enter your email address');
      return;
    }

    // Email validation
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(email)) {
      setError('Please enter a valid email address');
      return;
    }

    setIsLoading(true);

    try {
      // Simulate API call delay
      await new Promise(resolve => setTimeout(resolve, 1500));

      // In a real implementation, you would send a password reset email
      setSubmitted(true);
      setError('');
    } catch (err) {
      setError('Failed to send reset email. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  if (submitted) {
    return (
      <Layout
        title={`Password Reset - ${siteConfig.title}`}
        description="Reset your password for the Physical AI & Humanoid Robotics learning platform">
        <main className={clsx('container', 'margin-vert--xl', styles.authContainer)}>
          <div className="row">
            <div className="col col--8 col--offset-2">
              <div className={clsx('padding--lg', styles.authCard)}>
                <div className="text--center margin-bottom--lg">
                  <div className={styles.logoContainer}>
                    <div className={styles.robotIcon}>
                      <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                        <path d="M12 2C13.1 2 14 2.9 14 4C14 5.1 13.1 6 12 6C10.9 6 10 5.1 10 4C10 2.9 10.9 2 12 2Z" fill="#4b86b4"/>
                        <path d="M21 9V7C21 5.9 20.1 5 19 5H16.1C15.8 4.4 15.2 4 14.5 4H9.4C8.8 4 8.2 4.4 7.9 5H5C3.9 5 3 5.9 3 7V9C1.9 9 1 9.9 1 11V15C1 16.1 1.9 17 3 17H5C5.6 19.4 7.8 21 10.5 21H13.5C16.2 21 18.4 19.4 19 17H21C22.1 17 23 16.1 23 15V11C23 9.9 22.1 9 21 9ZM19 15C19 15 18.9 15.5 18.6 15.8C18.2 16.2 17.6 16.5 17 16.5H7C6.4 16.5 5.8 16.2 5.4 15.8C5.1 15.5 5 15 5 15V11H19V15Z" fill="#2a4d69"/>
                        <path d="M8 11H6V13H8V11Z" fill="#4b86b4"/>
                        <path d="M18 11H16V13H18V11Z" fill="#4b86b4"/>
                        <path d="M15 7H9V9H15V7Z" fill="#4b86b4"/>
                      </svg>
                    </div>
                    <h1 className={styles.signInTitle}>Password Reset Email Sent</h1>
                    <p className="text--secondary">Check your email for instructions to reset your password</p>
                  </div>
                </div>

                <div className="alert alert--success margin-bottom--md">
                  If an account exists for <strong>{email}</strong>, you will receive a password reset link shortly.
                </div>

                <div className="text--center">
                  <a href="/auth/signin" className="button--secondary">
                    Back to Sign In
                  </a>
                </div>
              </div>
            </div>
          </div>
        </main>
      </Layout>
    );
  }

  return (
    <Layout
      title={`Reset Password - ${siteConfig.title}`}
      description="Reset your password for the Physical AI & Humanoid Robotics learning platform">
      <main className={clsx('container', 'margin-vert--xl', styles.authContainer)}>
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className={clsx('padding--lg', styles.authCard)}>
              <div className="text--center margin-bottom--lg">
                <div className={styles.logoContainer}>
                  <div className={styles.robotIcon}>
                    <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                      <path d="M12 2C13.1 2 14 2.9 14 4C14 5.1 13.1 6 12 6C10.9 6 10 5.1 10 4C10 2.9 10.9 2 12 2Z" fill="#4b86b4"/>
                      <path d="M21 9V7C21 5.9 20.1 5 19 5H16.1C15.8 4.4 15.2 4 14.5 4H9.4C8.8 4 8.2 4.4 7.9 5H5C3.9 5 3 5.9 3 7V9C1.9 9 1 9.9 1 11V15C1 16.1 1.9 17 3 17H5C5.6 19.4 7.8 21 10.5 21H13.5C16.2 21 18.4 19.4 19 17H21C22.1 17 23 16.1 23 15V11C23 9.9 22.1 9 21 9ZM19 15C19 15 18.9 15.5 18.6 15.8C18.2 16.2 17.6 16.5 17 16.5H7C6.4 16.5 5.8 16.2 5.4 15.8C5.1 15.5 5 15 5 15V11H19V15Z" fill="#2a4d69"/>
                      <path d="M8 11H6V13H8V11Z" fill="#72e9e1"/>
                      <path d="M18 11H16V13H18V11Z" fill="#67e1cd"/>
                      <path d="M15 7H9V9H15V7Z" fill="#5be9d1"/>
                    </svg>
                  </div>
                  <h1 className={styles.signInTitle}>Reset Your Password</h1>
                  <p className="text--secondary">Enter your email address and we'll send you a link to reset your password</p>
                </div>
              </div>

              {error && (
                <div className="alert alert--danger margin-bottom--md">
                  {error}
                </div>
              )}

              <form onSubmit={handleSubmit}>
                <div className="form-group">
                  <label htmlFor="email" className="form-label">Email Address</label>
                  <div className="input-with-icon">
                    <svg className={styles.inputIcon} width="20" height="20" viewBox="0 0 20 20" fill="currentColor">
                      <path d="M2.003 5.884L10 9.882l7.997-3.998A2 2 0 0016 4H4a2 2 0 00-1.997 1.884z" />
                      <path d="M18 8.118l-8 4-8-4V14a2 2 0 002 2h12a2 2 0 002-2V8.118z" />
                    </svg>
                    <input
                      type="email"
                      id="email"
                      name="email"
                      className="form-control"
                      value={email}
                      onChange={(e) => {
                        setEmail(e.target.value);
                        if (error) setError('');
                      }}
                      placeholder="Enter your email address"
                      autoComplete="email"
                    />
                  </div>
                </div>

                <button
                  type="submit"
                  className={clsx("button--primary", styles.buttonPrimary)}
                  disabled={isLoading}
                >
                  {isLoading ? (
                    <div className={styles.loadingSpinner}>
                      <svg className={styles.spinner} width="20" height="20" viewBox="0 0 20 20" fill="none">
                        <circle cx="10" cy="10" r="8" stroke="currentColor" strokeWidth="2" fill="none" opacity="0.3" />
                        <path d="M10 2a8 8 0 018 8" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
                      </svg>
                      Sending reset link...
                    </div>
                  ) : (
                    'Send Reset Link'
                  )}
                </button>
              </form>

              <div className="auth-links">
                <p className="margin-bottom--sm">
                  Remember your password? <a href="/auth/signin">Sign in</a>
                </p>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}