import React, { useEffect, useState } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import styles from './signin-template.module.css';
import { AuthProvider, useAuth } from '../../contexts/AuthContext';

// Helper function to parse query parameters
const parseQueryParams = () => {
  const searchParams = new URLSearchParams(window.location.search);
  const params = {};
  for (let [key, value] of searchParams) {
    params[key] = value;
  }
  return params;
};

function OAuthCallbackWithAuth() {
  const { siteConfig } = useDocusaurusContext();
  const { loginWithTokens } = useAuth(); // Assuming this function exists in your AuthContext
  const history = useHistory();
  const [loading, setLoading] = useState(true);
  const [message, setMessage] = useState('Processing authentication...');
  const [error, setError] = useState('');

  useEffect(() => {
    const handleOAuthCallback = async () => {
      try {
        // Parse query parameters from URL
        const queryParams = parseQueryParams();

        // Extract OAuth tokens and user data from URL parameters
        const {
          provider,
          access_token,
          refresh_token,
          user_id,
          email,
          full_name,
          is_new_user
        } = queryParams;

        // Check if all required parameters are present
        if (!provider || !access_token || !refresh_token || !user_id || !email || !full_name) {
          throw new Error('Missing required authentication parameters from OAuth provider');
        }

        // Process the OAuth login with the received tokens
        const result = await loginWithTokens({
          access_token,
          refresh_token,
          user_id,
          email,
          full_name,
          is_new_user: is_new_user === 'true' // Convert string to boolean
        });

        if (result.success) {
          setMessage(is_new_user === 'true'
            ? `Welcome! Successfully signed up with ${provider}. Redirecting...`
            : `Welcome back! Successfully signed in with ${provider}. Redirecting...`);

          // Redirect to home or dashboard after a short delay
          setTimeout(() => {
            if (typeof history.push === 'function') {
              history.push('/');
            } else {
              window.location.href = '/';
            }
          }, 2000);
        } else {
          throw new Error(result.error || 'Failed to process OAuth login');
        }
      } catch (err) {
        console.error('OAuth callback error:', err);
        setError(err.message || 'An error occurred during OAuth authentication');
        setMessage('Authentication failed');
        setLoading(false);

        // Provide a way for users to go back to login page
        setTimeout(() => {
          if (typeof history.push === 'function') {
            history.push('/auth/signin');
          } else {
            window.location.href = '/auth/signin';
          }
        }, 3000);
      }
    };

    // Clear the URL parameters to avoid confusion
    window.history.replaceState({}, document.title, window.location.pathname);

    // Process the OAuth callback
    handleOAuthCallback();
  }, []);

  return (
    <Layout
      title={`Authentication - ${siteConfig.title}`}
      description="Processing OAuth authentication">
      <main className={styles.authContainer}>
        <div className={styles.authCard}>
          <div className={styles.formSection}>
            <h1>Authentication Processing</h1>

            {error ? (
              <div className={clsx(styles.alert, styles['alert--danger'])}>
                <p>{error}</p>
                <p>Redirecting to login page...</p>
              </div>
            ) : (
              <div className={clsx(styles.alert, styles['alert--info'])}>
                <p>{message}</p>
                {loading && (
                  <div className={styles.loadingSpinner}>
                    <svg className={styles.spinner} width="20" height="20" viewBox="0 0 20 20" fill="none">
                      <circle cx="10" cy="10" r="8" stroke="currentColor" strokeWidth="2" fill="none" opacity="0.3" />
                      <path d="M10 2a8 8 0 018 8" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
                    </svg>
                  </div>
                )}
              </div>
            )}

            <div className="text--center margin-top--lg">
              <p>If you're not redirected automatically, <a href="/auth/signin">click here to go back to login</a>.</p>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}

// Main component that wraps content with AuthProvider
export default function OAuthCallback() {
  return (
    <AuthProvider>
      <OAuthCallbackWithAuth />
    </AuthProvider>
  );
}