import React from 'react';
import { Redirect } from '@docusaurus/router';
import { useAuth } from '../contexts/AuthContext';

/**
 * PrivateRoute component that protects routes requiring authentication
 * Redirects unauthenticated users to the signin page
 */
const PrivateRoute = ({ children, redirectTo = '/auth/signin' }) => {
  const { isAuthenticated, loading } = useAuth();

  // Show loading state while checking authentication
  if (loading) {
    return (
      <div style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '50vh',
        flexDirection: 'column'
      }}>
        <div style={{
          width: '40px',
          height: '40px',
          border: '4px solid #f3f3f3',
          borderTop: '4px solid #3498db',
          borderRadius: '50%',
          animation: 'spin 1s linear infinite'
        }}></div>
        <p style={{ marginTop: '1rem', color: '#666' }}>Verifying authentication...</p>
        <style>{`
          @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
          }
        `}</style>
      </div>
    );
  }

  // Redirect to signin if not authenticated
  if (!isAuthenticated) {
    return <Redirect to={redirectTo} />;
  }

  // Render protected content if authenticated
  return children;
};

export default PrivateRoute;