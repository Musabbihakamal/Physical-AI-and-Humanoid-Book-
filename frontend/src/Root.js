import React from 'react';
import { AuthProvider } from './contexts/AuthContext';

// Root component that wraps the entire application with necessary providers
// Note: TranslationProvider is handled separately in theme components to avoid SSR issues
export default function Root({ children }) {
  // For SSR, we don't want to initialize providers that access browser APIs
  if (typeof window === 'undefined') {
    return <>{children}</>;
  }

  return (
    <AuthProvider>
      {children}
    </AuthProvider>
  );
}