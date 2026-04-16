// Frontend API Configuration
// This file handles the API configuration for the frontend

// Function to get the backend URL
export const getBackendUrl = () => {
  // In browser environment, try to get from various sources
  if (typeof window !== 'undefined') {
    // Priority 1: Environment variable (build-time or runtime)
    if (process.env.REACT_APP_BACKEND_URL) {
      return process.env.REACT_APP_BACKEND_URL;
    }

    // Priority 2: Window object (for runtime configuration)
    if (window.REACT_APP_BACKEND_URL) {
      return window.REACT_APP_BACKEND_URL;
    }

    // Priority 3: Detect environment
    const protocol = window.location.protocol;
    const hostname = window.location.hostname;

    // For localhost development
    if (hostname === 'localhost' || hostname === '127.0.0.1') {
      return `${protocol}//${hostname}:8000`;
    }

    // For Vercel production - requires REACT_APP_BACKEND_URL env var
    if (hostname.includes('vercel.app')) {
      console.warn('⚠️ REACT_APP_BACKEND_URL not configured for Vercel deployment');
      return 'https://api.example.com'; // Fallback - will fail
    }

    // Generic production fallback
    return `${protocol}//${hostname}:8000`;
  }

  // Fallback for server-side rendering (Node.js environment)
  return typeof process !== 'undefined' && process.env?.REACT_APP_BACKEND_URL
    ? process.env.REACT_APP_BACKEND_URL
    : 'http://localhost:8000';
};

// API base URL
export const API_BASE_URL = getBackendUrl();

// Specific endpoints
export const GOOGLE_OAUTH_URL = `${API_BASE_URL}/api/auth/google`;
export const GITHUB_OAUTH_URL = `${API_BASE_URL}/api/auth/github`;

// Function to check if backend is accessible
export const checkBackendConnection = async (timeout = 5000) => {
  try {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), timeout);

    const response = await fetch(`${API_BASE_URL}/health`, {
      signal: controller.signal,
    });

    clearTimeout(timeoutId);
    return response.ok;
  } catch (error) {
    console.error('Backend connection failed:', error);
    return false;
  }
};