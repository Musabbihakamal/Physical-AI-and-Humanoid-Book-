// Frontend API Configuration
// This file handles the API configuration for the frontend

// Function to get the backend URL
export const getBackendUrl = () => {
  // In browser environment, try to get from various sources
  if (typeof window !== 'undefined') {
    // Try to get from window object (set by environment or build process)
    if (window.REACT_APP_BACKEND_URL) {
      return window.REACT_APP_BACKEND_URL;
    }

    // Fallback to current host with backend port
    const protocol = window.location.protocol;
    const hostname = window.location.hostname;

    // For localhost development, the backend typically runs on port 8000 while frontend runs on 3000
    // So we should always target port 8000 for backend API calls
    // Special handling for localhost to avoid common development issues
    if (hostname === 'localhost' || hostname === '127.0.0.1') {
      return `${protocol}//${hostname}:8000`;
    } else {
      // For other hosts, construct the backend URL appropriately
      return `${protocol}//${hostname}:8000`;
    }
  }

  // Fallback for server-side rendering or other environments
  return 'http://localhost:8000';
};

// API base URL
export const API_BASE_URL = getBackendUrl();

// Specific endpoints
export const GOOGLE_OAUTH_URL = `${API_BASE_URL}/api/auth/google`;
export const GITHUB_OAUTH_URL = `${API_BASE_URL}/api/auth/github`;

// Function to check if backend is accessible
export const checkBackendConnection = async () => {
  try {
    const response = await fetch(`${API_BASE_URL}/health`);
    return response.ok;
  } catch (error) {
    console.error('Backend connection failed:', error);
    return false;
  }
};