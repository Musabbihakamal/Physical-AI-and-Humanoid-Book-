// frontend/src/utils/env.js
// Utility functions to safely access environment variables in both Node.js and browser environments

export function getEnvVar(name, defaultValue = undefined) {
  // Check for process.env first (Node.js/CRA environment)
  if (typeof process !== 'undefined' && process.env && process.env[name] !== undefined) {
    return process.env[name];
  }

  // Fallback to a global environment object if available
  if (typeof window !== 'undefined' && window.REACT_APP_ENV && window.REACT_APP_ENV[name] !== undefined) {
    return window.REACT_APP_ENV[name];
  }

  // Return default value if provided
  return defaultValue;
}

export function isDevelopment() {
  return getEnvVar('NODE_ENV', 'development') === 'development';
}

export function isProduction() {
  return getEnvVar('NODE_ENV', 'development') === 'production';
}