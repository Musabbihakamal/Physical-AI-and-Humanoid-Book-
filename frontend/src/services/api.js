// frontend/src/services/api.js
import cache from '../utils/cache';
import { API_BASE_URL } from '../constants/apiConfig';

// Use a mock API URL for static deployments or fallback to backend
// API_BASE_URL is imported from constants/apiConfig.js which handles the environment configuration

class ApiService {
  constructor() {
    this.baseUrl = API_BASE_URL;
    this.refreshingToken = null; // To prevent multiple refresh requests
  }

  // Helper method to get auth headers
  getAuthHeaders() {
    let accessToken = null;
    if (typeof window !== 'undefined') {
      accessToken = localStorage.getItem('accessToken');
    }
    const headers = {
      'Content-Type': 'application/json',
    };

    if (accessToken) {
      headers['Authorization'] = `Bearer ${accessToken}`;
    }

    return headers;
  }

  // Helper method to refresh token
  async refreshToken() {
    if (this.refreshingToken) {
      // If already refreshing, wait for the existing refresh to complete
      return this.refreshingToken;
    }

    let refreshToken = null;
    if (typeof window !== 'undefined') {
      refreshToken = localStorage.getItem('refreshToken');
    }
    if (!refreshToken) {
      throw new Error('No refresh token available');
    }

    this.refreshingToken = fetch(`${this.baseUrl}/api/auth/refresh`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ refresh_token: refreshToken }),
    })
      .then(response => response.json())
      .then(data => {
        if (data.access_token && data.refresh_token) {
          if (typeof window !== 'undefined') {
            localStorage.setItem('accessToken', data.access_token);
            localStorage.setItem('refreshToken', data.refresh_token);
          }
          this.refreshingToken = null;
          return data;
        } else {
          throw new Error('Token refresh failed');
        }
      })
      .catch(error => {
        // Clear tokens if refresh fails
        if (typeof window !== 'undefined') {
          localStorage.removeItem('accessToken');
          localStorage.removeItem('refreshToken');
        }
        this.refreshingToken = null;
        throw error;
      });

    return this.refreshingToken;
  }

  // Helper method to make API requests with auth
  async request(endpoint, options = {}, includeAuth = true) {
    // If no base URL is configured (static deployment), return mock responses
    if (!this.baseUrl || this.baseUrl === '') {
      return this.getMockResponse(endpoint, options);
    }

    let config = {
      headers: includeAuth ? {
        ...this.getAuthHeaders(),
        ...options.headers,
      } : {
        'Content-Type': 'application/json',
        ...options.headers,
      },
      ...options,
    };

    let url = `${this.baseUrl}${endpoint}`;
    let response;
    let data;

    try {
      response = await fetch(url, config);

      // If we get a 401, try to refresh the token and retry the request
      if (response.status === 401) {
        try {
          await this.refreshToken();
          // Retry the request with the new token
          config.headers = {
            ...this.getAuthHeaders(),
            ...options.headers,
          };
          response = await fetch(url, config);
        } catch (refreshError) {
          console.error('Token refresh failed:', refreshError);
          // If refresh fails, clear tokens and let the app handle the 401
          if (typeof window !== 'undefined') {
            localStorage.removeItem('accessToken');
            localStorage.removeItem('refreshToken');
          }
          // Re-throw the error to be handled upstream
          throw refreshError;
        }
      }

      // Try to parse response as JSON
      const contentType = response.headers.get('content-type');
      if (contentType && contentType.includes('application/json')) {
        data = await response.json();
      } else {
        // For non-JSON responses, get text and try to parse if possible
        const text = await response.text();
        try {
          data = JSON.parse(text);
        } catch {
          data = { message: text };
        }
      }

      if (!response.ok) {
        // If we get a 401 after all retries, make sure to clear tokens
        if (response.status === 401) {
          if (typeof window !== 'undefined') {
            localStorage.removeItem('accessToken');
            localStorage.removeItem('refreshToken');
          }
        }
        throw new Error(data.detail?.message || data.message || `API request failed with status ${response.status}`);
      }

      return data;
    } catch (error) {
      // Check if this is a network error (fetch failed to reach the server)
      if (error.name === 'TypeError' && (error.message.includes('fetch') || error.message.includes('network'))) {
        console.error(`Network error connecting to ${url}:`, error.message);
        throw new Error(`Unable to connect to the server. Please ensure the backend server is running on ${this.baseUrl}. Network error: ${error.message}`);
      } else {
        // For 401 errors specifically, also clear tokens
        if (error.message && (error.message.includes('401') || response?.status === 401)) {
          if (typeof window !== 'undefined') {
            localStorage.removeItem('accessToken');
            localStorage.removeItem('refreshToken');
          }
        }
        console.error(`API request failed: ${endpoint}`, error);
        throw error;
      }
    }
  }

  // Mock responses for static deployment
  getMockResponse(endpoint, options = {}) {
    console.warn(`Mock API call for static deployment: ${endpoint}`);

    // Mock responses for different endpoints
    if (endpoint.includes('/api/auth/')) {
      // Mock auth responses
      if (endpoint.includes('/login') || endpoint.includes('/register')) {
        return Promise.resolve({
          access_token: 'mock_token',
          refresh_token: 'mock_refresh_token',
          user: { id: 1, email: 'user@example.com', name: 'Demo User' }
        });
      } else if (endpoint.includes('/me')) {
        return Promise.resolve({ id: 1, email: 'user@example.com', name: 'Demo User' });
      }
    }

    // Default mock response
    return Promise.resolve({ message: 'Mock response for static deployment' });
  }

  // Get user profile
  async getUserProfile() {
    return this.request('/api/auth/me');
  }

  // Update user profile
  async updateUserProfile(profileData) {
    return this.request('/api/auth/me', {
      method: 'PUT',
      body: JSON.stringify(profileData)
    });
  }

  // Get user session info
  async getSessionInfo() {
    return this.request('/api/session-info');
  }

  // Translate content
  async translateContent(text, targetLanguage = 'ur', sourceLanguage = 'en') {
    return this.request('/api/translate/translate', {
      method: 'POST',
      body: JSON.stringify({
        text: text,
        target_language: targetLanguage,
        source_language: sourceLanguage
      })
    });
  }

  // Get cached results for completed requests
  async getCachedResult(requestId, type) {
    const cacheKey = `result:${type}:${requestId}`;
    return cache.get(cacheKey);
  }

  // Cache results for completed requests
  async cacheResult(requestId, type, result) {
    const cacheKey = `result:${type}:${requestId}`;
    // Cache for 10 minutes (600000 ms)
    cache.set(cacheKey, result, 600000);
  }
}

// Create API service instance lazily to avoid SSR issues during module initialization
let apiServiceInstance = null;

const getApiService = () => {
  if (!apiServiceInstance) {
    apiServiceInstance = new ApiService();
  }
  return apiServiceInstance;
};

// Create an API object that mimics Axios interface
const api = {
  request: (endpoint, options = {}, includeAuth = true) => getApiService().request(endpoint, options, includeAuth),
  get: (endpoint, options = {}) => getApiService().request(endpoint, { ...options, method: 'GET' }),
  post: (endpoint, data, options = {}, includeAuth = true) => getApiService().request(endpoint, { ...options, method: 'POST', body: JSON.stringify(data) }, includeAuth),
  put: (endpoint, data, options = {}) => getApiService().request(endpoint, { ...options, method: 'PUT', body: JSON.stringify(data) }),
  delete: (endpoint, options = {}) => getApiService().request(endpoint, { ...options, method: 'DELETE' }),
  // Additional methods that might be needed
  getUserProfile: () => getApiService().getUserProfile(),
  updateUserProfile: (profileData) => getApiService().updateUserProfile(profileData),
  getSessionInfo: () => getApiService().getSessionInfo(),
  translateContent: (text, targetLanguage = 'ur', sourceLanguage = 'en') => getApiService().translateContent(text, targetLanguage, sourceLanguage),
  getCachedResult: (requestId, type) => getApiService().getCachedResult(requestId, type),
  cacheResult: (requestId, type, result) => getApiService().cacheResult(requestId, type, result),
};

export default api;
export { getApiService }; // Export the factory function for other use cases
