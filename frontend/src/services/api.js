// frontend/src/services/api.js
import cache from '../utils/cache';
import { getEnvVar, isProduction } from '../utils/env';

// Use a mock API URL for static deployments or fallback to backend
const API_BASE_URL = getEnvVar('REACT_APP_BACKEND_URL',
  isProduction() ? '' : 'http://localhost:8000'); // Empty string for static deployment

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
  async request(endpoint, options = {}) {
    // If no base URL is configured (static deployment), return mock responses
    if (!this.baseUrl || this.baseUrl === '') {
      return this.getMockResponse(endpoint, options);
    }

    let config = {
      headers: {
        ...this.getAuthHeaders(),
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
        throw new Error(data.detail?.message || data.message || 'API request failed');
      }

      return data;
    } catch (error) {
      console.error(`API request failed: ${endpoint}`, error);
      throw error;
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
    } else if (endpoint.includes('/api/agents/code-explainer')) {
      return Promise.resolve({
        request_id: 'mock-request-id',
        status: 'COMPLETED',
        result: {
          overview: 'This is a mock code explanation for static deployment.',
          key_components: [{ type: 'function', name: 'example', declaration: 'def example():' }],
          ros2_specifics: ['rclpy'],
          isaac_sim_specifics: ['omni.isaac'],
          technical_concepts: ['Mock concept'],
          usage_context: 'Mock usage',
          learning_points: ['Mock learning point']
        }
      });
    } else if (endpoint.includes('/api/agents/status/')) {
      return Promise.resolve({
        status: 'COMPLETED',
        progress: 100
      });
    }

    // Default mock response
    return Promise.resolve({ message: 'Mock response for static deployment' });
  }

  // Code Explainer API
  async explainCode(code, language = null, parameters = {}) {
    return this.request('/api/agents/code-explainer', {
      method: 'POST',
      body: JSON.stringify({
        code,
        language,
        parameters,
      }),
    });
  }


  // Get request status
  async getRequestStatus(requestId) {
    // Don't cache status requests as they change frequently
    return this.request(`/api/agents/status/${requestId}`);
  }

  // Get user profile
  async getUserProfile() {
    return this.request('/api/auth/me');
  }

  // Get user session info
  async getSessionInfo() {
    return this.request('/api/agents/session-info');
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

export default getApiService;