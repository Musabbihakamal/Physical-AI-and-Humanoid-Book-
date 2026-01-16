// frontend/src/contexts/AuthContext.js
import React, { createContext, useContext, useReducer, useEffect } from 'react';
import apiService from '../services/api';

const AuthContext = createContext();

const authReducer = (state, action) => {
  switch (action.type) {
    case 'LOGIN_START':
      return { ...state, loading: true, error: null };
    case 'LOGIN_SUCCESS':
      return {
        ...state,
        loading: false,
        isAuthenticated: true,
        user: action.payload.user,
        accessToken: action.payload.accessToken,
        refreshToken: action.payload.refreshToken
      };
    case 'LOGIN_FAILURE':
      return {
        ...state,
        loading: false,
        isAuthenticated: false,
        user: null,
        accessToken: null,
        refreshToken: null,
        error: action.payload
      };
    case 'REGISTER_START':
      return { ...state, loading: true, error: null };
    case 'REGISTER_SUCCESS':
      return {
        ...state,
        loading: false,
        isAuthenticated: true,
        user: action.payload.user,
        accessToken: action.payload.accessToken,
        refreshToken: action.payload.refreshToken
      };
    case 'REGISTER_FAILURE':
      return {
        ...state,
        loading: false,
        isAuthenticated: false,
        user: null,
        accessToken: null,
        refreshToken: null,
        error: action.payload
      };
    case 'LOGOUT':
      return {
        ...state,
        isAuthenticated: false,
        user: null,
        accessToken: null,
        refreshToken: null
      };
    case 'SET_ERROR':
      return { ...state, error: action.payload };
    case 'CLEAR_ERROR':
      return { ...state, error: null };
    default:
      return state;
  }
};

const AuthProvider = ({ children }) => {
  const [state, dispatch] = useReducer(authReducer, {
    isAuthenticated: false,
    user: null,
    accessToken: null,
    refreshToken: null,
    loading: false,
    error: null
  });

  // Check for existing tokens on app start
  useEffect(() => {
    // Check if we're in the browser environment
    if (typeof window !== 'undefined') {
      const accessToken = localStorage.getItem('accessToken');
      const refreshToken = localStorage.getItem('refreshToken');

      if (accessToken && refreshToken) {
        // Verify token validity by getting user profile
        const verifyToken = async () => {
          try {
            const response = await apiService.getUserProfile();

            dispatch({
              type: 'LOGIN_SUCCESS',
              payload: {
                user: response,
                accessToken,
                refreshToken
              }
            });
          } catch (error) {
            // Token verification failed, clear stored tokens
            localStorage.removeItem('accessToken');
            localStorage.removeItem('refreshToken');
            dispatch({ type: 'LOGOUT' });
          }
        };

        verifyToken();
      }
    }
  }, []);

  const login = async ({ email, password }) => {
    dispatch({ type: 'LOGIN_START' });

    try {
      // Use the apiService instead of direct fetch
      const response = await apiService.request('/api/auth/login', {
        method: 'POST',
        body: JSON.stringify({ email, password }),
      });

      // Store tokens in localStorage (only in browser)
      if (typeof window !== 'undefined') {
        localStorage.setItem('accessToken', response.access_token);
        localStorage.setItem('refreshToken', response.refresh_token);
      }

      dispatch({
        type: 'LOGIN_SUCCESS',
        payload: {
          user: response,
          accessToken: response.access_token,
          refreshToken: response.refresh_token
        }
      });

      return { success: true, user: response };
    } catch (error) {
      const errorMessage = error.message || 'Login failed';
      dispatch({
        type: 'LOGIN_FAILURE',
        payload: errorMessage
      });
      return { success: false, error: errorMessage };
    }
  };

  const register = async (userData) => {
    dispatch({ type: 'REGISTER_START' });

    try {
      // Use the apiService instead of direct fetch
      const response = await apiService.request('/api/auth/register', {
        method: 'POST',
        body: JSON.stringify({
          email: userData.email,
          password: userData.password,
          full_name: userData.full_name || userData.name,
          experience_level: userData.experience_level || 'BEGINNER',
          technical_background: userData.technical_background || '',
          preferred_difficulty: userData.preferred_difficulty || 'MEDIUM',
          learning_goals: userData.learning_goals || [],
          hardware_access: userData.hardware_access || [],
          language_preference: userData.language_preference || 'en'
        })
      });

      // Store tokens in localStorage (only in browser)
      if (typeof window !== 'undefined') {
        localStorage.setItem('accessToken', response.access_token);
        localStorage.setItem('refreshToken', response.refresh_token);
      }

      dispatch({
        type: 'REGISTER_SUCCESS',
        payload: {
          user: response,
          accessToken: response.access_token,
          refreshToken: response.refresh_token
        }
      });

      return { success: true, user: response };
    } catch (error) {
      const errorMessage = error.message || 'Registration failed';
      dispatch({
        type: 'REGISTER_FAILURE',
        payload: errorMessage
      });
      return { success: false, error: errorMessage };
    }
  };

  const logout = () => {
    // Clear tokens from localStorage (only in browser)
    if (typeof window !== 'undefined') {
      localStorage.removeItem('accessToken');
      localStorage.removeItem('refreshToken');
    }

    dispatch({ type: 'LOGOUT' });
  };

  const loginWithTokens = async (tokensData) => {
    dispatch({ type: 'LOGIN_START' });

    try {
      const { access_token, refresh_token, user_id, email, full_name, is_new_user } = tokensData;

      // Store tokens in localStorage (only in browser)
      if (typeof window !== 'undefined') {
        localStorage.setItem('accessToken', access_token);
        localStorage.setItem('refreshToken', refresh_token);
      }

      // Create a mock user object with the data from OAuth
      const user = {
        user_id,
        email,
        full_name,
        is_new_user
      };

      dispatch({
        type: 'LOGIN_SUCCESS',
        payload: {
          user,
          accessToken: access_token,
          refreshToken: refresh_token
        }
      });

      return { success: true, user };
    } catch (error) {
      const errorMessage = error.message || 'OAuth login failed';
      dispatch({
        type: 'LOGIN_FAILURE',
        payload: errorMessage
      });
      return { success: false, error: errorMessage };
    }
  };

  const updateProfile = async (profileData) => {
    dispatch({ type: 'REGISTER_START' });

    try {
      const response = await apiService.updateUserProfile(profileData);

      dispatch({
        type: 'REGISTER_SUCCESS',
        payload: {
          user: response,
          accessToken: response.access_token || localStorage.getItem('accessToken'),
          refreshToken: response.refresh_token || localStorage.getItem('refreshToken')
        }
      });

      return { success: true, user: response };
    } catch (error) {
      const errorMessage = error.message || 'Profile update failed';
      dispatch({
        type: 'REGISTER_FAILURE',
        payload: errorMessage
      });
      return { success: false, error: errorMessage };
    }
  };

  const value = {
    ...state,
    login,
    register,
    logout,
    loginWithTokens,
    updateProfile,
    clearError: () => dispatch({ type: 'CLEAR_ERROR' })
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export { AuthProvider, useAuth };