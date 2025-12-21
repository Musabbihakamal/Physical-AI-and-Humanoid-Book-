// frontend/src/services/analytics.js
import { getEnvVar, isDevelopment } from '../utils/env';

class AnalyticsService {
  constructor() {
    this.isEnabled = getEnvVar('REACT_APP_ANALYTICS_ENABLED', 'false') === 'true';
    this.apiKey = getEnvVar('REACT_APP_ANALYTICS_API_KEY', '');
    this.endpoint = getEnvVar('REACT_APP_ANALYTICS_ENDPOINT', '/api/analytics');
    // Don't generate userId in constructor - it will be generated when needed
    this.userId = null;
  }

  // Generate a unique user ID for tracking
  generateUserId() {
    // Try to get from localStorage, otherwise generate a new one
    let userId = null;
    if (typeof window !== 'undefined') {
      try {
        userId = localStorage.getItem('userId');
        if (!userId) {
          userId = 'user_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
          localStorage.setItem('userId', userId);
        }
      } catch (error) {
        // Handle localStorage errors (e.g., when it's not available in SSR)
        userId = 'user_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
      }
    } else {
      // For server-side rendering, generate a temporary ID
      userId = 'ssr_user_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
    }
    return userId;
  }

  // Get user ID, generating it if needed
  getUserId() {
    if (!this.userId) {
      this.userId = this.generateUserId();
    }
    return this.userId;
  }

  // Track an event
  trackEvent(eventName, properties = {}) {
    if (!this.isEnabled) {
      return;
    }

    try {
      const event = {
        userId: this.getUserId(),
        eventName,
        properties: {
          ...properties,
          timestamp: new Date().toISOString(),
          userAgent: typeof window !== 'undefined' ? navigator.userAgent : 'SSR',
          url: typeof window !== 'undefined' ? window.location.href : 'SSR',
        },
        timestamp: new Date().toISOString(),
      };

      // Send event to analytics endpoint
      this.sendEvent(event);
    } catch (error) {
      console.warn('Analytics tracking error:', error);
    }
  }

  // Send event to analytics endpoint
  async sendEvent(event) {
    // In a real implementation, this would send to an analytics backend
    // For now, we'll just log to console if in development
    if (isDevelopment()) {
      console.log('Analytics event:', event);
    }

    // In production, you would send to your analytics service
    // Example:
    /*
    try {
      await fetch(this.endpoint, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.apiKey}`,
        },
        body: JSON.stringify(event),
      });
    } catch (error) {
      console.warn('Failed to send analytics event:', error);
    }
    */
  }

  // Track subagent usage
  trackSubagentUsage(agentType, params = {}) {
    this.trackEvent('subagent_usage', {
      agent_type: agentType,
      ...params,
    });
  }

  // Track glossary generation
  trackGlossaryGeneration(params = {}) {
    this.trackSubagentUsage('GLOSSARY_MAKER', {
      action: 'generate',
      ...params,
    });
  }

  // Track code explanation
  trackCodeExplanation(params = {}) {
    this.trackSubagentUsage('CODE_EXPLAINER', {
      action: 'explain',
      ...params,
    });
  }

  // Track quiz creation
  trackQuizCreation(params = {}) {
    this.trackSubagentUsage('QUIZ_CREATOR', {
      action: 'create',
      ...params,
    });
  }


  // Track user interactions
  trackUserInteraction(interactionType, params = {}) {
    this.trackEvent('user_interaction', {
      interaction_type: interactionType,
      ...params,
    });
  }

  // Track errors
  trackError(error, context = {}) {
    this.trackEvent('error', {
      error_message: error.message || error,
      error_stack: error.stack,
      ...context,
    });
  }

  // Get analytics configuration
  getConfig() {
    return {
      isEnabled: this.isEnabled,
      userId: this.userId,
    };
  }

  // Set user properties
  setUserProperties(properties) {
    this.trackEvent('user_properties', properties);
  }
}

// Create analytics instance lazily and only in browser to avoid SSR issues
let analyticsInstance = null;

const getAnalytics = () => {
  // Only create instance in browser environment
  if (typeof window !== 'undefined' && !analyticsInstance) {
    analyticsInstance = new AnalyticsService();
  }
  return analyticsInstance;
};

// Export individual tracking functions that only work in browser
export const trackSubagentUsage = (...args) => {
  const analytics = getAnalytics();
  return analytics ? analytics.trackSubagentUsage(...args) : null;
};

export const trackGlossaryGeneration = (...args) => {
  const analytics = getAnalytics();
  return analytics ? analytics.trackGlossaryGeneration(...args) : null;
};

export const trackCodeExplanation = (...args) => {
  const analytics = getAnalytics();
  return analytics ? analytics.trackCodeExplanation(...args) : null;
};

export const trackQuizCreation = (...args) => {
  const analytics = getAnalytics();
  return analytics ? analytics.trackQuizCreation(...args) : null;
};

export const trackUserInteraction = (...args) => {
  const analytics = getAnalytics();
  return analytics ? analytics.trackUserInteraction(...args) : null;
};

export const trackError = (...args) => {
  const analytics = getAnalytics();
  return analytics ? analytics.trackError(...args) : null;
};

// Also export the analytics service for direct use
export default getAnalytics;