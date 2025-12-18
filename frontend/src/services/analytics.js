// frontend/src/services/analytics.js
import { getEnvVar, isDevelopment } from '../utils/env';

class AnalyticsService {
  constructor() {
    this.isEnabled = getEnvVar('REACT_APP_ANALYTICS_ENABLED', 'false') === 'true';
    this.apiKey = getEnvVar('REACT_APP_ANALYTICS_API_KEY', '');
    this.endpoint = getEnvVar('REACT_APP_ANALYTICS_ENDPOINT', '/api/analytics');
    this.userId = this.generateUserId();
  }

  // Generate a unique user ID for tracking
  generateUserId() {
    // Try to get from localStorage, otherwise generate a new one
    let userId = localStorage.getItem('userId');
    if (!userId) {
      userId = 'user_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
      localStorage.setItem('userId', userId);
    }
    return userId;
  }

  // Track an event
  trackEvent(eventName, properties = {}) {
    if (!this.isEnabled) {
      return;
    }

    try {
      const event = {
        userId: this.userId,
        eventName,
        properties: {
          ...properties,
          timestamp: new Date().toISOString(),
          userAgent: navigator.userAgent,
          url: window.location.href,
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

  // Track chapter generation
  trackChapterGeneration(params = {}) {
    this.trackSubagentUsage('CHAPTER_GENERATOR', {
      action: 'generate',
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

// Create and export a global analytics instance
const analytics = new AnalyticsService();
export default analytics;

// Export individual tracking functions
export const trackSubagentUsage = analytics.trackSubagentUsage.bind(analytics);
export const trackGlossaryGeneration = analytics.trackGlossaryGeneration.bind(analytics);
export const trackCodeExplanation = analytics.trackCodeExplanation.bind(analytics);
export const trackQuizCreation = analytics.trackQuizCreation.bind(analytics);
export const trackChapterGeneration = analytics.trackChapterGeneration.bind(analytics);
export const trackUserInteraction = analytics.trackUserInteraction.bind(analytics);
export const trackError = analytics.trackError.bind(analytics);