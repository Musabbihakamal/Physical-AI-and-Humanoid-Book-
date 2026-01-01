import React, { useState } from 'react';
import { getApiService } from '../../services/api';
import './code-explainer-widget.css';

const CodeExplainerWidget = ({ code }) => {
  const [explanation, setExplanation] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  const explainCode = async () => {
    if (!code) {
      setError('No code provided for explanation');
      if (typeof window !== 'undefined') {
        const { trackError } = await import('../../services/analytics');
        trackError(new Error('No code provided for explanation'), {
          component: 'CodeExplainerWidget',
          action: 'explainCode'
        });
      }
      return;
    }

    setLoading(true);
    setError(null);

    try {
      // Track the code explanation request
      if (typeof window !== 'undefined') {
        const { trackCodeExplanation } = await import('../../services/analytics');
        trackCodeExplanation({
          code_length: code.length,
          trigger: 'widget_button_click'
        });
      }

      const apiService = getApiService(); // Get the API service instance
      const response = await apiService.explainCode(code);

      // In a real implementation, we would get the explanation data from the response
      // For now, we'll simulate polling for the result
      const requestId = response.request_id;

      // Simulate polling for the result
      const pollForResult = async () => {
        try {
          const apiService = getApiService(); // Get the API service instance
          const statusResponse = await apiService.getRequestStatus(requestId);
          if (statusResponse.status === 'COMPLETED') {
            // In a real implementation, we would fetch the actual generated content
            // For now, we'll simulate a response
            const mockExplanation = {
              overview: "This code demonstrates basic functionality",
              key_components: [
                { type: "function", name: "exampleFunction", declaration: "def exampleFunction():" }
              ],
              ros2_specifics: ["rclpy.Node", "create_publisher"],
              isaac_sim_specifics: ["omni.isaac.core", "World()"],
              technical_concepts: ["Object-oriented programming", "Asynchronous execution"],
              usage_context: "This code is typically used for initialization",
              learning_points: ["Understand the basic structure", "Learn about ROS 2 patterns"]
            };
            setExplanation(mockExplanation);
            setLoading(false);

            // Track successful completion
            if (typeof window !== 'undefined') {
              const { trackCodeExplanation } = await import('../../services/analytics');
              trackCodeExplanation({
                status: 'success',
                request_id: requestId,
                components_count: mockExplanation.key_components.length
              });
            }
          } else if (statusResponse.status === 'FAILED') {
            setError('Code explanation failed');
            setLoading(false);

            // Track failure
            if (typeof window !== 'undefined') {
              const { trackCodeExplanation } = await import('../../services/analytics');
              trackCodeExplanation({
                status: 'failed',
                request_id: requestId
              });
            }
          } else {
            // Still processing, check again in 2 seconds
            setTimeout(pollForResult, 2000);
          }
        } catch (err) {
          setError('Failed to check code explanation status');
          setLoading(false);

          // Track error
          if (typeof window !== 'undefined') {
            const { trackError } = await import('../../services/analytics');
            trackError(err, {
              component: 'CodeExplainerWidget',
              action: 'pollForResult',
              request_id: requestId
            });
          }
        }
      };

      // Start polling for the result
      setTimeout(pollForResult, 1000);
    } catch (err) {
      setError(err.message || 'Failed to explain code');
      setLoading(false);

      // Track error
      if (typeof window !== 'undefined') {
        const { trackError } = await import('../../services/analytics');
        trackError(err, {
          component: 'CodeExplainerWidget',
          action: 'explainCode'
        });
      }
    }
  };

  return (
    <div className="code-explainer-widget">
      <h3>Code Explainer</h3>
      <button
        onClick={explainCode}
        disabled={loading}
        className="explain-btn"
      >
        {loading ? 'Explaining...' : 'Explain Code'}
      </button>

      {error && <div className="error">{error}</div>}

      {explanation && (
        <div className="explanation-results">
          <h4>Explanation</h4>
          <div className="explanation-section">
            <h5>Overview</h5>
            <p>{explanation.overview}</p>
          </div>

          {explanation.key_components && explanation.key_components.length > 0 && (
            <div className="explanation-section">
              <h5>Key Components</h5>
              <ul>
                {explanation.key_components.map((component, index) => (
                  <li key={index}>
                    <strong>{component.type}:</strong> {component.name} - {component.declaration}
                  </li>
                ))}
              </ul>
            </div>
          )}

          {explanation.ros2_specifics && explanation.ros2_specifics.length > 0 && (
            <div className="explanation-section">
              <h5>ROS 2 Specifics</h5>
              <ul>
                {explanation.ros2_specifics.map((cmd, index) => (
                  <li key={index}>{cmd}</li>
                ))}
              </ul>
            </div>
          )}

          {explanation.isaac_sim_specifics && explanation.isaac_sim_specifics.length > 0 && (
            <div className="explanation-section">
              <h5>Isaac Sim Specifics</h5>
              <ul>
                {explanation.isaac_sim_specifics.map((cmd, index) => (
                  <li key={index}>{cmd}</li>
                ))}
              </ul>
            </div>
          )}

          {explanation.technical_concepts && explanation.technical_concepts.length > 0 && (
            <div className="explanation-section">
              <h5>Technical Concepts</h5>
              <ul>
                {explanation.technical_concepts.map((concept, index) => (
                  <li key={index}>{concept}</li>
                ))}
              </ul>
            </div>
          )}

          <div className="explanation-section">
            <h5>Learning Points</h5>
            <ul>
              {explanation.learning_points.map((point, index) => (
                <li key={index}>{point}</li>
              ))}
            </ul>
          </div>
        </div>
      )}
    </div>
  );
};

export default CodeExplainerWidget;