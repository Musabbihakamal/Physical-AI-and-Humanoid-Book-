import React, { useState } from 'react';
import ApiService from '../../services/api';
import { trackGlossaryGeneration, trackError } from '../../services/analytics';
import './glossary-widget.css';

const GlossaryWidget = ({ content }) => {
  const [glossary, setGlossary] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  const generateGlossary = async () => {
    if (!content) {
      setError('No content provided for glossary generation');
      trackError(new Error('No content provided for glossary generation'), {
        component: 'GlossaryWidget',
        action: 'generateGlossary'
      });
      return;
    }

    setLoading(true);
    setError(null);

    try {
      // Track the glossary generation request
      trackGlossaryGeneration({
        content_length: content.length,
        trigger: 'widget_button_click'
      });

      const response = await ApiService.createGlossary(content);

      // In a real implementation, we would get the glossary data from the response
      // For now, we'll simulate polling for the result
      const requestId = response.request_id;

      // Simulate polling for the result
      const pollForResult = async () => {
        try {
          const statusResponse = await ApiService.getRequestStatus(requestId);
          if (statusResponse.status === 'COMPLETED') {
            // In a real implementation, we would fetch the actual generated content
            // For now, we'll simulate a response
            const mockGlossary = {
              title: "Generated Glossary",
              entries: [
                { term: "Example Term 1", definition: "Definition for example term 1" },
                { term: "Example Term 2", definition: "Definition for example term 2" },
                { term: "Example Term 3", definition: "Definition for example term 3" }
              ]
            };
            setGlossary(mockGlossary);
            setLoading(false);

            // Track successful completion
            trackGlossaryGeneration({
              status: 'success',
              request_id: requestId,
              entries_count: mockGlossary.entries.length
            });
          } else if (statusResponse.status === 'FAILED') {
            setError('Glossary generation failed');
            setLoading(false);

            // Track failure
            trackGlossaryGeneration({
              status: 'failed',
              request_id: requestId
            });
          } else {
            // Still processing, check again in 2 seconds
            setTimeout(pollForResult, 2000);
          }
        } catch (err) {
          setError('Failed to check glossary generation status');
          setLoading(false);

          // Track error
          trackError(err, {
            component: 'GlossaryWidget',
            action: 'pollForResult',
            request_id: requestId
          });
        }
      };

      // Start polling for the result
      setTimeout(pollForResult, 1000);
    } catch (err) {
      setError(err.message || 'Failed to generate glossary');
      setLoading(false);

      // Track error
      trackError(err, {
        component: 'GlossaryWidget',
        action: 'generateGlossary'
      });
    }
  };

  return (
    <div className="glossary-widget">
      <h3>Glossary Maker</h3>
      <button
        onClick={generateGlossary}
        disabled={loading}
        className="generate-btn"
      >
        {loading ? 'Generating...' : 'Generate Glossary'}
      </button>

      {error && <div className="error">{error}</div>}

      {glossary && (
        <div className="glossary-results">
          <h4>{glossary.title}</h4>
          <div className="glossary-entries">
            {glossary.entries.map((entry, index) => (
              <div key={index} className="glossary-entry">
                <h5>{entry.term}</h5>
                <p>{entry.definition}</p>
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
};

export default GlossaryWidget;