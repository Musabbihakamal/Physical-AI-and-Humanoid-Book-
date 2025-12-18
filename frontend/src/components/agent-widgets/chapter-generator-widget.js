import React, { useState } from 'react';
import ApiService from '../../services/api';
import { trackChapterGeneration, trackError } from '../../services/analytics';
import './chapter-generator-widget.css';

const ChapterGeneratorWidget = () => {
  const [moduleFocus, setModuleFocus] = useState('');
  const [outline, setOutline] = useState('');
  const [chapter, setChapter] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  const generateChapter = async () => {
    if (!moduleFocus.trim()) {
      setError('Module focus is required');
      trackError(new Error('Module focus is required'), {
        component: 'ChapterGeneratorWidget',
        action: 'generateChapter'
      });
      return;
    }

    setLoading(true);
    setError(null);

    try {
      // Track the chapter generation request
      trackChapterGeneration({
        module_focus: moduleFocus,
        outline_length: outline ? outline.length : 0,
        trigger: 'widget_button_click'
      });

      // Parse outline if provided
      const outlineArray = outline ? outline.split('\n').filter(line => line.trim()) : null;

      const response = await ApiService.generateChapter(moduleFocus, outlineArray);

      // In a real implementation, we would get the chapter data from the response
      // For now, we'll simulate polling for the result
      const requestId = response.request_id;

      // Simulate polling for the result
      const pollForResult = async () => {
        try {
          const statusResponse = await ApiService.getRequestStatus(requestId);
          if (statusResponse.status === 'COMPLETED') {
            // In a real implementation, we would fetch the actual generated content
            // For now, we'll simulate a response
            const mockChapter = {
              id: "mock-chapter-id",
              title: `Chapter: ${moduleFocus}`,
              content: `# Chapter: ${moduleFocus}\n\n## Learning Objectives\n\n- Understand the fundamentals of ${moduleFocus}\n- Apply ${moduleFocus} concepts in practical scenarios\n\n## Introduction\n\nThis chapter covers ${moduleFocus}, a fundamental concept in robotics and AI.\n\n## Key Concepts\n\nThe key concepts of ${moduleFocus} include various important principles...\n\n## Summary\n\nIn this chapter, we've covered the essential aspects of ${moduleFocus}.`,
              module_focus: moduleFocus,
              difficulty_level: "INTERMEDIATE",
              word_count: 245
            };
            setChapter(mockChapter);
            setLoading(false);

            // Track successful completion
            trackChapterGeneration({
              status: 'success',
              request_id: requestId,
              word_count: mockChapter.word_count
            });
          } else if (statusResponse.status === 'FAILED') {
            setError('Chapter generation failed');
            setLoading(false);

            // Track failure
            trackChapterGeneration({
              status: 'failed',
              request_id: requestId
            });
          } else {
            // Still processing, check again in 2 seconds
            setTimeout(pollForResult, 2000);
          }
        } catch (err) {
          setError('Failed to check chapter generation status');
          setLoading(false);

          // Track error
          trackError(err, {
            component: 'ChapterGeneratorWidget',
            action: 'pollForResult',
            request_id: requestId
          });
        }
      };

      // Start polling for the result
      setTimeout(pollForResult, 1000);
    } catch (err) {
      setError(err.message || 'Failed to generate chapter');
      setLoading(false);

      // Track error
      trackError(err, {
        component: 'ChapterGeneratorWidget',
        action: 'generateChapter'
      });
    }
  };

  return (
    <div className="chapter-generator-widget">
      <h3>Chapter Generator</h3>

      <div className="input-group">
        <label htmlFor="module-focus">Module Focus:</label>
        <input
          type="text"
          id="module-focus"
          value={moduleFocus}
          onChange={(e) => setModuleFocus(e.target.value)}
          placeholder="Enter the main topic or focus of the chapter"
          className="input-field"
        />
      </div>

      <div className="input-group">
        <label htmlFor="outline">Outline (optional, one item per line):</label>
        <textarea
          id="outline"
          value={outline}
          onChange={(e) => setOutline(e.target.value)}
          placeholder="Introduction\nKey Concepts\nImplementation\nBest Practices\nSummary"
          rows="4"
          className="input-field"
        />
      </div>

      <button
        onClick={generateChapter}
        disabled={loading}
        className="generate-btn"
      >
        {loading ? 'Generating...' : 'Generate Chapter'}
      </button>

      {error && <div className="error">{error}</div>}

      {chapter && (
        <div className="chapter-results">
          <h4>{chapter.title}</h4>
          <div className="chapter-meta">
            <p><strong>Module Focus:</strong> {chapter.module_focus}</p>
            <p><strong>Difficulty:</strong> {chapter.difficulty_level}</p>
            <p><strong>Word Count:</strong> {chapter.word_count}</p>
          </div>
          <div className="chapter-content">
            <h5>Content Preview:</h5>
            <pre className="content-preview">
              {chapter.content.substring(0, 500)}...
            </pre>
            <button className="view-full-btn">
              View Full Chapter
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChapterGeneratorWidget;