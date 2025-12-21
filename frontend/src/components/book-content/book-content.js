import React, { useState, useEffect } from 'react';
import GlossaryWidget from '../agent-widgets/glossary-widget';
import CodeExplainerWidget from '../agent-widgets/code-explainer-widget';
import QuizWidget from '../agent-widgets/quiz-widget';

const BookContent = ({ content }) => {
  const [selectedText, setSelectedText] = useState('');
  const [showGlossaryWidget, setShowGlossaryWidget] = useState(false);
  const [showCodeExplainer, setShowCodeExplainer] = useState(false);
  const [persistedGlossaryContent, setPersistedGlossaryContent] = useState('');
  const [persistedCodeContent, setPersistedCodeContent] = useState('');
  const [isClient, setIsClient] = useState(false);

  // Check if running on client side
  useEffect(() => {
    setIsClient(true);
  }, []);

  const handleTextSelection = () => {
    if (typeof window !== 'undefined' && window.getSelection) {
      const selection = window.getSelection();
      if (selection.toString().trim()) {
        setSelectedText(selection.toString());
      }
    }
  };

  const handleGlossaryWidget = () => {
    // Always save the current selected text and show the widget
    if (selectedText) {
      setPersistedGlossaryContent(selectedText);
      setShowGlossaryWidget(true);
    }
  };

  const handleCodeExplainerWidget = () => {
    // Always save the current selected text and show the widget
    if (selectedText) {
      setPersistedCodeContent(selectedText);
      setShowCodeExplainer(true);
    }
  };

  // Function to update widget content without hiding the widget
  const updateGlossaryContent = () => {
    if (selectedText) {
      setPersistedGlossaryContent(selectedText);
    }
  };

  const updateCodeContent = () => {
    if (selectedText) {
      setPersistedCodeContent(selectedText);
    }
  };

  return (
    <div className="book-content" onMouseUp={isClient ? handleTextSelection : undefined}>
      <div className="content-display" dangerouslySetInnerHTML={{ __html: content }} />

      {isClient && selectedText && (
        <div className="text-selection-tools">
          <button
            onClick={handleGlossaryWidget}
            className="tool-btn glossary-btn"
          >
            Create/Update Glossary
          </button>
          <button
            onClick={handleCodeExplainerWidget}
            className="tool-btn code-btn"
          >
            Explain/Update Code
          </button>
        </div>
      )}

      {isClient && showGlossaryWidget && (
        <div className="widget-container">
          <div className="widget-header">
            <h3>Glossary Widget</h3>
            <button
              onClick={() => setShowGlossaryWidget(false)}
              className="close-widget-btn"
            >
              ×
            </button>
          </div>
          <GlossaryWidget content={persistedGlossaryContent} />
        </div>
      )}

      {isClient && showCodeExplainer && (
        <div className="widget-container">
          <div className="widget-header">
            <h3>Code Explainer Widget</h3>
            <button
              onClick={() => setShowCodeExplainer(false)}
              className="close-widget-btn"
            >
              ×
            </button>
          </div>
          <CodeExplainerWidget code={persistedCodeContent} />
        </div>
      )}

      <div className="standalone-widgets">
        <QuizWidget />
      </div>
    </div>
  );
};

export default BookContent;