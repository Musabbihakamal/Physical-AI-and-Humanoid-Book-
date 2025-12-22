import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import CodeExplainerWidget from '../agent-widgets/code-explainer-widget';

const BookContent = ({ content }) => {
  const [selectedText, setSelectedText] = useState('');
  const [showCodeExplainer, setShowCodeExplainer] = useState(false);
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

  const handleCodeExplainerWidget = () => {
    // Always save the current selected text and show the widget
    if (selectedText) {
      setPersistedCodeContent(selectedText);
      setShowCodeExplainer(true);
    }
  };

  // Function to update widget content without hiding the widget
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
            onClick={handleCodeExplainerWidget}
            className="tool-btn code-btn"
          >
            Explain/Update Code
          </button>
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
          <BrowserOnly>
            {() => <CodeExplainerWidget code={persistedCodeContent} />}
          </BrowserOnly>
        </div>
      )}
    </div>
  );
};

export default BookContent;