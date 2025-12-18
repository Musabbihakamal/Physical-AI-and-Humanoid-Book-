import React, { useState } from 'react';
import GlossaryWidget from '../agent-widgets/glossary-widget';
import CodeExplainerWidget from '../agent-widgets/code-explainer-widget';
import QuizWidget from '../agent-widgets/quiz-widget';
import ChapterGeneratorWidget from '../agent-widgets/chapter-generator-widget';

const BookContent = ({ content }) => {
  const [selectedText, setSelectedText] = useState('');
  const [showGlossaryWidget, setShowGlossaryWidget] = useState(false);
  const [showCodeExplainer, setShowCodeExplainer] = useState(false);

  const handleTextSelection = () => {
    const selection = window.getSelection();
    if (selection.toString().trim()) {
      setSelectedText(selection.toString());
    }
  };

  return (
    <div className="book-content" onMouseUp={handleTextSelection}>
      <div className="content-display" dangerouslySetInnerHTML={{ __html: content }} />

      {selectedText && (
        <div className="text-selection-tools">
          <button
            onClick={() => setShowGlossaryWidget(!showGlossaryWidget)}
            className="tool-btn glossary-btn"
          >
            Create Glossary
          </button>
          <button
            onClick={() => setShowCodeExplainer(!showCodeExplainer)}
            className="tool-btn code-btn"
          >
            Explain Code
          </button>
        </div>
      )}

      {showGlossaryWidget && (
        <div className="widget-container">
          <GlossaryWidget content={selectedText} />
        </div>
      )}

      {showCodeExplainer && (
        <div className="widget-container">
          <CodeExplainerWidget code={selectedText} />
        </div>
      )}

      <div className="standalone-widgets">
        <QuizWidget />
        <ChapterGeneratorWidget />
      </div>
    </div>
  );
};

export default BookContent;