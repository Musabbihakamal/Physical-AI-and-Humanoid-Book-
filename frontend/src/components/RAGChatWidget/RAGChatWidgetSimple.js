import React, { useState } from 'react';
import styles from './RAGChatWidget.module.css';

// Simplified version for testing
const RAGChatWidgetSimple = () => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleWidget = () => {
    console.log('🔵 RAG Widget Toggle Clicked! Current state:', isOpen);
    alert('RAG Widget button clicked! isOpen: ' + isOpen);
    setIsOpen(!isOpen);
  };

  console.log('🟢 RAGChatWidgetSimple rendering, isOpen:', isOpen);

  return (
    <>
      {/* Floating button */}
      {!isOpen && (
        <button
          className={styles.floatingButton}
          onClick={toggleWidget}
          aria-label="Open chat assistant"
          title="Ask questions about the book"
          style={{
            position: 'fixed',
            bottom: '24px',
            right: '24px',
            zIndex: 9999,
            background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
            color: 'white',
            border: 'none',
            borderRadius: '50%',
            width: '56px',
            height: '56px',
            cursor: 'pointer',
            boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)'
          }}
        >
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
        </button>
      )}

      {/* Chat widget */}
      {isOpen && (
        <div className={styles.chatWidget} style={{
          position: 'fixed',
          bottom: '24px',
          right: '24px',
          width: '380px',
          height: '600px',
          background: 'white',
          borderRadius: '12px',
          boxShadow: '0 8px 32px rgba(0, 0, 0, 0.12)',
          zIndex: 9999
        }}>
          {/* Header */}
          <div className={styles.chatHeader} style={{
            background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
            color: 'white',
            padding: '16px',
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center'
          }}>
            <span>Test Chat Widget</span>
            <button
              onClick={toggleWidget}
              style={{
                background: 'rgba(255, 255, 255, 0.2)',
                border: 'none',
                color: 'white',
                cursor: 'pointer',
                padding: '8px'
              }}
            >
              Close
            </button>
          </div>
          <div style={{ padding: '16px' }}>
            <p>Widget is open! This is a test version.</p>
          </div>
        </div>
      )}
    </>
  );
};

export default RAGChatWidgetSimple;
