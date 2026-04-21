import React, { useEffect, useState } from 'react';
import ReactDOM from 'react-dom';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { AuthProvider } from '../contexts/AuthContext';
import { TranslationProvider, useTranslation } from '../contexts/TranslationContext';

function TranslateButtonPortal({ portalRoot }) {
  const { translatedContent, setTranslatedContent, isTranslated, setIsTranslated, currentLang, setCurrentLang } = useTranslation();
  const [originalContentHTML, setOriginalContentHTML] = useState('');

  useEffect(() => {
    const captureContent = () => {
      const contentElement = document.querySelector('main div.markdown') ||
                            document.querySelector('main article') ||
                            document.querySelector('main');
      if (contentElement) {
        setOriginalContentHTML(contentElement.innerHTML);
      }
    };

    captureContent();
    const timer = setTimeout(captureContent, 1000);
    return () => clearTimeout(timer);
  }, []);

  useEffect(() => {
    const contentElement = document.querySelector('main div.markdown') ||
                          document.querySelector('main article') ||
                          document.querySelector('main');

    if (contentElement && isTranslated && translatedContent) {
      contentElement.innerHTML = translatedContent;
    } else if (contentElement && !isTranslated && originalContentHTML) {
      contentElement.innerHTML = originalContentHTML;
    }
  }, [isTranslated, translatedContent, originalContentHTML]);

  try {
    const TranslateButton = require('../components/TranslateButton').default;

    return ReactDOM.createPortal(
      <div style={{
        position: 'fixed',
        bottom: '20px',
        left: '50%',
        transform: 'translateX(-50%)',
        zIndex: '999',
        pointerEvents: 'auto'
      }}>
        <TranslateButton
          originalContentHTML={originalContentHTML}
          setTranslatedContent={setTranslatedContent}
          setIsTranslated={setIsTranslated}
          currentLang={currentLang}
          setCurrentLang={setCurrentLang}
        />
      </div>,
      portalRoot
    );
  } catch (e) {
    console.error('❌ Failed to load TranslateButton:', e);
    return null;
  }
}

export default function Root({ children }) {
  const [portalRoot, setPortalRoot] = useState(null);

  useEffect(() => {
    let root = document.getElementById('floating-elements-portal');
    if (!root) {
      root = document.createElement('div');
      root.id = 'floating-elements-portal';
      root.style.position = 'fixed';
      root.style.top = '0';
      root.style.left = '0';
      root.style.width = '100%';
      root.style.height = '100%';
      root.style.pointerEvents = 'none';
      root.style.zIndex = '9998';
      document.body.appendChild(root);
    }
    setPortalRoot(root);
  }, []);

  return (
    <BrowserOnly fallback={<div>{children}</div>}>
      {() => (
        <AuthProvider>
          <TranslationProvider>
            {children}
            {/* RAG Chat Widget */}
            {(() => {
              try {
                const RAGChatWidget = require('../components/RAGChatWidget').default;
                return <RAGChatWidget />;
              } catch (e) {
                console.error('Failed to load RAGChatWidget:', e);
                return null;
              }
            })()}
            {/* Translation Button */}
            {portalRoot && <TranslateButtonPortal portalRoot={portalRoot} />}
          </TranslationProvider>
        </AuthProvider>
      )}
    </BrowserOnly>
  );
}