import React, { useState, useEffect, useRef } from 'react';
import { HtmlClassNameProvider } from '@docusaurus/theme-common';
import { DocProvider } from '@docusaurus/plugin-content-docs/client';
import OriginalDocItem from '@theme-original/DocItem';
import DocItemPaginator from '@theme/DocItem/Paginator';
import { TranslationProvider, useTranslation } from '@site/src/contexts/TranslationContext';
import TranslateButtonWrapper from '@site/src/components/TranslateButton/TranslateButtonWrapper';
import BookContent from '@site/src/components/book-content/book-content';

// Separate component to handle content rendering and translation state
function DocItemContent(props) {
  const [originalContentHTML, setOriginalContentHTML] = useState(null);
  const [contentLoaded, setContentLoaded] = useState(false);
  const [showCustomContent, setShowCustomContent] = useState(false);
  const contentRef = useRef(null);

  const { isTranslated, translatedContent, setTranslatedContent, setIsTranslated, currentLang, setCurrentLang } = useTranslation();

  const docHtmlClassName = `docs-doc-id-${props.content.metadata.id}`;

  // Extract content from the DOM after OriginalDocItem renders
  useEffect(() => {
    let observer;

    // Function to try to get content with multiple fallback strategies
    const getMarkdownContent = () => {
      // Try multiple selectors in order of preference
      const selectors = [
        'main div.markdown',           // Docusaurus main content area
        'main article div.markdown',   // Docusaurus article content
        'article div.markdown',        // Alternative article content
        '.markdown',                   // Generic markdown class
        'main article',                // Main article container
        '.theme-doc-markdown',         // Docusaurus theme markdown
        'main .container',             // Main container
        'main'                         // Fallback to main
      ];

      for (const selector of selectors) {
        const element = document.querySelector(selector);
        if (element && element.innerHTML && element.innerHTML.trim()) {
          return element.innerHTML; // Return the HTML content
        }
      }

      return null;
    };

    // Try to get content immediately
    const tryExtractContent = () => {
      const contentHTML = getMarkdownContent();
      if (contentHTML && !originalContentHTML) {
        setOriginalContentHTML(contentHTML);
        setContentLoaded(true);
        setShowCustomContent(true);
        if (observer) {
          observer.disconnect();
        }
        return true;
      }
      return false;
    };

    // Try immediately first
    if (!tryExtractContent()) {
      // If not found, set up a MutationObserver to detect when content is added
      if (typeof window !== 'undefined' && window.MutationObserver) {
        observer = new MutationObserver((mutations) => {
          let found = false;
          mutations.forEach((mutation) => {
            if (mutation.type === 'childList' || mutation.type === 'subtree') {
              if (tryExtractContent()) {
                found = true;
              }
            }
          });
          // If content was found, stop observing
          if (found && observer) {
            observer.disconnect();
          }
        });

        // Start observing the document body for changes
        observer.observe(document.body, {
          childList: true,
          subtree: true,
          attributes: false,
          characterData: false
        });

        // Also try periodically as a backup
        const interval = setInterval(() => {
          if (tryExtractContent()) {
            clearInterval(interval);
            if (observer) {
              observer.disconnect();
            }
          }
        }, 200);

        // Clean up interval after a reasonable time
        setTimeout(() => {
          clearInterval(interval);
          if (observer && !originalContentHTML) {
            observer.disconnect();
          }
        }, 10000); // Increase timeout to 10 seconds to allow more time for content to load
      }
    } else {
      console.log('Content extracted immediately');
    }

    // Clean up observer on unmount
    return () => {
      if (observer) {
        observer.disconnect();
      }
    };
  }, [originalContentHTML, setOriginalContentHTML, setContentLoaded, setShowCustomContent]);

  return (
    <div className="doc-item-container" ref={contentRef}>
      {/* Original DocItem - needed for content to be rendered initially */}
      <div style={{ display: showCustomContent ? 'none' : 'block' }}>
        <OriginalDocItem {...props} />
      </div>

      {/* Custom content rendering - shown when we have content */}
      {showCustomContent && (
        <>
          {/* Translated content with BookContent widgets - prioritize translated content when available */}
          {isTranslated && translatedContent ? (
            <BookContent content={translatedContent} />
          ) : originalContentHTML ? (
            <BookContent content={originalContentHTML} />
          ) : (
            // Fallback to original content if no original content extracted yet
            <OriginalDocItem {...props} />
          )}
        </>
      )}

      {/* Pass the extracted original content HTML to the translator */}
      <TranslateButtonWrapper
        originalContentHTML={originalContentHTML}
      />

      {/* Chapter Navigation - Previous/Next buttons below translator button */}
      <DocItemPaginator />
    </div>
  );
}

// Main DocItem component with translation provider
export default function DocItem(props) {
  const docHtmlClassName = `docs-doc-id-${props.content.metadata.id}`;

  return (
    <DocProvider content={props.content}>
      <HtmlClassNameProvider className={docHtmlClassName}>
        <TranslationProvider>
          <DocItemContent {...props} />
        </TranslationProvider>
      </HtmlClassNameProvider>
    </DocProvider>
  );
}