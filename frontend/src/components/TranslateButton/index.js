import React, { useState, useEffect, useRef } from 'react';
import styles from './TranslateButton.module.css';

const TranslateButton = ({ originalContentHTML, setTranslatedContent, setIsTranslated, currentLang: propCurrentLang, setCurrentLang: propSetCurrentLang }) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [currentLang, setCurrentLang] = useState(propCurrentLang || 'en'); // 'en' for English, 'ur' for Urdu
  const [translationError, setTranslationError] = useState(null);
  const [translatorReady, setTranslatorReady] = useState(true); // Ready by default since using LLM-based translation
  const [isClient, setIsClient] = useState(false); // Track if we're in the browser
  const contentRef = useRef(null);

  // Initialize translator readiness - no longer dependent on Google Translate API
  useEffect(() => {
    // Check if we're in the browser environment before setting client flag
    if (typeof window !== 'undefined' && typeof document !== 'undefined') {
      // Set client flag to true in browser
      setIsClient(true);

      // Set translator as ready immediately since we're using LLM-based translation
      setTranslatorReady(true);
    }
  }, []);

  // Function to extract and preserve code blocks, diagrams, images, and technical terms
  const extractAndPreserveElements = (content) => {
    const preservedElements = [];
    let processedContent = content;

    // Extract and preserve various content types in order of complexity
    const patterns = [
      // 1. Full code blocks with language specification (```lang...```)
      { regex: /(```[\s\S]*?```)/g, type: 'fenced_code_block' },
      // 2. Images and diagrams
      { regex: /(<img[^>]*>)/gi, type: 'image_tag' },
      { regex: /(!\[[^\]]*\]\([^)]*\))/g, type: 'markdown_image' },
      // 3. Inline code
      { regex: /(`[^`]*`)/g, type: 'inline_code' },
      // 4. Code tags
      { regex: /(<code[^>]*>[\s\S]*?<\/code>)/gi, type: 'html_code_tag' },
      // 5. Preformatted text blocks
      { regex: /(<pre[^>]*>[\s\S]*?<\/pre>)/gi, type: 'html_pre_tag' },
      // 6. Tables
      { regex: /(\|[^\n]*\|(?:\n\|[-:|-\s]*)?\n(?:\|[^\n]*\|[\s\n]*)*)/g, type: 'markdown_table' },
      // 7. HTML divs that might contain diagrams or special content
      { regex: /(<div[^>]*class="[^"]*diagram[^"]*"[^>]*>[\s\S]*?<\/div>)/gi, type: 'diagram_div' },
      { regex: /(<div[^>]*class="[^"]*mermaid[^"]*"[^>]*>[\s\S]*?<\/div>)/gi, type: 'mermaid_diagram' },
      { regex: /(<div[^>]*>[\s\S]*?<\/div>)/gi, type: 'html_div' }, // More general divs
      // 8. Other HTML elements that should be preserved
      { regex: /(<figure[^>]*>[\s\S]*?<\/figure>)/gi, type: 'html_figure' },
      { regex: /(<svg[^>]*>[\s\S]*?<\/svg>)/gi, type: 'svg_element' },
      { regex: /(<canvas[^>]*>[\s\S]*?<\/canvas>)/gi, type: 'canvas_element' }
    ];

    let index = 0;

    // Process each pattern
    for (const pattern of patterns) {
      let match;
      // Use a temporary content to avoid replacement conflicts
      let tempContent = processedContent;
      while ((match = pattern.regex.exec(tempContent)) !== null) {
        const placeholder = `__PRESERVED_${pattern.type.toUpperCase()}_${index}__`;
        const original = match[0];

        preservedElements.push({
          placeholder,
          original,
          type: pattern.type
        });

        // Replace the match with placeholder using a global regex
        const escapedOriginal = original.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
        const placeholderRegex = new RegExp(escapedOriginal, 'g');
        tempContent = tempContent.replace(placeholderRegex, placeholder);

        index++;
        // Reset regex index to continue searching from the beginning
        pattern.regex.lastIndex = 0;
      }
      processedContent = tempContent;
    }

    // Extract and preserve technical terms (ROS 2, Gazebo, Isaac, URDF, etc.)
    const technicalTerms = ['ROS 2', 'Gazebo', 'Isaac', 'URDF', 'ROS2', 'isaac', 'gazebo', 'urdf', 'Python', 'C++', 'API', 'LLM', 'AI', 'JSON', 'XML', 'HTML', 'CSS', 'JavaScript', 'TypeScript', 'Node.js', 'React', 'Docusaurus'];
    for (const term of technicalTerms) {
      // Create a case-insensitive regex for the term
      const termRegex = new RegExp(`\\b${term.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}\\b`, 'gi');
      let termMatch;
      let termIndex = 0;

      // Use a temporary content for technical terms as well
      let tempContent = processedContent;
      while ((termMatch = termRegex.exec(tempContent)) !== null) {
        const placeholder = `__TECH_TERM_${index}_${termIndex}__`;
        const original = termMatch[0];

        preservedElements.push({
          placeholder,
          original,
          type: 'tech_term',
          originalTerm: term
        });

        // Replace with placeholder
        const escapedOriginal = original.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
        const placeholderRegex = new RegExp(escapedOriginal, 'g');
        tempContent = tempContent.replace(placeholderRegex, placeholder);

        termIndex++;
      }
      processedContent = tempContent;
      index++;
    }

    return { processedContent, preservedElements };
  };

  // Function to restore preserved elements back to content
  const restorePreservedElements = (translatedContent, preservedElements) => {
    let result = translatedContent;

    // Restore in order (not reverse) to handle nested elements properly
    for (let i = 0; i < preservedElements.length; i++) {
      const { placeholder, original } = preservedElements[i];
      // Use a more robust replacement that handles special regex characters properly
      const escapedPlaceholder = placeholder.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
      const regex = new RegExp(escapedPlaceholder, 'g');
      result = result.replace(regex, original);
    }

    return result;
  };

  // Function to translate content using Google Translate API
  const translateText = async (text, targetLang) => {
    // Check if we're in the browser environment
    if (typeof window === 'undefined' || !isClient) {
      throw new Error('Translation can only be performed in the browser.');
    }

    if (!window.google || !window.google.translate) {
      throw new Error('Google Translate API is not loaded. Please refresh the page.');
    }

    if (!translatorReady) {
      throw new Error('Translation service is not ready. Please refresh the page.');
    }

    // Validate input
    if (!text || typeof text !== 'string') {
      throw new Error('Invalid content provided for translation.');
    }

    setIsTranslating(true);
    setTranslationError(null);

    try {
      // Extract and preserve code blocks and technical terms
      const { processedContent, preservedElements } = extractAndPreserveElements(text);

      // Validate that we have content to translate after processing
      if (!processedContent || processedContent.trim().length === 0) {
        throw new Error('No translatable content found after processing.');
      }

      return new Promise((resolve, reject) => {
        try {
          // Use Google Translate API directly
          const translateElement = document.createElement('div');
          translateElement.style.display = 'none';
          translateElement.innerHTML = processedContent;
          document.body.appendChild(translateElement);

          // Create a hidden Google Translate element to perform the translation
          const googleTranslateElement = document.createElement('div');
          googleTranslateElement.id = 'google_translate_element';
          googleTranslateElement.style.display = 'none';
          document.body.appendChild(googleTranslateElement);

          // Initialize Google Translate with the hidden element
          new google.translate.TranslateElement({
            pageLanguage: 'en',
            includedLanguages: targetLang,
            layout: google.translate.TranslateElement.InlineLayout.SIMPLE,
            autoDisplay: false
          }, 'google_translate_element');

          // Wait a bit for initialization
          setTimeout(() => {
            // Perform translation by temporarily changing the page language
            const originalLang = document.documentElement.lang || 'en';

            // Use Google Translate API to translate the text
            const translateService = google.translate.TranslateService;
            if (translateService) {
              translateService.translateText(
                processedContent,
                'en', // source language
                targetLang, // target language
                (result) => {
                  if (result && result.sentences && result.sentences.length > 0) {
                    let translatedText = result.sentences.map(s => s.trans).join(' ');

                    // Restore preserved elements back to the translated content
                    const finalContent = restorePreservedElements(translatedText, preservedElements);

                    // Validate the final content
                    if (!finalContent || finalContent.trim().length === 0) {
                      reject(new Error('Translation result is empty after processing.'));
                      return;
                    }

                    document.body.removeChild(translateElement);
                    document.body.removeChild(googleTranslateElement);

                    resolve({
                      translatedText: finalContent,
                      sourceLanguage: 'en',
                      targetLanguage: targetLang
                    });
                  } else {
                    reject(new Error('Translation service returned empty result.'));
                  }
                }
              );
            } else {
              // Fallback method using Google Translate URL
              const encodedText = encodeURIComponent(processedContent);
              const apiUrl = `https://translate.googleapis.com/translate_a/single?client=gtx&sl=en&tl=${targetLang}&dt=t&q=${encodedText}`;

              fetch(apiUrl)
                .then(response => response.json())
                .then(data => {
                  if (data && data[0] && data[0].length > 0) {
                    let translatedText = '';
                    for (let i = 0; i < data[0].length; i++) {
                      if (data[0][i][0]) {
                        translatedText += data[0][i][0];
                      }
                    }

                    // Restore preserved elements back to the translated content
                    const finalContent = restorePreservedElements(translatedText, preservedElements);

                    // Validate the final content
                    if (!finalContent || finalContent.trim().length === 0) {
                      reject(new Error('Translation result is empty after processing.'));
                      return;
                    }

                    document.body.removeChild(translateElement);
                    document.body.removeChild(googleTranslateElement);

                    resolve({
                      translatedText: finalContent,
                      sourceLanguage: 'en',
                      targetLanguage: targetLang
                    });
                  } else {
                    reject(new Error('Translation service returned empty result.'));
                  }
                })
                .catch(error => {
                  document.body.removeChild(translateElement);
                  document.body.removeChild(googleTranslateElement);
                  reject(error);
                });
            }
          }, 500);
        } catch (error) {
          console.error('Translation error:', error);
          reject(error);
        }
      });
    } catch (error) {
      console.error('Translation error details:', error);
      let errorMessage = 'Translation failed. ';

      if (error.message.includes('Network Error') || error.message.includes('Failed to fetch')) {
        errorMessage += 'Cannot connect to translation service. Check your internet connection.';
      } else if (error.message.includes('Invalid content')) {
        errorMessage += 'Invalid content provided for translation.';
      } else if (error.message.includes('empty result')) {
        errorMessage += 'Translation service returned empty result.';
      } else {
        errorMessage += error.message || 'Please try again or check console for details.';
      }

      setTranslationError(errorMessage);
      throw error;
    } finally {
      setIsTranslating(false);
    }
  };

  // Method using Google Translate API directly
  const translateTextViaGoogle = async (text, targetLang) => {
    // Check if we're in the browser environment
    if (typeof window === 'undefined' || !isClient) {
      throw new Error('Translation can only be performed in the browser.');
    }

    // Validate input
    if (!text || typeof text !== 'string') {
      throw new Error('Invalid content provided for translation.');
    }

    setIsTranslating(true);
    setTranslationError(null);

    try {
      // Extract and preserve code blocks, diagrams, images, and technical terms
      const { processedContent, preservedElements } = extractAndPreserveElements(text);

      // Validate that we have content to translate after processing
      if (!processedContent || processedContent.trim().length === 0) {
        throw new Error('No translatable content found after processing.');
      }

      // Use Google Translate API via Google Translate Element
      // First, ensure the Google Translate script is loaded
      if (typeof google === 'undefined' || !google.translate) {
        // Load the Google Translate script dynamically
        return await translateViaFetchAPI(processedContent, targetLang, preservedElements);
      }

      // For now, using the fetch API approach as it's more reliable
      return await translateViaFetchAPI(processedContent, targetLang, preservedElements);
    } catch (error) {
      console.error('Translation error details:', error);
      let errorMessage = 'Translation failed. ';

      if (error.message.includes('Network Error') || error.message.includes('Failed to fetch')) {
        errorMessage += 'Cannot connect to Google Translate service. Check your internet connection.';
      } else if (error.message.includes('Invalid content')) {
        errorMessage += 'Invalid content provided for translation.';
      } else if (error.message.includes('empty result')) {
        errorMessage += 'Translation service returned empty result.';
      } else {
        errorMessage += error.message || 'Please try again or check console for details.';
      }

      setTranslationError(errorMessage);
      throw error;
    } finally {
      setIsTranslating(false);
    }
  };

  // Helper function to translate via Google Translate API
  const translateViaFetchAPI = async (text, targetLang, preservedElements) => {
    try {
      // Encode the text for the URL
      const encodedText = encodeURIComponent(text);
      const apiUrl = `https://translate.googleapis.com/translate_a/single?client=gtx&sl=en&tl=${targetLang}&dt=t&q=${encodedText}`;

      const response = await fetch(apiUrl);

      if (!response.ok) {
        throw new Error(`Google Translate API error: ${response.status}`);
      }

      const data = await response.json();

      if (data && data[0]) {
        let translatedText = '';
        for (let i = 0; i < data[0].length; i++) {
          if (data[0][i][0]) {
            translatedText += data[0][i][0];
          }
        }

        // Restore preserved elements back to the translated content
        // This will put back code blocks, images, diagrams, etc. in their original positions
        const finalContent = restorePreservedElements(translatedText, preservedElements);

        // Validate the final content
        if (!finalContent || finalContent.trim().length === 0) {
          throw new Error('Translation result is empty after processing.');
        }

        return {
          translatedText: finalContent,
          sourceLanguage: 'en',
          targetLanguage: targetLang
        };
      } else {
        throw new Error('Google Translate service returned empty result.');
      }
    } catch (error) {
      console.error('Google Translate API error:', error);
      throw error;
    }
  };

  const handleTranslate = async () => {
    // Check if we're in the browser environment
    if (typeof window === 'undefined' || !isClient) {
      setTranslationError('Translation can only be performed in the browser.');
      return;
    }

    setTranslationError(null);

    try {
      if (currentLang === 'en') {
        // Translate to Urdu
        setIsTranslating(true);
        setTranslationError(null);

        let contentToTranslate = null;

        // Check if we're in the browser before accessing DOM
        if (typeof window !== 'undefined' && window.document) {
          // Get the current chapter content from the DOM with more specific selectors
          // Try multiple selectors in order of preference to find the main content area
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

          let contentElement = null;
          for (const selector of selectors) {
            contentElement = document.querySelector(selector);
            if (contentElement && contentElement.innerHTML && contentElement.innerHTML.trim()) {
              break; // Found a valid element with content
            }
          }

          if (contentElement && contentElement.innerHTML) {
            // Get the full HTML content for translation
            contentToTranslate = contentElement.innerHTML;
          } else if (originalContentHTML) {
            // Use the content passed from the parent component
            contentToTranslate = originalContentHTML;
          }
        } else if (originalContentHTML) {
          // Fallback for SSR - use the content passed from the parent component
          contentToTranslate = originalContentHTML;
        }

        if (contentToTranslate && contentToTranslate.trim()) {
          // Translate the content using Google Translate API
          const result = await translateTextViaGoogle(contentToTranslate, 'ur');

          // Store the translated text in context
          if (setTranslatedContent && setIsTranslated) {
            setTranslatedContent(result.translatedText);
            setIsTranslated(true);
          }
        } else if (originalContentHTML && originalContentHTML.trim()) {
          // If DOM content wasn't found, try using the original content passed from parent
          const result = await translateTextViaGoogle(originalContentHTML, 'ur');

          // Store the translated text in context
          if (setTranslatedContent && setIsTranslated) {
            setTranslatedContent(result.translatedText);
            setIsTranslated(true);
          }
        } else {
          throw new Error('No content found to translate. Please ensure the page has loaded completely before attempting translation.');
        }

        const newLang = 'ur';
        setCurrentLang(newLang);
        if (propSetCurrentLang) {
          propSetCurrentLang(newLang);
        }
      } else {
        // Switch back to English - reset to original content
        if (setTranslatedContent && setIsTranslated) {
          setTranslatedContent(null);
          setIsTranslated(false);
        }
        const newLang = 'en';
        setCurrentLang(newLang);
        if (propSetCurrentLang) {
          propSetCurrentLang(newLang);
        }
      }
    } catch (error) {
      console.error('Translation failed:', error);
      // Error is already set in translateText function
    }
  };

  // Clean up on component unmount
  useEffect(() => {
    return () => {
      if (typeof window !== 'undefined' && isClient) {
        // Reset translation state when component unmounts if we're in translated state
        if (currentLang === 'ur' && setTranslatedContent && setIsTranslated) {
          setTranslatedContent(null);
          setIsTranslated(false);
        }
      }
    };
  }, [currentLang, setTranslatedContent, setIsTranslated]);

  return (
    <div className={styles.translateContainer}>
      {isClient ? (
        <button
          className={`${styles.translateButton} ${isTranslating ? styles.loading : ''}`}
          onClick={handleTranslate}
          disabled={isTranslating}
          title={currentLang === 'en' ? 'Translate this chapter to Urdu' : 'Switch back to English'}
        >
          {isTranslating ? (
            <>
              <span className={styles.spinner}></span>
              Translating...
            </>
          ) : currentLang === 'en' ? (
            '.Translate to Urdu'
          ) : (
            'Switch to English'
          )}
        </button>
      ) : (
        <button
          className={styles.translateButton}
          disabled={true}
          title="Translation service initializing..."
        >
          Loading...
        </button>
      )}

      {translationError && (
        <div className={styles.translationError}>
          {translationError}
        </div>
      )}

      {isClient && currentLang === 'ur' && !isTranslating && !translationError && translatorReady && (
        <div className={styles.translationNotice}>
          Content translated to Urdu
        </div>
      )}

      {isClient && currentLang === 'en' && !isTranslating && !translationError && translatorReady && (
        <div className={styles.translationNotice}>
          Showing content in English
        </div>
      )}
    </div>
  );
};

export default TranslateButton;