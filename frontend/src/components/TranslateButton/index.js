import React, { useState, useEffect, useRef } from 'react';
import styles from './TranslateButton.module.css';
import { API_BASE_URL } from '../../constants/apiConfig';

const TranslateButton = ({ originalContentHTML, setTranslatedContent, setIsTranslated, currentLang: propCurrentLang, setCurrentLang: propSetCurrentLang }) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [currentLang, setCurrentLang] = useState(propCurrentLang || 'en'); // 'en' for English, 'ur' for Urdu
  const [translationError, setTranslationError] = useState(null);
  const [translatorReady, setTranslatorReady] = useState(true);
  const [isClient, setIsClient] = useState(true); // Always true since rendered in BrowserOnly
  const contentRef = useRef(null);

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

  // Function to translate content using backend API
  const translateText = async (text, targetLang) => {
    if (!text || typeof text !== 'string') {
      throw new Error('Invalid content provided for translation.');
    }

    setIsTranslating(true);
    setTranslationError(null);

    try {
      // Simplified approach: only preserve essential code blocks
      let processedContent = text;
      const preservedElements = [];

      // Only preserve code blocks and keep it simple
      const codeBlockRegex = /```[\s\S]*?```/g;
      let match;
      let index = 0;

      while ((match = codeBlockRegex.exec(text)) !== null) {
        const placeholder = `__CODE_BLOCK_${index}__`;
        preservedElements.push({
          placeholder,
          original: match[0]
        });
        processedContent = processedContent.replace(match[0], placeholder);
        index++;
      }

      // Also preserve inline code
      const inlineCodeRegex = /`[^`]+`/g;
      while ((match = inlineCodeRegex.exec(processedContent)) !== null) {
        const placeholder = `__INLINE_CODE_${index}__`;
        preservedElements.push({
          placeholder,
          original: match[0]
        });
        processedContent = processedContent.replace(match[0], placeholder);
        index++;
      }

      if (!processedContent || processedContent.trim().length === 0) {
        throw new Error('No translatable content found after processing.');
      }

      // Call backend translation API
      const response = await fetch(`${API_BASE_URL}/api/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          text: processedContent,
          target_language: targetLang,
          source_language: 'en'
        })
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `Translation API error: ${response.status}`);
      }

      const result = await response.json();

      if (!result.translated_text) {
        throw new Error('Translation API returned empty result.');
      }

      // Restore preserved elements
      let finalContent = result.translated_text;
      for (const element of preservedElements) {
        finalContent = finalContent.replace(element.placeholder, element.original);
      }

      if (!finalContent || finalContent.trim().length === 0) {
        throw new Error('Translation result is empty after processing.');
      }

      return {
        translatedText: finalContent,
        sourceLanguage: result.source_language,
        targetLanguage: result.target_language
      };

    } catch (error) {
      console.error('Translation error details:', error);
      let errorMessage = 'Translation failed. ';

      if (error.message.includes('Failed to fetch') || error.message.includes('Network Error')) {
        errorMessage += 'Cannot connect to translation service. Check your internet connection.';
      } else if (error.message.includes('401') || error.message.includes('Unauthorized')) {
        errorMessage += 'Please sign in to use translation feature.';
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

  const handleTranslate = async () => {
    setTranslationError(null);

    try {
      if (currentLang === 'en') {
        // Translate to Urdu
        setIsTranslating(true);
        setTranslationError(null);

        let contentToTranslate = null;

        // Get content from DOM or use passed content
        if (typeof window !== 'undefined' && window.document) {
          const contentElement = document.querySelector('main div.markdown') ||
                                document.querySelector('main article') ||
                                document.querySelector('main');

          if (contentElement && contentElement.innerHTML) {
            contentToTranslate = contentElement.innerHTML;
          } else if (originalContentHTML) {
            contentToTranslate = originalContentHTML;
          }
        } else if (originalContentHTML) {
          contentToTranslate = originalContentHTML;
        }

        if (contentToTranslate && contentToTranslate.trim()) {
          // Translate the content using backend API
          const result = await translateText(contentToTranslate, 'ur');

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
      <button
        className={`${styles.translateButton} ${isTranslating ? styles.loading : ''}`}
        onClick={handleTranslate}
        disabled={isTranslating || !isClient}
        title={currentLang === 'en' ? 'Translate this chapter to Urdu' : 'Switch back to English'}
      >
        {isTranslating ? (
          <>
            <span className={styles.spinner}></span>
            Translating...
          </>
        ) : currentLang === 'en' ? (
          'Translate to Urdu'
        ) : (
          'Switch to English'
        )}
      </button>

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