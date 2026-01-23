import React from 'react';
import { TranslationProvider, useTranslation } from '../../contexts/TranslationContext';
import TranslateButtonComponent from './index';

// Wrapper component that provides its own TranslationProvider to avoid SSR issues
const TranslateButtonWrapperComponent = (props) => {
  const { translatedContent, setTranslatedContent, isTranslated, setIsTranslated, currentLang, setCurrentLang } = useTranslation();

  return (
    <TranslateButtonComponent
      originalContentHTML={props.originalContentHTML}
      setTranslatedContent={setTranslatedContent}
      setIsTranslated={setIsTranslated}
      currentLang={currentLang}
      setCurrentLang={setCurrentLang}
    />
  );
};

const TranslateButtonWrapper = (props) => {
  // Only initialize TranslationProvider in browser environment
  if (typeof window === 'undefined') {
    return null;
  }

  return (
    <TranslationProvider>
      <TranslateButtonWrapperComponent {...props} />
    </TranslationProvider>
  );
};

export default TranslateButtonWrapper;