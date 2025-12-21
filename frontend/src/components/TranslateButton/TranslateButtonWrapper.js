import React from 'react';
import { useTranslation } from '@site/src/contexts/TranslationContext';
import TranslateButtonComponent from './index';

const TranslateButtonWrapper = (props) => {
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

export default TranslateButtonWrapper;