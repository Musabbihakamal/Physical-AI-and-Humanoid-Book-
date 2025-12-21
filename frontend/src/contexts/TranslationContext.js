import React, { createContext, useContext, useState } from 'react';

const TranslationContext = createContext();

// Provider that manages translation state
export const TranslationProvider = ({ children }) => {
  const [translatedContent, setTranslatedContent] = useState(null);
  const [isTranslated, setIsTranslated] = useState(false);
  const [currentLang, setCurrentLang] = useState('en');

  const value = {
    translatedContent,
    setTranslatedContent,
    isTranslated,
    setIsTranslated,
    currentLang,
    setCurrentLang
  };

  return (
    <TranslationContext.Provider value={value}>
      {children}
    </TranslationContext.Provider>
  );
};

export const useTranslation = () => {
  const context = useContext(TranslationContext);
  if (!context) {
    throw new Error('useTranslation must be used within a TranslationProvider');
  }
  return context;
};