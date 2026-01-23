import React from 'react';
import { useLocation } from '@docusaurus/router';
import OriginalDocContent from '@theme-original/DocContent';
import TranslateButtonWrapper from '../components/TranslateButton/TranslateButtonWrapper';

export default function DocContent(props) {
  const location = useLocation();

  // Only show translator on documentation pages
  const isDocPage = location.pathname.startsWith('/docs/');

  return (
    <>
      <OriginalDocContent {...props} />

      {/* Translator button at the end of documentation pages */}
      {isDocPage && (
        <div style={{ marginTop: '2rem', paddingTop: '1rem', borderTop: '1px solid #eee', textAlign: 'center' }}>
          <TranslateButtonWrapper />
        </div>
      )}
    </>
  );
}