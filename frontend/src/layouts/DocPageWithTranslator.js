import React from 'react';
import Layout from '@theme/Layout';
import { useLocation } from '@docusaurus/router';
import TranslateButtonWrapper from '@site/src/components/TranslateButton/TranslateButtonWrapper';

// Custom layout that includes the translator button on documentation pages
export default function DocPageWithTranslator(props) {
  const location = useLocation();

  // Show translator button only on documentation pages (not on auth pages, etc.)
  const isDocPage = location.pathname.startsWith('/docs/');

  return (
    <Layout {...props}>
      <div style={{ position: 'relative' }}>
        {/* Render the original content */}
        {props.children}

        {/* Add translator button at the end of documentation pages */}
        {isDocPage && (
          <div style={{ marginTop: '2rem', paddingTop: '1rem', borderTop: '1px solid #eee' }}>
            <TranslateButtonWrapper />
          </div>
        )}
      </div>
    </Layout>
  );
}