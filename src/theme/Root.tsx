import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <>
      {children}
      <BrowserOnly fallback={<div />}>
        {() => <ChatWidget />}
      </BrowserOnly>
    </>
  );
}
