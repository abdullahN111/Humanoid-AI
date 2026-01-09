import React, { useState, useEffect } from 'react';
import { FloatingChatButton } from './FloatingChatButton';
import { AIQuestionModal } from './AIQuestionModal';
import { PageContextProvider } from '../PageContextProvider';
import { SelectedTextHandler } from '../SelectedTextHandler';

interface AIChatWrapperProps {
  children?: React.ReactNode;
}

export const AIChatWrapper: React.FC<AIChatWrapperProps> = ({ children }) => {
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [selectedText, setSelectedText] = useState('');

  const handleOpenModal = () => {
    setIsModalOpen(true);
  };

  const handleCloseModal = () => {
    setIsModalOpen(false);
  };

  // Get current page context
  const pageContext = {
    url: typeof window !== 'undefined' ? window.location.href : '',
    title: typeof window !== 'undefined' ? document.title : ''
  };

  return (
    <PageContextProvider initialContext={pageContext}>
      <SelectedTextHandler onSelect={setSelectedText}>
        {children}
        <FloatingChatButton onClick={handleOpenModal} />
        <AIQuestionModal
          isOpen={isModalOpen}
          onClose={handleCloseModal}
          selectedText={selectedText}
          pageContext={pageContext}
        />
      </SelectedTextHandler>
    </PageContextProvider>
  );
};