import React, { useState, useEffect, useRef } from 'react';
import { AIQuestionInterface } from '../AIQuestionInterface';
import { PageContextProvider } from '../PageContextProvider';
import { SelectedTextProvider } from '../SelectedTextHandler';
import './styles.css';

interface FloatingChatbotProps {
  config?: {
    baseUrl?: string;
    timeout?: number;
  };
}

export const FloatingChatbot: React.FC<FloatingChatbotProps> = ({
  config = {
    baseUrl: typeof window !== 'undefined' && window.ENV && window.ENV.API_BASE_URL
      ? window.ENV.API_BASE_URL
      : (typeof process !== 'undefined' && process.env
        ? process.env.REACT_APP_AGENT_API_URL || 'http://localhost:8000'
        : 'http://localhost:8000'),
    timeout: 30000,
  },
}) => {
  const [isOpen, setIsOpen] = useState<boolean>(false);
  const [isMinimized, setIsMinimized] = useState<boolean>(false);
  const [hasUnread, setHasUnread] = useState<boolean>(false);
  const chatContainerRef = useRef<HTMLDivElement>(null);

  // Close chat when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (chatContainerRef.current && !chatContainerRef.current.contains(event.target as Node) && isOpen && !isMinimized) {
        setIsOpen(false);
      }
    };

    if (isOpen && !isMinimized) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen, isMinimized]);

  const toggleChat = () => {
    if (!isOpen) {
      setIsOpen(true);
      setHasUnread(false);
    } else {
      setIsMinimized(!isMinimized);
    }
  };

  const closeChat = () => {
    setIsOpen(false);
    setIsMinimized(false);
  };

  return (
    <>
      {/* Floating chat button */}
      <div className={`floating-chat-button ${isOpen && !isMinimized ? 'hidden' : ''}`} onClick={toggleChat}>
        <div className="chat-icon">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H17L14.25 20.75C14.0288 21.0382 13.7192 21.2387 13.3718 21.318C13.0244 21.3973 12.659 21.3509 12.3579 21.1887C12.0567 21.0265 11.8383 20.7631 11.75 20.45C11.6617 20.1369 11.7131 19.8058 11.8879 19.5379L13.5 17H5C4.46957 17 3.96086 16.7893 3.58579 16.4142C3.21071 16.0391 3 15.5304 3 15V5C3 4.46957 3.21071 3.96086 3.58579 3.58579C3.96086 3.21071 4.46957 3 5 3H19C19.5304 3 20.0391 3.21071 20.4142 3.58579C20.7893 3.96086 21 4.46957 21 5V15Z" stroke="white" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        </div>
        {hasUnread && <div className="unread-indicator"></div>}
      </div>

      {/* Chat container */}
      {isOpen && (
        <div
          ref={chatContainerRef}
          className={`floating-chat-container ${isMinimized ? 'minimized' : 'expanded'}`}
        >
          {/* Chat header */}
          <div className="chat-header">
            <div className="chat-title">AI Assistant</div>
            <div className="chat-controls">
              <button
                className="minimize-btn"
                onClick={() => setIsMinimized(true)}
                title="Minimize"
              >
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M4 8H12" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
                </svg>
              </button>
              <button
                className="close-btn"
                onClick={closeChat}
                title="Close"
              >
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M12 4L4 12M4 4L12 12" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
                </svg>
              </button>
            </div>
          </div>

          {/* Chat content */}
          {!isMinimized && (
            <div className="chat-content">
              <PageContextProvider initialContext={{
                url: typeof window !== 'undefined' ? window.location.href : '',
                title: typeof window !== 'undefined' ? document.title : '',
                section: window.location.pathname.split('/').pop() || ''
              }}>
                <SelectedTextProvider>
                  <AIQuestionInterface
                    config={config}
                    currentPageContext={{
                      url: typeof window !== 'undefined' ? window.location.href : '',
                      title: typeof window !== 'undefined' ? document.title : '',
                      section: window.location.pathname.split('/').pop() || ''
                    }}
                  />
                </SelectedTextProvider>
              </PageContextProvider>
            </div>
          )}
        </div>
      )}
    </>
  );
};

export default FloatingChatbot;