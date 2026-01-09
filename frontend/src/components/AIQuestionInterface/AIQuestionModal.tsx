import React, { useEffect } from 'react';
import { createPortal } from 'react-dom';
import { AIQuestionInterface } from './AIQuestionInterface';

interface AIQuestionModalProps {
  isOpen: boolean;
  onClose: () => void;
}

export const AIQuestionModal: React.FC<AIQuestionModalProps> = ({
  isOpen,
  onClose
}) => {
  useEffect(() => {
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        onClose();
      }
    };

    if (isOpen) {
      document.addEventListener('keydown', handleEscape);
      document.body.style.overflow = 'hidden';
    }

    return () => {
      document.removeEventListener('keydown', handleEscape);
      document.body.style.overflow = '';
    };
  }, [isOpen, onClose]);

  if (!isOpen) {
    return null;
  }

  return createPortal(
    <div className="ai-question-modal-overlay" onClick={onClose}>
      <div className="ai-question-modal" onClick={(e) => e.stopPropagation()}>
        <div className="modal-header">
          <h3>Ask a Question</h3>
          <button className="close-button" onClick={onClose} aria-label="Close">
            ×
          </button>
        </div>
        <div className="modal-content">
          <AIQuestionInterface />
        </div>
      </div>
    </div>,
    document.body
  );
};

export default AIQuestionModal;