import React from 'react';
import { QuestionResponsePair } from '../Utils/SessionManager';

interface ConversationHistoryProps {
  history: QuestionResponsePair[];
  onQuestionClick?: (question: string) => void;
  onFollowUpClick?: (question: string) => void;
}

export const ConversationHistory: React.FC<ConversationHistoryProps> = ({
  history,
  onQuestionClick,
  onFollowUpClick
}) => {
  if (history.length === 0) {
    return (
      <div className="conversation-history-empty">
        <p>No conversation history yet. Ask a question to start a conversation.</p>
      </div>
    );
  }

  return (
    <div className="conversation-history">
      <h3>Conversation History</h3>
      <div className="history-list">
        {history.map((item, index) => (
          <div key={index} className="history-item">
            <div className="question" onClick={() => onQuestionClick && onQuestionClick(item.question)}>
              <strong>You:</strong> {item.question}
            </div>
            <div className="response">
              <strong>AI:</strong> {typeof item.response === 'object' && item.response.answer
                ? item.response.answer
                : item.response}
            </div>
            <div className="history-actions">
              <button
                type="button"
                className="follow-up-button"
                onClick={(e) => {
                  e.stopPropagation(); // Prevent the question click handler from firing
                  onFollowUpClick && onFollowUpClick(item.question);
                }}
              >
                Ask follow-up
              </button>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};

export default ConversationHistory;