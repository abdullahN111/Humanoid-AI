import React, { useState, useEffect } from 'react';
import { useChatState } from '../../hooks/useChatState';
import { usePageContext } from '../PageContextProvider';
import { useSelectedText } from '../SelectedTextHandler';
import { QueryRequest } from '../../services/api/types';
import { isValidQuery, sanitizeInput } from '../../utils/helpers';
import './styles.css';

interface AIQuestionInterfaceProps {
  onLoadingChange?: (loading: boolean) => void;
}

export const AIQuestionInterface: React.FC<AIQuestionInterfaceProps> = ({
  onLoadingChange
}) => {
  const [question, setQuestion] = useState<string>('');
  const { chatState, sendQuery, clearChat } = useChatState();
  const { pageContext } = usePageContext();
  const { selectedText } = useSelectedText();

  useEffect(() => {
    if (onLoadingChange) {
      onLoadingChange(chatState.loading);
    }
  }, [chatState.loading, onLoadingChange]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!isValidQuery(question)) {
      console.error('Invalid query');
      return;
    }

    const sanitizedQuestion = sanitizeInput(question);

    try {
      const request: QueryRequest = {
        query: sanitizedQuestion,
        pageContext,
        selectedText,
        retrieval_params: {
          top_k: 3,
          threshold: -0.1,  // Low threshold to accommodate negative similarity scores
        },
        grounding_required: true,
      };

      await sendQuery(request);
    } catch (error) {
      console.error('Error sending query:', error);
    }
  };

  return (
    <div className="ai-question-interface">
      <form onSubmit={handleSubmit} className="question-form">
        <div className="input-container">
          <textarea
            value={question}
            onChange={(e) => setQuestion(e.target.value)}
            placeholder="Ask a question about this documentation..."
            className="question-input"
            rows={3}
            disabled={chatState.loading}
          />
          <button
            type="submit"
            disabled={chatState.loading || !question.trim()}
            className="ask-button"
          >
            {chatState.loading ? 'Asking...' : 'Ask AI'}
          </button>
        </div>

        {selectedText && (
          <div className="selected-text-preview">
            <small>Including selected text: "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</small>
          </div>
        )}
      </form>

      {chatState.loading && (
        <div className="loading-state">
          <div className="spinner"></div>
          <p>AI is processing your question...</p>
        </div>
      )}

      {chatState.error && (
        <div className="error-state">
          <p>{chatState.error}</p>
        </div>
      )}

      {chatState.response && (
        <div className="response-container">
          <div className="response-content">
            {(() => {
              // Clean up the response to remove redundant "not enough information" messages
              // when sources are available
              let displayAnswer = chatState.response.answer;

              if (chatState.response.sources && chatState.response.sources.length > 0) {
                // If there are sources, try to clean up the response to focus on the useful content
                const lines = displayAnswer.split('\n');
                const cleanedLines = lines.filter(line =>
                  !line.trim().startsWith('The provided context does not contain enough information') ||
                  line.trim().includes('Source') // Keep lines that mention sources
                );
                displayAnswer = cleanedLines.join('\n');
              }

              return <div dangerouslySetInnerHTML={{ __html: displayAnswer.replace(/\n/g, '<br />') }} />;
            })()}
          </div>

          {chatState.response.sources && chatState.response.sources.length > 0 && (
            <div className="sources-section">
              <h4>Sources:</h4>
              <ul>
                {chatState.response.sources.map((source, index) => (
                  <li key={index}>
                    <a href={source.url} target="_blank" rel="noopener noreferrer">
                      {source.title}
                    </a>
                    <p>{source.content.substring(0, 150)}{source.content.length > 150 ? '...' : ''}</p>
                  </li>
                ))}
              </ul>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default AIQuestionInterface;