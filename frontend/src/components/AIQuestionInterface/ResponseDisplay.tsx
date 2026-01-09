import React from 'react';
import { QueryResponse } from '../../../services/api/types';

interface ResponseDisplayProps {
  response?: QueryResponse;
  loading?: boolean;
  error?: string;
}

export const ResponseDisplay: React.FC<ResponseDisplayProps> = ({
  response,
  loading,
  error
}) => {
  if (loading) {
    return (
      <div className="response-display loading">
        <div className="loading-spinner"></div>
        <p>AI is processing your question...</p>
      </div>
    );
  }

  if (error) {
    return (
      <div className="response-display error">
        <p className="error-message">{error}</p>
      </div>
    );
  }

  if (!response) {
    return null;
  }

  return (
    <div className="response-display">
      <div className="response-content">
        <div className="answer">
          {response.answer}
        </div>

        {response.sources && response.sources.length > 0 && (
          <div className="sources-section">
            <h4>Sources:</h4>
            <ul className="sources-list">
              {response.sources.map((source, index) => (
                <li key={index} className="source-item">
                  <a
                    href={source.url}
                    target="_blank"
                    rel="noopener noreferrer"
                    className="source-link"
                  >
                    {source.title}
                  </a>
                  <p className="source-excerpt">
                    {source.content.substring(0, 200)}{source.content.length > 200 ? '...' : ''}
                  </p>
                </li>
              ))}
            </ul>
          </div>
        )}
      </div>
    </div>
  );
};

export default ResponseDisplay;