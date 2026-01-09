import React from 'react';
import DOMPurify from 'dompurify';

interface ResponseDisplayProps {
  response: any; // Using 'any' to accommodate the AIResponseModel structure
  showCitations?: boolean;
  showSources?: boolean;
}

export const ResponseDisplay: React.FC<ResponseDisplayProps> = ({
  response,
  showCitations = true,
  showSources = true
}) => {
  // Sanitize the response content to prevent XSS
  const sanitizeContent = (content: string): string => {
    return DOMPurify.sanitize(content, {
      ALLOWED_TAGS: [
        'p', 'br', 'strong', 'em', 'u', 'ol', 'ul', 'li', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6',
        'blockquote', 'code', 'pre', 'a', 'table', 'thead', 'tbody', 'tr', 'th', 'td'
      ],
      ALLOWED_ATTR: ['href', 'target', 'rel']
    });
  };

  if (!response) {
    return null;
  }

  return (
    <div className="response-container">
      <div
        className="response-content"
        dangerouslySetInnerHTML={{
          __html: sanitizeContent(response.answer || response.content || response.text || '')
        }}
      />

      {showCitations && response.validation_details && (
        <div className="validation-details">
          <p><strong>Relevance Score:</strong> {(response.relevance_score * 100).toFixed(1)}%</p>
          <p><strong>Grounding Validation:</strong> {response.validation_details.grounding_validation_passed ? 'Passed' : 'Failed'}</p>
          {response.validation_details.hallucination_detected && (
            <p><strong>Warning:</strong> Potential hallucination detected</p>
          )}
        </div>
      )}

      {showSources && response.sources && response.sources.length > 0 && (
        <div className="sources-section">
          <h4>Sources:</h4>
          <ul>
            {response.sources.map((source: string, index: number) => (
              <li key={index}>
                <a href={source} target="_blank" rel="noopener noreferrer">
                  {source.length > 60 ? source.substring(0, 60) + '...' : source}
                </a>
              </li>
            ))}
          </ul>
        </div>
      )}

      {response.retrieved_chunks && response.retrieved_chunks.length > 0 && showCitations && (
        <details className="chunks-details">
          <summary>View retrieved content chunks</summary>
          <div className="chunks-list">
            {response.retrieved_chunks.slice(0, 3).map((chunk: any, index: number) => (
              <div key={index} className="chunk-item">
                <p><strong>Source:</strong> {chunk.source_title}</p>
                <p><strong>Score:</strong> {(chunk.similarity_score * 100).toFixed(1)}%</p>
                <p><small>{chunk.chunk_text.substring(0, 200)}{chunk.chunk_text.length > 200 ? '...' : ''}</small></p>
              </div>
            ))}
          </div>
        </details>
      )}
    </div>
  );
};

export default ResponseDisplay;