import React, { useState, useEffect, useRef } from 'react';
import DOMPurify from 'dompurify';

interface StreamingResponseDisplayProps {
  initialContent?: string;
  onContentUpdate?: (content: string) => void;
}

export const StreamingResponseDisplay: React.FC<StreamingResponseDisplayProps> = ({
  initialContent = '',
  onContentUpdate
}) => {
  const [content, setContent] = useState<string>(initialContent);
  const [isStreaming, setIsStreaming] = useState<boolean>(false);
  const contentRef = useRef<HTMLDivElement>(null);

  // Update content when it changes
  useEffect(() => {
    if (onContentUpdate) {
      onContentUpdate(content);
    }
  }, [content, onContentUpdate]);

  // Auto-scroll to bottom when content updates
  useEffect(() => {
    if (contentRef.current) {
      contentRef.current.scrollTop = contentRef.current.scrollHeight;
    }
  }, [content]);

  const sanitizedContent = DOMPurify.sanitize(content, {
    ALLOWED_TAGS: [
      'p', 'br', 'strong', 'em', 'u', 'ol', 'ul', 'li', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6',
      'blockquote', 'code', 'pre', 'a', 'table', 'thead', 'tbody', 'tr', 'th', 'td'
    ],
    ALLOWED_ATTR: ['href', 'target', 'rel']
  });

  return (
    <div className="streaming-response-container">
      <div
        ref={contentRef}
        className="streaming-response-content"
        dangerouslySetInnerHTML={{ __html: sanitizedContent }}
      />
      {isStreaming && (
        <div className="streaming-indicator">
          <span className="typing-indicator">
            <span></span>
            <span></span>
            <span></span>
          </span>
        </div>
      )}
    </div>
  );
};

export default StreamingResponseDisplay;