import React, { useState } from 'react';

interface QueryInputProps {
  value: string;
  onChange: (value: string) => void;
  onSubmit: () => void;
  disabled?: boolean;
  placeholder?: string;
}

export const QueryInput: React.FC<QueryInputProps> = ({
  value,
  onChange,
  onSubmit,
  disabled = false,
  placeholder = 'Ask a question about this documentation...'
}) => {
  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    onSubmit();
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      onSubmit();
    }
  };

  return (
    <form onSubmit={handleSubmit} className="query-input-form">
      <div className="input-container">
        <textarea
          value={value}
          onChange={(e) => onChange(e.target.value)}
          placeholder={placeholder}
          className="query-textarea"
          rows={3}
          disabled={disabled}
          onKeyDown={handleKeyDown}
        />
        <button
          type="submit"
          disabled={disabled || !value.trim()}
          className="submit-button"
        >
          {disabled ? 'Sending...' : 'Send'}
        </button>
      </div>
    </form>
  );
};

export default QueryInput;