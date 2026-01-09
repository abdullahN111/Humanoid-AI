import React from 'react';

interface AIConfigurationProps {
  topK: number;
  relevanceThreshold: number;
  includeCitations: boolean;
  onTopKChange: (value: number) => void;
  onRelevanceThresholdChange: (value: number) => void;
  onIncludeCitationsChange: (value: boolean) => void;
}

export const AIConfiguration: React.FC<AIConfigurationProps> = ({
  topK,
  relevanceThreshold,
  includeCitations,
  onTopKChange,
  onRelevanceThresholdChange,
  onIncludeCitationsChange
}) => {
  return (
    <div className="ai-configuration">
      <h4>Configuration Options</h4>
      <div className="config-option">
        <label htmlFor="top-k">Number of results (top_k):</label>
        <input
          id="top-k"
          type="number"
          min="1"
          max="20"
          value={topK}
          onChange={(e) => onTopKChange(Number(e.target.value))}
        />
      </div>

      <div className="config-option">
        <label htmlFor="relevance-threshold">Relevance threshold (0-1):</label>
        <input
          id="relevance-threshold"
          type="number"
          min="0"
          max="1"
          step="0.01"
          value={relevanceThreshold}
          onChange={(e) => onRelevanceThresholdChange(Number(e.target.value))}
        />
      </div>

      <div className="config-option">
        <label htmlFor="include-citations">Include citations:</label>
        <input
          id="include-citations"
          type="checkbox"
          checked={includeCitations}
          onChange={(e) => onIncludeCitationsChange(e.target.checked)}
        />
      </div>
    </div>
  );
};

export default AIConfiguration;