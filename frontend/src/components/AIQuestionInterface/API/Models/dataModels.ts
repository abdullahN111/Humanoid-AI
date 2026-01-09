// TypeScript interfaces for data models based on data-model.md

export interface QuestionRequestModel {
  question: string;
  context: {
    selected_text?: string;
    current_page_url: string;
    current_page_title: string;
    document_section?: string;
  };
  metadata?: {
    user_id?: string;
    session_id?: string;
    timestamp: number;
  };
  config?: {
    top_k?: number;
    relevance_threshold?: number;
    include_citations?: boolean;
  };
}

export interface AIResponseModel {
  id: string;
  question: string;
  answer: string;
  sources: string[];
  retrieved_chunks: RetrievedChunkModel[];
  relevance_score: number;
  is_valid: boolean;
  execution_time: number;
  validation_details: {
    content_similarity_score: number;
    source_citation_accuracy: number;
    hallucination_detected: boolean;
    grounding_validation_passed: boolean;
    total_results: number;
    top_result_score: number;
  };
  error_message?: string;
}

export interface RetrievedChunkModel {
  id: string;
  chunk_text: string;
  similarity_score: number;
  source_url: string;
  source_title: string;
  section_hierarchy?: string;
  rank: number;
  content_hash: string;
}

export interface ConversationSessionModel {
  id: string;
  user_id: string;
  created_at: number;
  last_accessed: number;
  conversation_history: ConversationEntryModel[];
  metadata?: {
    current_document?: string;
    user_preferences?: Record<string, any>;
  };
}

export interface ConversationEntryModel {
  id: string;
  type: 'question' | 'response';
  content: string;
  timestamp: number;
  context?: {
    selected_text?: string;
    current_page_url?: string;
    current_page_title?: string;
  };
  metadata?: {
    processing_time?: number;
    source_citations?: string[];
  };
}

export interface UIStateModel {
  currentQuestion: string;
  isLoading: boolean;
  currentResponse: AIResponseModel | null;
  error: string | null;
  session: ConversationSessionModel | null;
  inputContext: {
    selectedText: string;
    currentPageUrl: string;
    currentPageTitle: string;
  };
  displayOptions: {
    showCitations: boolean;
    showSources: boolean;
    streamResponse: boolean;
  };
}

export interface APIRequestModel {
  method: 'GET' | 'POST' | 'PUT' | 'DELETE';
  endpoint: string;
  headers: Record<string, string>;
  body?: any;
}

export interface APIResponseModel {
  status: number;
  data: any;
  error: string | null;
  timestamp: number;
}

// Validation functions for the models
export class ModelValidator {
  static validateQuestionRequest(request: QuestionRequestModel): { isValid: boolean; errors: string[] } {
    const errors: string[] = [];

    // Validate question
    if (!request.question || request.question.trim().length === 0) {
      errors.push('Question is required and cannot be empty');
    } else if (request.question.length > 1000) {
      errors.push('Question must be 1000 characters or less');
    }

    // Validate context
    if (!request.context) {
      errors.push('Context is required');
    } else {
      if (!request.context.current_page_url) {
        errors.push('Current page URL is required');
      }
      if (!request.context.current_page_title) {
        errors.push('Current page title is required');
      }
      if (request.context.selected_text && request.context.selected_text.length > 500) {
        errors.push('Selected text must be 500 characters or less');
      }
    }

    // Validate config if present
    if (request.config) {
      if (request.config.top_k && (request.config.top_k < 1 || request.config.top_k > 20)) {
        errors.push('top_k must be between 1 and 20');
      }
      if (request.config.relevance_threshold !== undefined) {
        if (request.config.relevance_threshold < 0.0 || request.config.relevance_threshold > 1.0) {
          errors.push('relevance_threshold must be between 0.0 and 1.0');
        }
      }
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }

  static validateAIResponse(response: AIResponseModel): { isValid: boolean; errors: string[] } {
    const errors: string[] = [];

    // Validate required fields
    if (!response.id) errors.push('ID is required');
    if (response.question === undefined) errors.push('Question is required');
    if (response.answer === undefined) errors.push('Answer is required');
    if (response.sources === undefined) errors.push('Sources are required');
    if (response.retrieved_chunks === undefined) errors.push('Retrieved chunks are required');
    if (response.relevance_score === undefined) errors.push('Relevance score is required');
    if (response.is_valid === undefined) errors.push('Valid status is required');
    if (response.execution_time === undefined) errors.push('Execution time is required');
    if (!response.validation_details) errors.push('Validation details are required');

    // Validate data types and ranges
    if (typeof response.relevance_score !== 'number' || response.relevance_score < 0.0 || response.relevance_score > 1.0) {
      errors.push('Relevance score must be a number between 0.0 and 1.0');
    }

    if (typeof response.is_valid !== 'boolean') {
      errors.push('Valid status must be a boolean');
    }

    if (typeof response.execution_time !== 'number' || response.execution_time < 0) {
      errors.push('Execution time must be a non-negative number');
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }
}