// Data models for frontend-backend communication

export interface QuestionRequest {
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

export interface AIResponse {
  id: string;
  question: string;
  answer: string;
  sources: string[];
  retrieved_chunks: RetrievedChunk[];
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

export interface RetrievedChunk {
  id: string;
  chunk_text: string;
  similarity_score: number;
  source_url: string;
  source_title: string;
  section_hierarchy?: string;
  rank: number;
  content_hash: string;
}

export interface SessionResponse {
  session_id: string;
  status: string;
  created_at: number;
  conversation_history_length: number;
  context: Record<string, any>;
}

export interface SessionHistoryResponse {
  session_id: string;
  history: Array<{
    query: string;
    response_id: string;
    timestamp: number;
    relevance_score: number;
    is_valid: boolean;
  }>;
  total_entries: number;
}

export interface HealthResponse {
  status: string;
  timestamp: number;
  response_time: number;
  services: {
    qdrant: {
      status: string;
      response_time?: number;
    };
    cohere: {
      status: string;
      response_time?: number;
    };
  };
}

export interface ErrorResponse {
  detail: string;
}

export interface APIRequest {
  method: 'GET' | 'POST' | 'PUT' | 'DELETE';
  endpoint: string;
  headers: Record<string, string>;
  body?: any;
}

export interface APIResponse {
  status: number;
  data: any;
  error: string | null;
  timestamp: number;
}