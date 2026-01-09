// API request/response types

// Request types
export interface QueryRequest {
  query: string;
  pageContext?: PageContext;
  selectedText?: string;
  sessionId?: string;
  retrieval_params?: {
    top_k?: number;
    threshold?: number;
  };
  grounding_required?: boolean;
}

export interface PageContext {
  url: string;
  title: string;
  section?: string;
  metadata?: Record<string, any>;
}

// Response types
export interface QueryResponse {
  id: string;
  answer: string;
  sources?: Source[];
  timestamp: string;
  sessionId: string;
}

export interface Source {
  title: string;
  url: string;
  content: string;
  score?: number;
}

// Error response type
export interface ErrorResponse {
  error: string;
  message?: string;
  code?: string;
}

// API communication state
export interface ChatState {
  loading: boolean;
  error?: string;
  response?: QueryResponse;
  sessionId: string;
}

// Context types
export interface ChatContextType {
  chatState: ChatState;
  sendQuery: (request: QueryRequest) => Promise<QueryResponse>;
  clearChat: () => void;
}