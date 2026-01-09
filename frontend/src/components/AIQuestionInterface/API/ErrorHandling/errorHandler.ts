export interface APIError {
  message: string;
  status?: number;
  details?: any;
}

export class APIErrorHandler {
  static handle(error: any): APIError {
    if (error.name === 'AbortError' || error.message.includes('timeout')) {
      return {
        message: 'Request timeout: The server took too long to respond. Please try again.',
        status: 408,
      };
    }

    if (error.message.includes('NetworkError') || error.message.includes('Failed to fetch')) {
      return {
        message: 'Network error: Unable to connect to the server. Please check your connection.',
        status: 0,
      };
    }

    if (error.status) {
      switch (error.status) {
        case 400:
          return {
            message: 'Bad request: The request was invalid. Please check your input.',
            status: 400,
            details: error,
          };
        case 401:
          return {
            message: 'Unauthorized: Please check your authentication credentials.',
            status: 401,
          };
        case 403:
          return {
            message: 'Forbidden: You do not have permission to access this resource.',
            status: 403,
          };
        case 404:
          return {
            message: 'Not found: The requested resource was not found.',
            status: 404,
          };
        case 429:
          return {
            message: 'Too many requests: Please wait before making another request.',
            status: 429,
          };
        case 500:
          return {
            message: 'Internal server error: Something went wrong on the server. Please try again later.',
            status: 500,
          };
        case 502:
          return {
            message: 'Bad gateway: The server is temporarily unavailable. Please try again later.',
            status: 502,
          };
        case 503:
          return {
            message: 'Service unavailable: The server is currently unavailable. Please try again later.',
            status: 503,
          };
        default:
          return {
            message: `Server error (${error.status}): ${error.message || 'An unknown error occurred'}`,
            status: error.status,
          };
      }
    }

    return {
      message: error.message || 'An unknown error occurred',
      details: error,
    };
  }

  static logError(error: APIError, context?: string): void {
    const timestamp = new Date().toISOString();
    console.error(`[${timestamp}] API Error${context ? ` in ${context}` : ''}:`, {
      message: error.message,
      status: error.status,
      details: error.details,
    });
  }
}

export class ValidationError extends Error {
  constructor(message: string, public field?: string) {
    super(message);
    this.name = 'ValidationError';
  }
}

export class ValidationHelper {
  static validateQuestionRequest(request: any): string[] {
    const errors: string[] = [];

    if (!request.question || typeof request.question !== 'string' || request.question.trim().length === 0) {
      errors.push('Question is required and must be a non-empty string');
    }

    if (request.question && request.question.length > 1000) {
      errors.push('Question must be less than 1000 characters');
    }

    if (!request.context) {
      errors.push('Context is required');
    } else {
      if (!request.context.current_page_url) {
        errors.push('Current page URL is required in context');
      }

      if (!request.context.current_page_title) {
        errors.push('Current page title is required in context');
      }
    }

    if (request.config) {
      if (request.config.top_k && (request.config.top_k < 1 || request.config.top_k > 20)) {
        errors.push('top_k must be between 1 and 20');
      }

      if (request.config.relevance_threshold &&
          (request.config.relevance_threshold < 0.0 || request.config.relevance_threshold > 1.0)) {
        errors.push('relevance_threshold must be between 0.0 and 1.0');
      }
    }

    return errors;
  }

  static validateString(value: string, fieldName: string, minLength: number = 1, maxLength: number = 1000): string | null {
    if (typeof value !== 'string') {
      return `${fieldName} must be a string`;
    }

    if (value.length < minLength) {
      return `${fieldName} must be at least ${minLength} character(s)`;
    }

    if (value.length > maxLength) {
      return `${fieldName} must be no more than ${maxLength} characters`;
    }

    return null;
  }
}