// Helper functions for the AI chat interface

/**
 * Generates a unique ID for chat sessions
 */
export const generateSessionId = (): string => {
  return Math.random().toString(36).substring(2, 15) + Date.now().toString(36);
};

/**
 * Sanitizes user input to prevent XSS and other security issues
 */
export const sanitizeInput = (input: string): string => {
  if (!input || typeof input !== 'string') {
    return '';
  }

  // Remove any HTML tags that might be injected
  return input
    .replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '')
    .replace(/<iframe\b[^<]*(?:(?!<\/iframe>)<[^<]*)*<\/iframe>/gi, '')
    .replace(/javascript:/gi, '')
    .trim();
};

/**
 * Validates if a query is valid (not empty, not just whitespace)
 */
export const isValidQuery = (query: string): boolean => {
  return query && query.trim().length > 0 && query.trim().length <= 1000;
};

/**
 * Formats a timestamp to a readable format
 */
export const formatTimestamp = (date: Date | string): string => {
  if (typeof date === 'string') {
    date = new Date(date);
  }
  return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
};

/**
 * Formats the response from the AI agent for display
 */
export const formatResponse = (response: string): string => {
  // Add any formatting needed for the response
  return response || 'No response received.';
};

/**
 * Debounces a function call
 */
export const debounce = <T extends (...args: any[]) => any>(func: T, wait: number) => {
  let timeout: NodeJS.Timeout | null = null;

  return function executedFunction(...args: Parameters<T>): void {
    if (timeout) {
      clearTimeout(timeout);
    }

    timeout = setTimeout(() => func.apply(this, args), wait);
  };
};

/**
 * Sleep function to introduce delays
 */
export const sleep = (ms: number): Promise<void> => {
  return new Promise(resolve => setTimeout(resolve, ms));
};

/**
 * Checks if the user is on a mobile device
 */
export const isMobileDevice = (): boolean => {
  return /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
};

/**
 * Extracts selected text from the current page
 */
export const getSelectedText = (): string => {
  if (typeof window !== 'undefined') {
    const selection = window.getSelection();
    return selection ? selection.toString().trim() : '';
  }
  return '';
};