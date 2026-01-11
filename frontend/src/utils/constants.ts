// API endpoint constants
// Docusaurus uses window.ENV for environment variables
const getAPIBaseUrl = (): string => {

  // Fallback to environment variables if available during build time
  if (typeof process !== 'undefined' && process.env) {
    return process.env.REACT_APP_API_BASE_URL
  }
  return
};

export const API_BASE_URL = getAPIBaseUrl();
export const API_ENDPOINTS = {
  QUERY: `${API_BASE_URL}/api/agent/query`,
  HEALTH: `${API_BASE_URL}/health`,
};

// Configuration constants
export const CHAT_CONFIG = {
  MAX_RETRIES: 3,
  TIMEOUT_MS: 30000,
  DEBOUNCE_MS: 300,
};

// Error messages
export const ERROR_MESSAGES = {
  NETWORK_ERROR: 'Network error occurred. Please check your connection.',
  TIMEOUT_ERROR: 'Request timed out. Please try again.',
  INVALID_QUERY: 'Please enter a valid question.',
  EMPTY_RESPONSE: 'No response received from the server.',
};