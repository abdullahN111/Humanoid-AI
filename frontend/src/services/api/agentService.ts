import { QueryRequest, QueryResponse, ErrorResponse } from './types';
import { apiClient } from './apiClient';
import { ERROR_MESSAGES } from '../../utils/constants';

class AgentService {
  async sendQuery(request: QueryRequest): Promise<QueryResponse> {
    try {
      const response = await apiClient.post<QueryResponse>('/api/v1/chat/ask', request);
      return response.data;
    } catch (error: any) {
      if (error.response) {
        // Server responded with error status
        const errorResponse: ErrorResponse = error.response.data;
        throw new Error(errorResponse.error || 'Failed to get response from agent');
      } else if (error.request) {
        // Request was made but no response received
        throw new Error(ERROR_MESSAGES.NETWORK_ERROR);
      } else {
        // Something else happened
        throw new Error(error.message || 'An error occurred while sending the query');
      }
    }
  }

  async healthCheck(): Promise<boolean> {
    try {
      const response = await apiClient.get('/health');
      return response.status === 200;
    } catch (error) {
      console.error('Health check failed:', error);
      return false;
    }
  }
}

export const agentService = new AgentService();
export default agentService;