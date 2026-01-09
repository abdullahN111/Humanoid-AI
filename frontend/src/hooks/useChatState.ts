import { useState, useCallback } from 'react';
import { ChatState, QueryRequest, QueryResponse } from '../services/api/types';
import { agentService } from '../services/api/agentService';

export const useChatState = () => {
  const [chatState, setChatState] = useState<ChatState>({
    loading: false,
    error: undefined,
    response: undefined,
    sessionId: Math.random().toString(36).substring(2, 15),
  });

  const sendQuery = useCallback(async (request: QueryRequest): Promise<QueryResponse> => {
    try {
      setChatState(prev => ({
        ...prev,
        loading: true,
        error: undefined,
      }));

      const response = await agentService.sendQuery({
        ...request,
        sessionId: chatState.sessionId,
      });

      setChatState(prev => ({
        ...prev,
        loading: false,
        response,
      }));

      return response;
    } catch (error: any) {
      setChatState(prev => ({
        ...prev,
        loading: false,
        error: error.message || 'An error occurred while processing your query',
      }));
      throw error;
    }
  }, [chatState.sessionId]);

  const clearChat = useCallback(() => {
    setChatState({
      loading: false,
      error: undefined,
      response: undefined,
      sessionId: Math.random().toString(36).substring(2, 15),
    });
  }, []);

  const resetError = useCallback(() => {
    setChatState(prev => ({
      ...prev,
      error: undefined,
    }));
  }, []);

  return {
    chatState,
    sendQuery,
    clearChat,
    resetError,
  };
};