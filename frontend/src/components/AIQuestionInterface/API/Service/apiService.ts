import { QuestionRequest, AIResponse } from './Models/types';

export interface APIConfig {
  baseUrl: string;
  timeout?: number;
  headers?: Record<string, string>;
}

export class APIService {
  private config: APIConfig;

  constructor(config: APIConfig) {
    this.config = {
      timeout: 30000, // 30 second default timeout
      headers: {
        'Content-Type': 'application/json',
      },
      ...config,
    };
  }

  async askQuestion(questionRequest: QuestionRequest, maxRetries: number = 3, sessionId?: string): Promise<AIResponse> {
    let lastError: Error | null = null;

    for (let attempt = 0; attempt <= maxRetries; attempt++) {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

      try {
        // Include session ID in the request if provided
        const requestPayload = sessionId
          ? { ...questionRequest, session_id: sessionId }
          : questionRequest;

        const response = await fetch(`${this.config.baseUrl}/api/v1/chat/ask`, {
          method: 'POST',
          headers: {
            ...this.config.headers,
            ...this.getAuthHeaders(),
          },
          body: JSON.stringify(requestPayload),
          signal: controller.signal,
        });

        clearTimeout(timeoutId);

        if (!response.ok) {
          throw new Error(`API request failed with status ${response.status}: ${response.statusText}`);
        }

        const data = await response.json();
        return data as AIResponse;
      } catch (error) {
        clearTimeout(timeoutId);

        if (error.name === 'AbortError') {
          lastError = new Error('Request timeout: The request took too long to complete');
        } else {
          lastError = error as Error;
        }

        // If this was the last attempt, throw the error
        if (attempt === maxRetries) {
          break;
        }

        // Wait before retrying (exponential backoff: 1s, 2s, 4s...)
        await this.sleep(Math.pow(2, attempt) * 1000);
      }
    }

    // If we get here, all retries have been exhausted
    throw lastError!;
  }

  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  async askQuestionStream(
    questionRequest: QuestionRequest,
    onMessage: (message: string) => void,
    onError?: (error: Error) => void,
    onCompletion?: () => void
  ): Promise<void> {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

    try {
      const response = await fetch(`${this.config.baseUrl}/api/v1/chat/stream-ask`, {
        method: 'POST',
        headers: {
          ...this.config.headers,
          ...this.getAuthHeaders(),
        },
        body: JSON.stringify(questionRequest),
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (!response.ok) {
        throw new Error(`Stream API request failed with status ${response.status}: ${response.statusText}`);
      }

      if (!response.body) {
        throw new Error('ReadableStream not supported in this environment');
      }

      const reader = response.body.getReader();
      const decoder = new TextDecoder();
      let buffer = '';

      try {
        while (true) {
          const { done, value } = await reader.read();

          if (done) {
            break;
          }

          buffer += decoder.decode(value, { stream: true });

          // Process the buffer for complete lines (SSE format)
          let boundary = buffer.indexOf('\n');
          while (boundary !== -1) {
            const line = buffer.substring(0, boundary).trim();
            buffer = buffer.substring(boundary + 1);

            if (line.startsWith('data: ')) {
              const data = line.substring(6); // Remove 'data: ' prefix

              if (data === '[DONE]') {
                if (onCompletion) onCompletion();
                break;
              }

              try {
                const parsed = JSON.parse(data);
                onMessage(parsed.content || parsed.text || parsed);
              } catch (e) {
                console.error('Error parsing stream data:', e);
                if (onError) onError(e as Error);
              }
            }

            boundary = buffer.indexOf('\n');
          }
        }
      } finally {
        reader.releaseLock();
      }

      if (onCompletion) onCompletion();
    } catch (error) {
      clearTimeout(timeoutId);

      if (error.name === 'AbortError') {
        const abortError = new Error('Stream request timeout: The request took too long to complete');
        if (onError) onError(abortError);
        throw abortError;
      }

      if (onError) onError(error as Error);
      throw error;
    }
  }

  async checkHealth(): Promise<boolean> {
    try {
      const response = await fetch(`${this.config.baseUrl}/health`, {
        method: 'GET',
        headers: this.config.headers,
      });

      return response.ok;
    } catch (error) {
      console.error('Health check failed:', error);
      return false;
    }
  }

  private getAuthHeaders(): Record<string, string> {
    // For now, we don't require authentication as per research.md
    // This can be extended to include API keys or tokens if needed
    return {};
  }
}