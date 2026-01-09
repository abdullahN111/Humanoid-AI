import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { AIQuestionInterface } from '../AIQuestionInterface';

// Mock the API service
jest.mock('../API/Service/apiService', () => ({
  APIService: jest.fn().mockImplementation(() => ({
    askQuestion: jest.fn()
  }))
}));

describe('Error Handling and Loading States', () => {
  let mockAPIService: any;

  beforeEach(() => {
    mockAPIService = require('../API/Service/apiService').APIService.mock.instances[0];
  });

  test('shows loading state during API request', async () => {
    // Mock a delayed response
    mockAPIService.askQuestion.mockImplementation(() =>
      new Promise(resolve => setTimeout(() => resolve({
        answer: 'Test response',
        sources: ['https://example.com']
      }), 100))
    );

    render(<AIQuestionInterface />);

    const questionInput = screen.getByPlaceholderText('Ask a question about this documentation...');
    const submitButton = screen.getByText('Ask AI');

    fireEvent.change(questionInput, { target: { value: 'Test question' } });
    fireEvent.click(submitButton);

    // Loading state should appear immediately
    expect(screen.getByText('Asking...')).toBeInTheDocument();

    // Wait for response to complete
    await waitFor(() => {
      expect(screen.getByText('Test response')).toBeInTheDocument();
    });
  });

  test('shows error when API request fails', async () => {
    mockAPIService.askQuestion.mockRejectedValue(new Error('Network error'));

    render(<AIQuestionInterface />);

    const questionInput = screen.getByPlaceholderText('Ask a question about this documentation...');
    const submitButton = screen.getByText('Ask AI');

    fireEvent.change(questionInput, { target: { value: 'Test question' } });
    fireEvent.click(submitButton);

    // Wait for error to appear
    await waitFor(() => {
      expect(screen.getByText('Network error')).toBeInTheDocument();
    });
  });

  test('shows timeout error when request takes too long', async () => {
    mockAPIService.askQuestion.mockRejectedValue(new Error('Request timeout: The request took too long to complete'));

    render(<AIQuestionInterface />);

    const questionInput = screen.getByPlaceholderText('Ask a question about this documentation...');
    const submitButton = screen.getByText('Ask AI');

    fireEvent.change(questionInput, { target: { value: 'Test question' } });
    fireEvent.click(submitButton);

    // Wait for timeout error to appear
    await waitFor(() => {
      expect(screen.getByText('Request timeout: The request took too long to complete')).toBeInTheDocument();
    });
  });

  test('retry button appears when error occurs', async () => {
    mockAPIService.askQuestion.mockRejectedValue(new Error('API Error'));

    render(<AIQuestionInterface />);

    const questionInput = screen.getByPlaceholderText('Ask a question about this documentation...');
    const submitButton = screen.getByText('Ask AI');

    fireEvent.change(questionInput, { target: { value: 'Test question' } });
    fireEvent.click(submitButton);

    // Wait for error and retry button to appear
    await waitFor(() => {
      expect(screen.getByText('API Error')).toBeInTheDocument();
      expect(screen.getByText('Retry')).toBeInTheDocument();
    });
  });

  test('retry button works after error', async () => {
    // First call fails, second call succeeds
    mockAPIService.askQuestion
      .mockRejectedValueOnce(new Error('API Error'))
      .mockResolvedValueOnce({
        answer: 'Retry response',
        sources: ['https://example.com']
      });

    render(<AIQuestionInterface />);

    const questionInput = screen.getByPlaceholderText('Ask a question about this documentation...');
    const submitButton = screen.getByText('Ask AI');

    // Initial submission
    fireEvent.change(questionInput, { target: { value: 'Test question' } });
    fireEvent.click(submitButton);

    // Wait for error
    await waitFor(() => {
      expect(screen.getByText('API Error')).toBeInTheDocument();
    });

    // Click retry
    const retryButton = screen.getByText('Retry');
    fireEvent.click(retryButton);

    // Wait for success after retry
    await waitFor(() => {
      expect(screen.getByText('Retry response')).toBeInTheDocument();
    });
  });

  test('loading state is disabled after error', async () => {
    mockAPIService.askQuestion.mockRejectedValue(new Error('API Error'));

    render(<AIQuestionInterface />);

    const questionInput = screen.getByPlaceholderText('Ask a question about this documentation...');
    const submitButton = screen.getByText('Ask AI');

    fireEvent.change(questionInput, { target: { value: 'Test question' } });
    fireEvent.click(submitButton);

    // Wait for error to appear
    await waitFor(() => {
      expect(screen.getByText('API Error')).toBeInTheDocument();
    });

    // Submit button should be enabled again after error
    expect(submitButton).not.toBeDisabled();
  });
});