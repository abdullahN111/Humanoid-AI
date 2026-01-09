import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { AIQuestionInterface } from '../AIQuestionInterface';

// Mock the API service
jest.mock('../API/Service/apiService', () => ({
  APIService: jest.fn().mockImplementation(() => ({
    askQuestion: jest.fn().mockResolvedValue({
      answer: 'This is a test response',
      sources: ['https://example.com/doc1', 'https://example.com/doc2'],
      validation_details: {
        grounding_validation_passed: true,
        hallucination_detected: false,
        relevance_score: 0.85
      },
      retrieved_chunks: [
        {
          chunk_text: 'This is a sample chunk of text from the documentation',
          source_title: 'Sample Document',
          similarity_score: 0.85
        }
      ]
    })
  }))
}));

describe('AIQuestionInterface', () => {
  test('submits question and displays response', async () => {
    render(<AIQuestionInterface />);

    // Find the question input and submit button
    const questionInput = screen.getByPlaceholderText('Ask a question about this documentation...');
    const submitButton = screen.getByText('Ask AI');

    // Enter a question
    fireEvent.change(questionInput, { target: { value: 'What is ROS 2?' } });

    // Click the submit button
    fireEvent.click(submitButton);

    // Wait for the loading state to appear
    expect(screen.getByText('Asking...')).toBeInTheDocument();

    // Wait for the response to appear
    await waitFor(() => {
      expect(screen.getByText('This is a test response')).toBeInTheDocument();
    });

    // Check that sources are displayed
    expect(screen.getByText('Sources:')).toBeInTheDocument();
    expect(screen.getByText('https://example.com/doc1')).toBeInTheDocument();
    expect(screen.getByText('https://example.com/doc2')).toBeInTheDocument();
  });

  test('shows error when question is empty', () => {
    render(<AIQuestionInterface />);

    const submitButton = screen.getByText('Ask AI');

    // Click the submit button without entering a question
    fireEvent.click(submitButton);

    // Check that error message is displayed
    expect(screen.getByText('Please enter a question')).toBeInTheDocument();
  });

  test('handles API errors gracefully', async () => {
    // Mock API service to reject
    const mockAPIService = require('../API/Service/apiService');
    mockAPIService.APIService.mockImplementation(() => ({
      askQuestion: jest.fn().mockRejectedValue(new Error('API Error'))
    }));

    render(<AIQuestionInterface />);

    const questionInput = screen.getByPlaceholderText('Ask a question about this documentation...');
    const submitButton = screen.getByText('Ask AI');

    fireEvent.change(questionInput, { target: { value: 'Test question' } });
    fireEvent.click(submitButton);

    // Wait for the error to appear
    await waitFor(() => {
      expect(screen.getByText('API Error')).toBeInTheDocument();
    });
  });
});