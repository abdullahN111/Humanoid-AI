# RAG Agent API Quickstart Guide

## Overview
This guide provides instructions for setting up and running the RAG-powered AI assistant that answers questions from the Physical AI & Humanoid Robotics book.

## Prerequisites
- Python 3.11+
- Access to OpenAI API (or Google Gemini API)
- Access to Qdrant Cloud collection with book embeddings
- Access to Cohere API for embeddings

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies
```bash
cd backend
pip install -r requirements.txt
# Or if using poetry:
poetry install
```

### 3. Environment Configuration
Create a `.env` file in the backend directory with the required variables:

```env
# OpenAI Configuration
OPENAI_API_KEY=your_openai_api_key_here

# Qdrant Configuration
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=book_content

# Cohere Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Google Gemini Configuration (for external LLM in OpenAI Agent SDK) - Optional
GEMINI_API_KEY=your_gemini_api_key_here
GEMINI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/

# Agent Configuration
MAX_RETRIEVED_CHUNKS=5
RETRIEVAL_THRESHOLD=0.7
GROUNDING_REQUIRED=true

# API Configuration
API_PREFIX=/api
DEBUG=false
```

### 4. Run the Agent Service
```bash
cd backend
python -m uvicorn src.api.main:app --host 0.0.0.0 --port 8000 --reload
```

Or run the main application directly:
```bash
cd backend
python src/api/main.py
```

## Usage

### Query the Agent
Make a POST request to `/api/agent/query` with a JSON payload:
```json
{
  "query": "What are the key components of humanoid robotics?",
  "user_context": {},
  "retrieval_params": {
    "top_k": 5,
    "threshold": 0.7
  },
  "grounding_required": true
}
```

### Health Check
Make a GET request to `/api/health` to check the service status.

## API Endpoints

### POST /api/agent/query
Submit a query to the RAG agent.

Request body:
- `query` (string, required): The user's query text
- `user_context` (object, optional): Context about the user or session
- `retrieval_params` (object, optional): Parameters for the retrieval process
- `grounding_required` (boolean): Whether strict grounding is required (default: true)

Response:
- `success` (boolean): Whether the request was successful
- `answer` (string): The agent's answer to the query
- `sources` (array): List of sources used to generate the answer
- `timestamp` (string): When the response was generated
- `query_id` (string): ID of the original query
- `error` (string, optional): Error message if the request failed

### GET /api/health
Health check endpoint to verify service availability.

Response:
- `status` (string): Health status (healthy, degraded, unavailable)
- `timestamp` (string): When the health check was performed
- `response_time_ms` (number): Response time in milliseconds
- `details` (object): Additional health check details

## Validation

To validate that the service is working properly:

1. Start the service
2. Make a test query to `/api/agent/query` with a simple question
3. Verify that you receive a response with sources from the book content
4. Check the health endpoint at `/api/health`

Example test query:
```bash
curl -X POST http://localhost:8000/api/agent/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key components of humanoid robotics?",
    "grounding_required": true
  }'
```

## Troubleshooting

### Common Issues
1. **API Keys Not Set**: Ensure all required API keys are set in the `.env` file
2. **Qdrant Connection Issues**: Verify QDRANT_URL and QDRANT_API_KEY are correct
3. **Rate Limits**: Check if API rate limits are being exceeded

### Debugging
Enable debug logging by setting the environment variable:
```env
DEBUG=true
```

## Architecture

The RAG Agent follows this flow:
1. **Query**: User submits a question
2. **Retrieve**: System retrieves relevant book content from Qdrant based on semantic similarity
3. **Generate**: LLM generates a response based on the retrieved content
4. **Validate**: Response is validated for grounding in the provided content
5. **Respond**: Answer is returned with citations to source material