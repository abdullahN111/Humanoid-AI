# RAG Agent API Documentation

## Overview

The RAG Agent API provides a service for querying the Physical AI & Humanoid Robotics book content using natural language queries. The API uses retrieval-augmented generation (RAG) to provide accurate, grounded responses based on the book's content.

## Base URL

```
http://localhost:8000/api
```

## Endpoints

### Query Agent

Submit a query to the RAG agent to retrieve information from the book content.

#### Request

```
POST /api/agent/query
```

**Headers:**
- `Content-Type: application/json`

**Request Body:**
```json
{
  "query": "string (required) - The question or query to ask the agent",
  "user_context": {
    "optional": "object - Additional context about the user or session"
  },
  "retrieval_params": {
    "top_k": "integer - Number of top results to retrieve (default: 5)",
    "threshold": "float - Minimum similarity threshold (default: 0.7)"
  },
  "grounding_required": "boolean - Whether strict grounding is required (default: true)"
}
```

**Example Request:**
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

#### Response

**Success Response (200):**
```json
{
  "success": true,
  "answer": "string - The agent's response to the query",
  "sources": [
    {
      "id": "string - Unique identifier for the content chunk",
      "content": "string - The content from the book (truncated)",
      "source_url": "string - URL of the original source",
      "page_title": "string - Title of the page/chapter",
      "similarity_score": "float - How similar this content is to the query",
      "chunk_order": "integer - Order of this chunk in the original document",
      "retrieval_rank": "integer - Rank of this chunk in the retrieval results"
    }
  ],
  "timestamp": "string - ISO formatted timestamp",
  "query_id": "string - Unique identifier for this query"
}
```

**Error Response (400, 500):**
```json
{
  "success": false,
  "answer": "string - Empty or error message",
  "sources": [],
  "timestamp": "string - ISO formatted timestamp",
  "query_id": "string - Unique identifier for this query",
  "error": "string - Error message describing the issue"
}
```

### Health Check

Check the health status of the API service.

#### Request

```
GET /api/health
```

#### Response

**Success Response (200):**
```json
{
  "status": "string - Health status (healthy, degraded, unavailable)",
  "timestamp": "string - ISO formatted timestamp",
  "response_time_ms": "number - Response time in milliseconds",
  "details": {
    "config": "string - Configuration accessibility status",
    "database": "string - Database connection status",
    "vector_db": "string - Vector database connection status"
  }
}
```

## Environment Variables

The API requires the following environment variables to be set:

- `OPENAI_API_KEY`: API key for OpenAI services
- `QDRANT_URL`: URL for the Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant access
- `QDRANT_COLLECTION_NAME`: Name of the Qdrant collection (default: book_content)
- `COHERE_API_KEY`: API key for Cohere embedding services
- `GEMINI_API_KEY`: API key for Google Gemini services (optional)
- `GEMINI_BASE_URL`: Base URL for Gemini API (optional)
- `MAX_RETRIEVED_CHUNKS`: Maximum number of chunks to retrieve (default: 5)
- `RETRIEVAL_THRESHOLD`: Minimum similarity threshold (default: 0.7)
- `GROUNDING_REQUIRED`: Whether strict grounding is required (default: true)
- `API_PREFIX`: API prefix (default: /api)
- `DEBUG`: Enable debug mode (default: false)

## Error Handling

The API follows standard HTTP status codes:

- `200`: Success
- `400`: Bad request (invalid input)
- `422`: Validation error (malformed request)
- `500`: Internal server error

## Rate Limiting

The API is designed to handle concurrent requests but may implement rate limiting in production environments.

## Security

- All API keys should be stored securely and never exposed in client-side code
- Implement proper authentication for production use
- Use HTTPS in production environments