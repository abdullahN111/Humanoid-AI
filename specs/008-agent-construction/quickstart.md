# Quickstart Guide: Agent Construction

## Overview
This guide provides instructions for setting up and running the RAG-powered AI assistant that answers questions from the Physical AI & Humanoid Robotics book.

## Prerequisites
- Python 3.11+
- Poetry (for dependency management)
- Access to OpenAI API
- Access to Qdrant Cloud collection with book embeddings
- Access to Cohere API for embeddings (if not using OpenAI embeddings)

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies
```bash
cd backend
poetry install
```

### 3. Environment Configuration
Create a `.env` file in the backend directory with the following variables:
```env
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
COHERE_API_KEY=your_cohere_api_key  # Optional if using OpenAI embeddings
```

### 4. Run the Agent Service
```bash
cd backend
poetry run uvicorn src.api.main:app --host 0.0.0.0 --port 8000 --reload
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
Make a GET request to `/health` to check the service status.

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

### GET /health
Health check endpoint to verify service availability.

Response:
- `status` (string): Health status (healthy, degraded, unavailable)
- `timestamp` (string): When the health check was performed
- `details` (object): Additional health check details

## Development

### Running Tests
```bash
cd backend
poetry run pytest
```

### Adding New Features
1. Update the specification in `specs/008-agent-construction/spec.md`
2. Update the implementation plan in `specs/008-agent-construction/plan.md`
3. Create or update tests in the `tests/` directory
4. Implement the feature in the appropriate modules
5. Update documentation as needed

## Troubleshooting

### Common Issues
1. **API Keys Not Set**: Ensure all required API keys are set in the `.env` file
2. **Qdrant Connection Issues**: Verify QDRANT_URL and QDRANT_API_KEY are correct
3. **Rate Limits**: Check if API rate limits are being exceeded

### Debugging
Enable debug logging by setting the environment variable:
```env
LOG_LEVEL=DEBUG
```