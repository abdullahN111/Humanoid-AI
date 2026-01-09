# API Contracts: Retrieval Validation Service

## Overview
This document defines the API contracts for the retrieval validation service that validates the RAG system's ability to accurately return relevant book content from the vector database.

## Validation API

### POST /api/v1/validation/query
Submit a query for validation against the retrieval system.

**Request**:
```json
{
  "query_text": "What are the key components of humanoid robotics?",
  "metadata_filters": {
    "url": "https://humanoidai.vercel.app/docs/components",
    "section": "robotics-fundamentals"
  },
  "expected_results_count": 5
}
```

**Response (200)**:
```json
{
  "validation_id": "uuid-string",
  "query_text": "What are the key components of humanoid robotics?",
  "retrieved_chunks": [
    {
      "id": "chunk-uuid",
      "content": "The key components of humanoid robotics include...",
      "source_url": "https://humanoidai.vercel.app/docs/components",
      "similarity_score": 0.85,
      "retrieval_rank": 1
    }
  ],
  "validation_result": {
    "is_relevant": true,
    "relevance_score": 0.92,
    "accuracy_metrics": {
      "precision": 0.8,
      "recall": 0.75,
      "f1_score": 0.77
    }
  },
  "timestamp": "2026-01-05T10:00:00Z"
}
```

**Response (400)**:
```json
{
  "error": "Invalid request parameters",
  "details": "Query text is required"
}
```

### POST /api/v1/validation/batch
Submit a batch of queries for validation.

**Request**:
```json
{
  "queries": [
    {
      "query_text": "What are the key components of humanoid robotics?",
      "expected_results_count": 5
    },
    {
      "query_text": "Explain the control systems for humanoid robots",
      "expected_results_count": 3
    }
  ],
  "options": {
    "batch_size": 10,
    "similarity_threshold": 0.7
  }
}
```

**Response (200)**:
```json
{
  "batch_id": "uuid-string",
  "total_queries": 2,
  "completed_queries": 2,
  "validation_results": [
    {
      "query_id": "query-uuid-1",
      "relevance_score": 0.92,
      "retrieved_count": 5,
      "expected_count": 5,
      "accuracy_metrics": {
        "precision": 0.8,
        "recall": 0.75,
        "f1_score": 0.77
      }
    }
  ],
  "summary_metrics": {
    "average_precision": 0.82,
    "average_recall": 0.76,
    "average_f1_score": 0.78
  },
  "timestamp": "2026-01-05T10:00:00Z"
}
```

### GET /api/v1/validation/results/{validation_id}
Get the results of a specific validation.

**Response (200)**:
```json
{
  "validation_id": "uuid-string",
  "query_text": "What are the key components of humanoid robotics?",
  "retrieved_chunks": [
    {
      "id": "chunk-uuid",
      "content": "The key components of humanoid robotics include...",
      "source_url": "https://humanoidai.vercel.app/docs/components",
      "similarity_score": 0.85,
      "retrieval_rank": 1
    }
  ],
  "validation_result": {
    "is_relevant": true,
    "relevance_score": 0.92,
    "accuracy_metrics": {
      "precision": 0.8,
      "recall": 0.75,
      "f1_score": 0.77
    }
  },
  "validation_notes": "Query returned highly relevant results",
  "timestamp": "2026-01-05T10:00:00Z"
}
```

## Metrics API

### GET /api/v1/validation/metrics
Get overall validation metrics.

**Response (200)**:
```json
{
  "metrics": {
    "precision": 0.82,
    "recall": 0.76,
    "f1_score": 0.78,
    "mrr": 0.85,
    "hit_rate": 0.94,
    "total_queries": 100,
    "total_relevant_retrieved": 76,
    "total_expected_retrieved": 80
  },
  "report_period": {
    "start_date": "2026-01-01T00:00:00Z",
    "end_date": "2026-01-05T23:59:59Z"
  },
  "last_updated": "2026-01-05T10:00:00Z"
}
```

## Configuration API

### GET /api/v1/validation/config
Get current configuration of the validation service.

**Response (200)**:
```json
{
  "config": {
    "similarity_threshold": 0.7,
    "max_retrieval_count": 10,
    "validation_batch_size": 10,
    "qdrant_collection": "book_content_chunks"
  }
}
```

### PUT /api/v1/validation/config
Update configuration of the validation service.

**Request**:
```json
{
  "config": {
    "similarity_threshold": 0.75,
    "max_retrieval_count": 5
  }
}
```

**Response (200)**:
```json
{
  "status": "updated",
  "message": "Configuration updated successfully"
}
```

## Health Check API

### GET /api/v1/validation/health
Check the health status of the validation service.

**Response (200)**:
```json
{
  "status": "healthy",
  "timestamp": "2026-01-05T10:00:00Z",
  "services": {
    "qdrant_connection": "connected",
    "validation_engine": "ready",
    "test_dataset": "loaded"
  }
}
```