# API Contracts: Data Ingestion Service

## Overview
This document defines the API contracts for the data ingestion service that converts book content into vector embeddings for retrieval.

## Ingestion API

### POST /api/v1/ingest
Trigger the ingestion pipeline to crawl and process book content.

**Request**:
```json
{
  "sitemap_url": "https://humanoidai.vercel.app/sitemap.xml",
  "batch_size": 10,
  "force_reprocess": false
}
```

**Response (202)**:
```json
{
  "job_id": "uuid-string",
  "status": "started",
  "message": "Ingestion job started successfully",
  "estimated_completion": "2026-01-05T12:00:00Z"
}
```

**Response (400)**:
```json
{
  "error": "Invalid request parameters",
  "details": "Sitemap URL must be a valid URL"
}
```

### GET /api/v1/ingest/status/{job_id}
Check the status of an ingestion job.

**Response (200)**:
```json
{
  "job_id": "uuid-string",
  "status": "completed",
  "progress": {
    "total_urls": 100,
    "processed_urls": 85,
    "successful_chunks": 420,
    "failed_chunks": 2
  },
  "started_at": "2026-01-05T10:00:00Z",
  "completed_at": "2026-01-05T10:15:00Z",
  "message": "Ingestion completed successfully"
}
```

### DELETE /api/v1/ingest
Clear all vector database content (for reprocessing).

**Request**:
```json
{
  "confirmation": true
}
```

**Response (200)**:
```json
{
  "status": "success",
  "message": "Vector database cleared successfully"
}
```

**Response (400)**:
```json
{
  "error": "Confirmation required",
  "details": "Set confirmation to true to clear the database"
}
```

## Health Check API

### GET /api/v1/health
Check the health status of the ingestion service.

**Response (200)**:
```json
{
  "status": "healthy",
  "timestamp": "2026-01-05T10:00:00Z",
  "services": {
    "qdrant_connection": "connected",
    "cohere_api": "available",
    "web_crawler": "ready"
  }
}
```

## Configuration API

### GET /api/v1/config
Get current configuration of the ingestion service.

**Response (200)**:
```json
{
  "config": {
    "sitemap_url": "https://humanoidai.vercel.app/sitemap.xml",
    "batch_size": 10,
    "chunk_size": 1000,
    "chunk_overlap": 200,
    "embedding_model": "cohere/embed-english-v3.0",
    "qdrant_collection": "book_content_chunks"
  }
}
```

### PUT /api/v1/config
Update configuration of the ingestion service.

**Request**:
```json
{
  "config": {
    "batch_size": 15,
    "chunk_size": 1200
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