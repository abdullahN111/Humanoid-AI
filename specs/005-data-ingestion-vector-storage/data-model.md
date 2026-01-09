# Data Models: Data Ingestion & Vector Storage

**Feature**: 005-data-ingestion-vector-storage
**Created**: 2026-01-05
**Status**: Complete

## Overview

This document defines the data models for the data ingestion and vector storage system. These models represent the core entities that will be processed, stored, and retrieved by the system.

## Core Entities

### 1. ContentChunk

**Description**: A segment of extracted text from a book page with associated metadata.

**Fields**:
- `id` (string): Unique identifier for the chunk (UUID)
- `content` (string): The extracted and normalized text content
- `source_url` (string): The URL of the original page
- `page_title` (string): Title of the source page
- `content_hash` (string): Hash of the content for deduplication
- `ingestion_timestamp` (datetime): Time when the content was ingested
- `chunk_order` (integer): Order of this chunk within the source document
- `metadata` (object): Additional metadata about the content

**Validation Rules**:
- `content` must not be empty
- `source_url` must be a valid URL
- `content_hash` must be unique per `source_url`
- `chunk_order` must be non-negative

**Relationships**:
- One-to-many with VectorEmbedding (one chunk can have multiple embeddings for different models)

### 2. VectorEmbedding

**Description**: A numerical representation of content chunk text with metadata.

**Fields**:
- `id` (string): Unique identifier for the embedding (UUID)
- `chunk_id` (string): Reference to the associated ContentChunk
- `vector` (array[float]): The embedding vector values
- `model_name` (string): Name of the embedding model used
- `model_version` (string): Version of the embedding model
- `vector_size` (integer): Dimension of the embedding vector
- `created_at` (datetime): Time when the embedding was created

**Validation Rules**:
- `vector` must have the correct dimension for the specified model
- `chunk_id` must reference an existing ContentChunk
- `vector_size` must match the expected size for the model

**Relationships**:
- Many-to-one with ContentChunk (multiple embeddings per chunk possible)
- Embedded within Qdrant payload

### 3. SourceReference

**Description**: Metadata linking embeddings back to the original book page.

**Fields**:
- `chunk_id` (string): Reference to the ContentChunk
- `source_url` (string): The URL of the original page
- `page_title` (string): Title of the source page
- `section_title` (string, optional): Title of the specific section
- `content_preview` (string): Short preview of the content
- `created_at` (datetime): Time when the reference was created

**Validation Rules**:
- `source_url` must be a valid URL
- `content_preview` should be limited to 200 characters
- `chunk_id` must reference an existing ContentChunk

**Relationships**:
- One-to-one with ContentChunk
- Embedded within Qdrant payload for retrieval

### 4. IngestionRecord

**Description**: Tracks ingestion runs and their status.

**Fields**:
- `id` (string): Unique identifier for the ingestion run (UUID)
- `start_time` (datetime): When the ingestion started
- `end_time` (datetime): When the ingestion completed
- `status` (string): Status of the ingestion (pending, running, completed, failed)
- `processed_urls` (integer): Number of URLs processed
- `successful_chunks` (integer): Number of chunks successfully created
- `failed_chunks` (integer): Number of chunks that failed
- `error_details` (object, optional): Details about any errors

**Validation Rules**:
- `status` must be one of the allowed values
- `processed_urls` must be non-negative
- `start_time` must be before `end_time` (when both present)

**Relationships**:
- One-to-many with ContentChunk (via tracking which chunks were created in which run)

## Qdrant Collection Schema

### Collection Name: `book_content_chunks`

**Vector Configuration**:
- `vector_name`: "content_embedding"
- `size`: 1024 (for Cohere embed-english-v3.0)
- `distance`: "Cosine"

**Payload Schema**:
```json
{
  "chunk_id": "keyword",
  "content": "text",
  "source_url": "keyword",
  "page_title": "text",
  "content_hash": "keyword",
  "ingestion_timestamp": "datetime",
  "chunk_order": "integer",
  "metadata": "object"
}
```

**Indexes**:
- Index on `source_url` for efficient lookups by URL
- Index on `content_hash` for deduplication
- Index on `ingestion_timestamp` for time-based queries

## Data Flow

1. **Source Data**: Web pages from https://humanoidai.vercel.app/
2. **Processing**: Content extraction and normalization
3. **Chunking**: Text splitting into ContentChunk entities
4. **Embedding**: Generation of VectorEmbedding from ContentChunk
5. **Storage**: Storage in Qdrant with SourceReference metadata
6. **Retrieval**: Search by semantic similarity with metadata

## Constraints and Guarantees

1. **Uniqueness**: Content chunks are deduplicated using content hash
2. **Completeness**: All source references are preserved
3. **Consistency**: Embeddings are generated consistently for the same content
4. **Availability**: Vector database is available for real-time retrieval
5. **Durability**: Data is persisted and backed up appropriately