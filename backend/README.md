# Data Ingestion & Vector Storage Service

This service implements the data ingestion pipeline for the RAG-enabled Physical AI & Humanoid Robotics documentation system. It crawls the Docusaurus website, extracts content, generates vector embeddings, and stores them in Qdrant Cloud for semantic search.

## Features

- Sitemap-based crawling of Docusaurus website
- Content extraction and normalization
- Vector embedding generation using Cohere
- Storage in Qdrant Cloud with metadata
- Idempotent ingestion with deduplication

## Setup

1. Copy environment variables:
   ```bash
   cp .env.example .env
   # Update .env with your API keys and configuration
   ```

2. Install dependencies:
   ```bash
   uv sync
   ```

## Usage

Run the ingestion pipeline:
```bash
python scripts/ingest.py
```

Validate the ingestion:
```bash
python scripts/validate.py
```

## Project Structure

- `src/` - Source code modules
  - `config/` - Configuration management
  - `ingestion/` - Crawling, extraction, and processing
  - `embeddings/` - Embedding generation
  - `storage/` - Vector storage and data models
- `scripts/` - Entry point scripts
- `tests/` - Unit and integration tests
- `docs/` - Documentation