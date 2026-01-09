# Quickstart: Data Ingestion & Vector Storage

**Feature**: 005-data-ingestion-vector-storage
**Created**: 2026-01-05
**Status**: Complete

## Overview

This quickstart guide provides instructions for setting up and running the data ingestion pipeline that converts book content into vector embeddings for retrieval.

## Prerequisites

- Python 3.9 or higher
- `uv` package manager (install with `pip install uv`)
- Access to the root `.env` file with required API keys

## Setup Instructions

### 1. Create Backend Directory and Initialize Project

```bash
# Create backend directory
mkdir backend
cd backend

# Initialize Python project with uv
uv init .

# Set up project structure
mkdir -p src/{config,ingestion,embeddings,storage}
mkdir scripts tests docs
```

### 2. Add Dependencies

```bash
# Navigate to backend directory
cd backend

# Add required dependencies using uv
uv add requests beautifulsoup4 lxml cohere qdrant-client python-dotenv
uv add pytest black mypy (for development)
```

### 3. Environment Variables

Copy the required environment variables from the root `.env` file:

```bash
# Create .env file in backend directory
cp ../.env .env.example
cp ../.env .env  # Or copy specific variables to new .env file

# Required variables:
# QDRANT_URL=your_qdrant_cloud_url
# QDRANT_API_KEY=your_qdrant_api_key
# COHERE_API_KEY=your_cohere_api_key
```

### 4. Project Structure

The backend project follows this structure:

```
backend/
├── pyproject.toml          # Project configuration
├── .env                    # Environment variables
├── .env.example            # Example environment variables
├── src/
│   ├── __init__.py
│   ├── main.py
│   ├── config/
│   │   ├── __init__.py
│   │   └── settings.py
│   ├── ingestion/
│   │   ├── __init__.py
│   │   ├── crawler.py
│   │   ├── extractor.py
│   │   ├── processor.py
│   │   └── pipeline.py
│   ├── embeddings/
│   │   ├── __init__.py
│   │   ├── generator.py
│   │   └── client.py
│   └── storage/
│       ├── __init__.py
│       ├── vector_db.py
│       └── models.py
├── scripts/
│   └── ingest.py
├── tests/
│   └── __init__.py
└── docs/
    └── setup.md
```

## Running the Ingestion Pipeline

### 1. Execute the Ingestion Script

```bash
# Run the ingestion pipeline
python scripts/ingest.py

# Or with specific options
python scripts/ingest.py --sitemap-url https://humanoidai.vercel.app/sitemap.xml --batch-size 10
```

### 2. Verify the Ingestion

```bash
# Run validation script to check ingestion status
python scripts/validate.py
```

## Key Components

### Ingestion Pipeline Flow

1. **Sitemap Processing**: Download and parse sitemap.xml
2. **Web Crawling**: Fetch each URL from the sitemap
3. **Content Extraction**: Extract clean text from HTML pages
4. **Text Processing**: Normalize and chunk the content
5. **Embedding Generation**: Create vector embeddings using Cohere
6. **Vector Storage**: Store embeddings in Qdrant Cloud with metadata

### Configuration

The system uses the following configuration:

- Book site URL: `https://humanoidai.vercel.app/`
- Sitemap URL: `https://humanoidai.vercel.app/sitemap.xml`
- Default embedding model: Cohere's embed-english-v3.0
- Chunk size: 1000 characters with 200 character overlap
- Batch size: 10 (configurable)

## Troubleshooting

### Common Issues

1. **API Keys Not Found**: Ensure `.env` file contains all required API keys
2. **Qdrant Connection Issues**: Verify QDRANT_URL and QDRANT_API_KEY are correct
3. **Crawling Failures**: Check if the target website is accessible and not blocking requests

### Logging

The system logs to `backend/logs/` directory with different levels:
- `info`: General operation information
- `warning`: Non-critical issues
- `error`: Errors that require attention
- `debug`: Detailed information for troubleshooting