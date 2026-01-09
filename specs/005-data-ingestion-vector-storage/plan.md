# Implementation Plan: Data Ingestion & Vector Storage

**Feature**: 005-data-ingestion-vector-storage
**Created**: 2026-01-05
**Status**: Draft
**Author**: Claude Code

## Technical Context

The Data Ingestion & Vector Storage feature implements the foundational pipeline for the RAG system. It will:
- Crawl the Docusaurus website at https://humanoidai.vercel.app/ using its sitemap
- Extract clean text content from pages
- Generate vector embeddings using embedding models
- Store embeddings and metadata in a vector database
- Ensure idempotent and repeatable ingestion

**Architecture**: Backend Python service using uv for package management
**Dependencies**:
- Web crawling and text extraction libraries
- Embedding model integration
- Vector database client (Qdrant)
- Configuration management

## Constitution Check

This implementation aligns with the project constitution:

✓ **Spec-Driven Development**: Implementation follows the detailed specification in spec.md
✓ **Accuracy and Grounding**: System will index only book content for RAG responses
✓ **Reproducibility**: Using uv for consistent dependency management and environment setup
✓ **Clear Communication**: Well-documented code with clear error messages
✓ **Technology Stack Standards**: Using Qdrant Cloud for vector storage as specified
✓ **Content Accuracy**: Maintaining synchronization between book content and RAG index

## Gates

- [x] Architecture supports modular, inspectable design
- [x] Implementation allows for versioned, backward-compatible APIs
- [x] Dependencies align with project technology stack
- [x] Error handling provides clear, actionable feedback
- [x] Environment variables are properly abstracted and documented

## Phase 0: Research & Unknowns Resolution

Research completed and documented in research.md. All unknowns have been resolved:

- Web crawling approach: requests + BeautifulSoup for efficient sitemap processing
- Text extraction: Custom parsing with BeautifulSoup targeting Docusaurus structure
- Embedding models: Cohere API integration
- Vector storage: Qdrant Cloud with Python client
- Idempotent ingestion: Content hashing with URL tracking

### Implementation Approach

The backend will be structured as a Python project using uv for package management. The ingestion pipeline will be implemented as a series of steps:

1. **Sitemap Processing**: Download and parse sitemap.xml to get all URLs
2. **Content Crawling**: Fetch each URL and extract text content
3. **Text Processing**: Clean and normalize the extracted text
4. **Content Chunking**: Split large documents into manageable chunks
5. **Embedding Generation**: Generate vector embeddings for each chunk
6. **Vector Storage**: Store embeddings with metadata in Qdrant Cloud
7. **Tracking**: Maintain ingestion state to support idempotent runs

## Phase 1: Design & Contracts

### Project Structure
```
backend/
├── pyproject.toml          # Project configuration with dependencies
├── .env                    # Environment variables template
├── .env.example            # Example environment variables
├── src/
│   ├── __init__.py
│   ├── main.py             # Main ingestion script
│   ├── config/
│   │   ├── __init__.py
│   │   └── settings.py     # Configuration management
│   ├── ingestion/
│   │   ├── __init__.py
│   │   ├── crawler.py      # Web crawling functionality
│   │   ├── extractor.py    # Text extraction from HTML
│   │   ├── processor.py    # Text normalization and chunking
│   │   └── pipeline.py     # Main pipeline orchestration
│   ├── embeddings/
│   │   ├── __init__.py
│   │   ├── generator.py    # Embedding generation
│   │   └── client.py       # Embedding API client
│   └── storage/
│       ├── __init__.py
│       ├── vector_db.py    # Qdrant Cloud integration
│       └── models.py       # Data models for storage
├── scripts/
│   ├── ingest.py           # Ingestion script entrypoint
│   └── validate.py         # Validation script
├── tests/
│   ├── __init__.py
│   ├── test_crawler.py
│   ├── test_extractor.py
│   └── test_pipeline.py
└── docs/
    └── setup.md            # Setup and run documentation
```

### Data Models

Based on the spec, the key entities are:

1. **Content Chunk**: A segment of extracted text with metadata
2. **Vector Embedding**: Numerical representation of content with metadata
3. **Source Reference**: Links embeddings back to original content

### API Contracts (Future)

When the ingestion service is expanded to include API endpoints, these would be the contracts based on functional requirements:

- `POST /api/v1/ingest`: Trigger ingestion pipeline
- `GET /api/v1/ingest/status`: Check ingestion status
- `DELETE /api/v1/ingest`: Clear vector database

### Environment Variables

The system will use the following environment variables (from root .env file):

- `QDRANT_URL`: Qdrant Cloud endpoint URL
- `QDRANT_API_KEY`: Qdrant Cloud API key
- `COHERE_API_KEY`: Cohere API key for embeddings
- `BOOK_SITE_URL`: Base URL of the book website (https://humanoidai.vercel.app/)
- `SITEMAP_URL`: URL of the sitemap.xml
- `INGESTION_BATCH_SIZE`: Number of pages to process in each batch
- `EMBEDDING_MODEL`: Name of the embedding model to use

## Phase 2: Implementation Plan

### Step 1: Project Setup
1. Create `backend/` directory
2. Initialize Python project with `uv init`
3. Add required dependencies using `uv add`
4. Create directory structure

### Step 2: Configuration Management
1. Create settings module to manage environment variables
2. Set up logging configuration
3. Create .env.example file

### Step 3: Web Crawling Module
1. Implement sitemap.xml parser
2. Create web crawler to fetch pages
3. Add error handling for failed URLs

### Step 4: Text Extraction Module
1. Implement HTML-to-text extraction
2. Add content filtering to exclude navigation/UI elements
3. Create text normalization functions

### Step 5: Content Processing
1. Implement text chunking algorithm
2. Add content validation
3. Create content hashing for deduplication

### Step 6: Embedding Generation
1. Integrate with Cohere embedding API
2. Implement batch embedding generation
3. Add retry logic for API failures

### Step 7: Vector Storage
1. Set up Qdrant Cloud collection schema
2. Implement embedding storage with metadata
3. Add source reference tracking

### Step 8: Pipeline Orchestration
1. Create main ingestion pipeline
2. Implement idempotent execution
3. Add progress tracking and logging

### Step 9: Documentation and Scripts
1. Create setup documentation
2. Create ingestion execution script
3. Add validation and testing scripts

## Quality Assurance

- Unit tests for each module
- Integration tests for the full pipeline
- Performance testing for large document sets
- Validation of embedding quality
- Error handling verification

## Deployment Considerations

- Containerization support for consistent deployment
- Environment-specific configuration
- Monitoring and logging capabilities
- Backup and recovery procedures