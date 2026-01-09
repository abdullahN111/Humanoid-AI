# Setup Documentation: Data Ingestion & Vector Storage

## Prerequisites

- Python 3.9 or higher
- `uv` package manager (install with `pip install uv`)
- Access to API keys for:
  - Qdrant Cloud (vector database)
  - Cohere API (embedding generation)

## Installation

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-directory>
```

### 2. Set up Environment

Copy the environment variables file and update with your credentials:

```bash
cp .env.example .env
# Edit .env and add your API keys and configuration
```

Required environment variables:
- `QDRANT_URL`: Your Qdrant Cloud endpoint URL
- `QDRANT_API_KEY`: Your Qdrant Cloud API key
- `COHERE_API_KEY`: Your Cohere API key
- `BOOK_SITE_URL`: Base URL of the book website (default: https://humanoidai.vercel.app/)
- `SITEMAP_URL`: URL of the sitemap.xml (default: https://humanoidai.vercel.app/sitemap.xml)
- `INGESTION_BATCH_SIZE`: Number of pages to process in each batch (default: 10)
- `EMBEDDING_MODEL`: Name of the embedding model to use (default: cohere/embed-english-v3.0)

### 3. Install Dependencies

Using `uv` (recommended):
```bash
uv sync
```

Or using pip:
```bash
pip install -r requirements.txt
```

## Configuration

### Settings

The application uses the following configuration from environment variables:

- **Qdrant Configuration**:
  - `QDRANT_URL`: Cloud endpoint for vector storage
  - `QDRANT_API_KEY`: Authentication key for Qdrant

- **Cohere Configuration**:
  - `COHERE_API_KEY`: API key for embedding generation

- **Ingestion Configuration**:
  - `INGESTION_BATCH_SIZE`: Number of URLs to process in parallel
  - `EMBEDDING_MODEL`: Model name for Cohere embeddings

## Usage

### Run the Ingestion Pipeline

```bash
python scripts/ingest.py
```

Additional options:
```bash
# With custom sitemap URL
python scripts/ingest.py --sitemap-url https://example.com/sitemap.xml

# With custom batch size
python scripts/ingest.py --batch-size 20

# With verbose logging
python scripts/ingest.py --verbose

# Force reprocessing of all pages
python scripts/ingest.py --force-reprocess
```

### Validate the Ingestion

```bash
python scripts/validate.py
```

Additional validation options:
```bash
# Check content quality metrics
python scripts/validate.py --check-content

# Check embedding storage metrics
python scripts/validate.py --check-embeddings

# With verbose logging
python scripts/validate.py --verbose
```

## Architecture

### Components

1. **Ingestion Module** (`src/ingestion/`):
   - `crawler.py`: Sitemap parsing and web crawling
   - `extractor.py`: HTML content extraction
   - `processor.py`: Text processing and chunking
   - `pipeline.py`: Main orchestration logic

2. **Embeddings Module** (`src/embeddings/`):
   - `client.py`: Cohere API client with retry logic
   - `generator.py`: Embedding generation functions

3. **Storage Module** (`src/storage/`):
   - `models.py`: Data model definitions
   - `vector_db.py`: Qdrant Cloud integration

4. **Configuration Module** (`src/config/`):
   - `settings.py`: Environment variable management

## Troubleshooting

### Common Issues

1. **API Key Errors**: Ensure all required API keys are set in `.env`
2. **Qdrant Connection**: Verify QDRANT_URL and QDRANT_API_KEY are correct
3. **Rate Limits**: The system includes retry logic for API rate limits
4. **Memory Issues**: Reduce batch size if processing large documents

### Logging

The system logs to both console and files in the `logs/` directory:
- `ingestion.log`: Main ingestion logs
- Log levels: INFO, WARNING, ERROR, DEBUG (when verbose is enabled)

## Development

### Running Tests

```bash
# Run all tests
python -m pytest tests/

# Run specific test module
python -m pytest tests/test_crawler.py
```

### Code Formatting

The project uses standard Python formatting. Ensure code is properly formatted before committing.