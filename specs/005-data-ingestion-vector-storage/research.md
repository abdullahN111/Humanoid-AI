# Research: Data Ingestion & Vector Storage

**Feature**: 005-data-ingestion-vector-storage
**Created**: 2026-01-05
**Status**: Complete

## 1. Web Crawling and Sitemap Processing

### Decision: Use requests + BeautifulSoup for crawling
### Rationale:
- Lightweight and sufficient for the task
- Good for static content extraction
- Easy to implement sitemap parsing
- Handles basic HTTP requests and error handling

### Alternatives Considered:

**Scrapy:**
- Pros: More robust, handles complex crawling scenarios, built-in sitemap support
- Cons: Overkill for this simple use case, steeper learning curve

**Playwright:**
- Pros: Can handle JavaScript-rendered content
- Cons: Heavyweight, requires browser automation, unnecessary for Docusaurus static content

**requests + xml.etree.ElementTree:**
- Pros: Built-in XML parsing, lightweight
- Cons: Less convenient for HTTP handling than requests

### Final Choice: requests + BeautifulSoup + xml.etree.ElementTree
This combination provides:
- Efficient sitemap.xml parsing
- Reliable HTTP requests
- Easy HTML content extraction
- Good error handling capabilities

## 2. Text Extraction from Docusaurus Pages

### Decision: Custom parsing with BeautifulSoup for Docusaurus-specific extraction
### Rationale:
- Docusaurus has predictable HTML structure
- Can target specific content containers
- Achieves high accuracy by excluding navigation elements
- Customizable for specific content selectors

### Alternatives Considered:

**newspaper3k:**
- Pros: Designed for content extraction, handles various formats
- Cons: May not be optimized for Docusaurus structure, potential over-processing

**readability:**
- Pros: Good at extracting main content, handles various HTML structures
- Cons: May not be as accurate for Docusaurus-specific structure

**html2text:**
- Pros: Simple HTML to text conversion
- Cons: Less control over content selection, may include unwanted elements

### Final Choice: Custom BeautifulSoup extraction
Target selectors like:
- `main div[class*="docItem"]` for Docusaurus documentation
- `article` tags for content
- Exclude selectors for navigation, headers, footers

## 3. Embedding Model Selection

### Decision: Use Cohere embedding models via API
### Rationale:
- Requirement specifically mentions Cohere models
- Good quality embeddings for semantic search
- Reliable API with good rate limits
- Supports batch processing

### Alternatives Considered:

**OpenAI embeddings:**
- Pros: High quality, well-documented
- Cons: Not specified in requirements, potential cost concerns

**Sentence Transformers:**
- Pros: Open source, can run locally
- Cons: Requires model hosting, may not match Cohere quality

### Final Choice: Cohere API with appropriate model
- Model: cohere.embed-english-v3.0 (or latest version)
- Input type: "search_document" for stored documents
- Supports up to 512 dimensions

## 4. Qdrant Cloud Integration

### Decision: Use Qdrant Cloud free tier with Python client
### Rationale:
- Requirement specifically states Qdrant Cloud
- Python client is mature and well-documented
- Free tier sufficient for initial development
- Supports metadata storage and filtering

### Alternatives Considered:

**Pinecone:**
- Pros: Good performance, managed service
- Cons: Not specified in requirements

**Weaviate:**
- Pros: Open source, good features
- Cons: Not specified in requirements

### Final Choice: Qdrant Python client
- Collection schema with proper vector dimensions
- Metadata storage for source references
- Payload filtering for retrieval

## 5. Idempotent Ingestion Strategy

### Decision: Content hashing with URL tracking for deduplication
### Rationale:
- Prevents duplicate entries when pipeline is re-run
- Allows for incremental updates
- Efficient for checking if content has changed

### Alternatives Considered:

**Timestamp-based:**
- Pros: Simple to implement
- Cons: May miss content changes with same timestamp

**Database constraints:**
- Pros: Enforced at database level
- Cons: May cause ingestion failures

**URL tracking only:**
- Pros: Simple
- Cons: Won't detect if content changed at same URL

### Final Choice: Combined approach
- Store content hash with each chunk
- Track processed URLs
- Compare hashes to detect content changes
- Support resuming from failure points

## 6. Text Chunking Strategy

### Decision: Recursive character text splitting with overlap
### Rationale:
- Maintains context between chunks
- Handles various content types well
- Allows for appropriate chunk sizes for embedding models

### Alternatives Considered:

**Sentence splitting:**
- Pros: Natural boundaries
- Cons: May create very long or short chunks

**Fixed character length:**
- Pros: Predictable chunk sizes
- Cons: May break context in middle of sentences

### Final Choice: RecursiveCharacterTextSplitter
- Chunk size: 1000 characters (within embedding model limits)
- Chunk overlap: 200 characters (maintains context)
- Separators: ["\n\n", "\n", " ", ""]

## 7. Error Handling and Resilience

### Decision: Comprehensive error handling with retry mechanisms
### Rationale:
- Web requests can fail due to network issues
- API calls may have rate limits
- Need to handle temporary failures gracefully

### Final Approach:
- Retry mechanisms for HTTP requests and API calls
- Graceful degradation when individual URLs fail
- Progress tracking to resume from failure points
- Comprehensive logging for debugging