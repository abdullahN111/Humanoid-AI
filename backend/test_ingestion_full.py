import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, '.')

from src.ingestion.pipeline import IngestionPipeline
from src.config.settings import settings

def test_ingestion_with_mock_embeddings():
    """
    Test the ingestion pipeline by temporarily modifying it to not call the embedding API
    during the test to avoid rate limiting.
    """
    print("Testing ingestion with mocked embeddings to avoid rate limiting...")

    try:
        # Create pipeline
        pipeline = IngestionPipeline()
        print("Pipeline created successfully")

        # Parse sitemap to see what URLs are available
        from src.ingestion.crawler import parse_sitemap
        urls = parse_sitemap(settings.SITEMAP_URL)
        print(f"Found {len(urls)} URLs in sitemap")

        # Take just the first URL to test
        test_urls = urls[:1]  # Just test with one URL
        print(f"Testing with URL: {test_urls[0]}")

        # Manually process the batch without calling the embedding API
        successful_count = 0
        failed_count = 0

        for url in test_urls:
            try:
                print(f"Processing URL: {url}")

                # Fetch page content
                from src.ingestion.crawler import fetch_page_content_with_retry
                html_content = fetch_page_content_with_retry(url)
                print(f"  Fetched HTML content: {len(html_content)} chars")

                # Extract content and title
                from src.ingestion.extractor import extract_content_from_html, extract_page_title
                content = extract_content_from_html(html_content, url)
                page_title = extract_page_title(html_content)
                print(f"  Extracted content: {len(content)} chars, title: {page_title}")

                # Process content into chunks
                from src.ingestion.processor import process_content
                content_chunks = process_content(content, url, page_title)
                print(f"  Created {len(content_chunks)} content chunks")

                if content_chunks:
                    print(f"  First chunk preview: {content_chunks[0].content[:100]}...")

                # Mock the embedding generation and storage process
                print("  Mocking embedding generation and storage...")

                # Import storage functionality
                from src.storage.vector_db import get_qdrant_storage
                from src.storage.models import VectorEmbedding, SourceReference
                import uuid
                from datetime import datetime

                # Get storage instance
                storage = get_qdrant_storage()

                # Ensure collection exists
                storage.ensure_collection_exists()

                # Generate mock embeddings and store chunks
                embeddings = []
                source_references = []

                # Process each content chunk with mock embeddings
                for i, content_chunk in enumerate(content_chunks):
                    # Create a mock embedding (using a simple mock - in real usage this would call the API)
                    # For testing purposes, create a mock vector (this simulates what the API would return)
                    mock_vector = [0.1] * 1024  # Mock embedding vector of size 1024 (Cohere's default)

                    # Create mock vector embedding
                    vector_embedding = VectorEmbedding(
                        chunk_id=content_chunk.id,
                        vector=mock_vector,
                        model_name="mock-embed-english-v3.0",
                        model_version="",  # This would be populated if the API provides it
                        vector_size=len(mock_vector)
                    )

                    # Update the embedding ID to be consistent with our pattern
                    vector_embedding.id = str(uuid.uuid5(uuid.NAMESPACE_DNS, f"{url}_{content_chunk.id}_{i}"))
                    embeddings.append(vector_embedding)

                    # Create source reference
                    source_ref = SourceReference(
                        chunk_id=content_chunk.id,
                        source_url=url,
                        page_title=page_title,
                        section_title=content_chunk.metadata.get('section_title', '') if content_chunk.metadata else '',
                        content_preview=content_chunk.content[:200],
                        created_at=datetime.now()
                    )
                    source_references.append(source_ref)

                    print(f"    Generated mock embedding for chunk {content_chunk.id}")

                # Store embeddings in Qdrant (with mock data)
                if embeddings:
                    success = storage.store_embeddings(embeddings, source_references)
                    if success:
                        print(f"    Successfully stored {len(embeddings)} mock embeddings for {url}")
                    else:
                        print(f"    Failed to store mock embeddings for {url}")
                        failed_count += len(embeddings)
                        continue

                # Mark URL as processed
                content_hash = pipeline._get_content_hash(content)
                pipeline._mark_url_processed(url, content_hash)

                successful_count += len(content_chunks)
                print(f"  Successfully processed {url} - created {len(content_chunks)} chunks")

            except Exception as e:
                print(f"  Error processing {url}: {e}")
                import traceback
                traceback.print_exc()
                failed_count += 1

        print(f"Ingestion test completed with mock embeddings. Successful: {successful_count}, Failed: {failed_count}")

        # Now check the Qdrant collection to see if data was stored
        from src.storage.vector_db import get_qdrant_storage
        storage = get_qdrant_storage()
        points_count = storage.client.count(storage.collection_name)
        print(f"Final count in Qdrant: {points_count.count} points")

    except Exception as e:
        print(f"Error during ingestion test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_ingestion_with_mock_embeddings()