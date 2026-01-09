import logging
from typing import List, Dict, Any, Optional
from datetime import datetime
import hashlib
import json
import os
from src.ingestion.crawler import parse_sitemap, fetch_page_content_with_retry
from src.ingestion.extractor import extract_content_from_html, extract_page_title
from src.ingestion.processor import process_content
from src.storage.models import ContentChunk, IngestionRecord
from src.config.settings import settings

logger = logging.getLogger(__name__)

class IngestionPipeline:
    """Main ingestion pipeline for processing website content."""

    def __init__(self):
        self.tracking_file = os.path.join(os.path.dirname(__file__), '..', '..', 'ingestion_tracking.json')
        self.processed_urls = self._load_tracking_data()

    def _load_tracking_data(self) -> Dict[str, str]:
        """Load previously processed URLs from tracking file."""
        if os.path.exists(self.tracking_file):
            try:
                with open(self.tracking_file, 'r') as f:
                    data = json.load(f)
                    return data.get('processed_urls', {})
            except Exception as e:
                logger.warning(f"Could not load tracking data: {e}")
                return {}
        return {}

    def _save_tracking_data(self):
        """Save processed URLs to tracking file."""
        try:
            data = {
                'last_updated': datetime.now().isoformat(),
                'processed_urls': self.processed_urls
            }
            with open(self.tracking_file, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            logger.error(f"Could not save tracking data: {e}")

    def _is_url_processed(self, url: str, content_hash: str) -> bool:
        """Check if a URL has been processed with the same content."""
        if url not in self.processed_urls:
            return False
        # Check if content has changed by comparing hashes
        return self.processed_urls[url] == content_hash

    def _mark_url_processed(self, url: str, content_hash: str):
        """Mark a URL as processed with its content hash."""
        self.processed_urls[url] = content_hash
        self._save_tracking_data()

    def _get_content_hash(self, content: str) -> str:
        """Generate a hash for content to detect changes."""
        return hashlib.sha256(content.encode('utf-8')).hexdigest()

    def run_ingestion(self, sitemap_url: str = None) -> IngestionRecord:
        """
        Run the main ingestion pipeline.

        Args:
            sitemap_url: URL of the sitemap.xml. If None, uses default from settings.

        Returns:
            IngestionRecord with statistics about the run
        """
        if sitemap_url is None:
            sitemap_url = settings.SITEMAP_URL

        logger.info(f"Starting ingestion pipeline for {sitemap_url}")

        # Initialize ingestion record
        ingestion_record = IngestionRecord(
            status="running",
            start_time=datetime.now()
        )

        try:
            # Parse sitemap to get URLs
            urls = parse_sitemap(sitemap_url)
            logger.info(f"Found {len(urls)} URLs to process")

            # Process URLs in batches
            batch_size = settings.INGESTION_BATCH_SIZE
            successful_chunks = 0
            failed_chunks = 0

            for i in range(0, len(urls), batch_size):
                batch = urls[i:i + batch_size]
                logger.info(f"Processing batch {i//batch_size + 1}/{(len(urls)-1)//batch_size + 1}")

                batch_successful, batch_failed = self._process_batch(batch)
                successful_chunks += batch_successful
                failed_chunks += batch_failed

            # Update ingestion record
            ingestion_record.status = "completed" if failed_chunks == 0 else "completed_with_errors"
            ingestion_record.end_time = datetime.now()
            ingestion_record.processed_urls = len(urls)
            ingestion_record.successful_chunks = successful_chunks
            ingestion_record.failed_chunks = failed_chunks

            logger.info(f"Ingestion completed. Successful: {successful_chunks}, Failed: {failed_chunks}")
            return ingestion_record

        except Exception as e:
            logger.error(f"Ingestion failed: {e}")
            ingestion_record.status = "failed"
            ingestion_record.end_time = datetime.now()
            ingestion_record.error_details = {"error": str(e)}
            return ingestion_record

    def _process_batch(self, urls: List[str]) -> tuple[int, int]:
        """Process a batch of URLs and return (successful_count, failed_count)."""
        successful_count = 0
        failed_count = 0

        for url in urls:
            try:
                # Fetch page content
                html_content = fetch_page_content_with_retry(url)

                # Extract content and title
                content = extract_content_from_html(html_content, url)
                page_title = extract_page_title(html_content)

                # Get content hash to check if it's already processed
                content_hash = self._get_content_hash(content)

                # Check if this URL has been processed with the same content
                if self._is_url_processed(url, content_hash):
                    logger.info(f"Skipping {url} - already processed with same content")
                    continue

                # Process content into chunks
                content_chunks = process_content(content, url, page_title)

                # Import storage functionality
                from src.storage.vector_db import get_qdrant_storage
                from src.embeddings.generator import get_embedding_generator
                from src.storage.models import VectorEmbedding, SourceReference
                import uuid
                from datetime import datetime

                # Get storage and embedding generator instances
                storage = get_qdrant_storage()
                embedding_gen = get_embedding_generator()

                # Ensure collection exists
                storage.ensure_collection_exists()

                # Generate embeddings and store chunks
                embeddings = []
                source_references = []

                # Process each content chunk
                for i, content_chunk in enumerate(content_chunks):
                    try:
                        # Generate embedding for the chunk using the embedding generator
                        vector_embedding = embedding_gen.generate_embedding_for_chunk(
                            content_chunk.content,
                            content_chunk.id
                        )

                        if vector_embedding is not None:
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

                            logger.debug(f"Generated embedding for chunk {content_chunk.id}: {len(content_chunk.content)} chars")
                        else:
                            logger.error(f"Failed to generate embedding for chunk {content_chunk.id}")
                            continue
                    except Exception as e:
                        logger.error(f"Error generating embedding for chunk {content_chunk.id}: {e}")
                        continue

                # Store embeddings in Qdrant
                if embeddings:
                    success = storage.store_embeddings(embeddings, source_references)
                    if success:
                        logger.info(f"Successfully stored {len(embeddings)} embeddings for {url}")
                    else:
                        logger.error(f"Failed to store embeddings for {url}")
                        failed_count += len(embeddings)  # Count as failed
                        continue

                # Mark URL as processed
                self._mark_url_processed(url, content_hash)

                successful_count += len(content_chunks)
                logger.info(f"Successfully processed {url} - created {len(content_chunks)} chunks")

            except Exception as e:
                logger.error(f"Failed to process {url}: {e}")
                failed_count += 1

        return successful_count, failed_count

    def get_ingestion_status(self) -> Dict[str, Any]:
        """Get current ingestion status and statistics."""
        return {
            'total_processed_urls': len(self.processed_urls),
            'last_updated': datetime.now().isoformat(),
            'tracking_file': self.tracking_file
        }

def run_ingestion_pipeline(sitemap_url: str = None) -> IngestionRecord:
    """Convenience function to run the ingestion pipeline."""
    try:
        pipeline = IngestionPipeline()
        return pipeline.run_ingestion(sitemap_url)
    except Exception as e:
        logger.error(f"Error in ingestion pipeline: {e}")
        # Return a failed ingestion record
        return IngestionRecord(
            status="failed",
            end_time=datetime.now(),
            error_details={"error": str(e)}
        )

def validate_and_run_ingestion(sitemap_url: str = None) -> tuple[bool, IngestionRecord]:
    """
    Validate settings and run ingestion pipeline.

    Args:
        sitemap_url: URL of the sitemap.xml. If None, uses default from settings.

    Returns:
        Tuple of (success: bool, ingestion_record: IngestionRecord)
    """
    from src.config.settings import settings

    # Validate settings
    validation_errors = settings.validate()
    if validation_errors:
        logger.error(f"Configuration validation errors: {validation_errors}")
        record = IngestionRecord(
            status="failed",
            end_time=datetime.now(),
            error_details={"validation_errors": validation_errors}
        )
        return False, record

    # Run ingestion
    record = run_ingestion_pipeline(sitemap_url)
    success = record.status in ["completed", "completed_with_errors"]
    return success, record