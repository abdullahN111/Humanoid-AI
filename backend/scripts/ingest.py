#!/usr/bin/env python3
"""
Ingestion script for the Data Ingestion & Vector Storage service.

This script orchestrates the entire ingestion pipeline:
1. Crawls the Docusaurus website using sitemap
2. Extracts and processes content
3. Generates embeddings
4. Stores embeddings in Qdrant
"""

import argparse
import logging
import sys
import os
from datetime import datetime

# Add src to path to import modules - adjust path to be relative to project root
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from src.ingestion.pipeline import run_ingestion_pipeline
from src.ingestion.crawler import parse_sitemap
from src.ingestion.extractor import extract_content_from_docusaurus_page, extract_page_title
from src.ingestion.processor import process_content, validate_content_quality
from src.embeddings.generator import get_embedding_generator
from src.storage.vector_db import get_qdrant_storage
from src.config.settings import settings
from src.logging_config import logger

def main():
    parser = argparse.ArgumentParser(description="Data Ingestion Pipeline")
    parser.add_argument("--sitemap-url", type=str, default=None,
                        help="URL of the sitemap.xml (default: from settings)")
    parser.add_argument("--batch-size", type=int, default=None,
                        help="Number of pages to process in each batch (default: from settings)")
    parser.add_argument("--verbose", action="store_true",
                        help="Enable verbose logging")
    parser.add_argument("--force-reprocess", action="store_true",
                        help="Force reprocessing of already processed pages")

    args = parser.parse_args()

    # Configure logging level
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Validate settings
    errors = settings.validate()
    if errors:
        logger.error(f"Configuration errors: {errors}")
        sys.exit(1)

    logger.info("Starting ingestion pipeline...")

    # Override settings with command line args if provided
    sitemap_url = args.sitemap_url or settings.SITEMAP_URL
    if args.batch_size:
        # Note: In a real implementation, we'd need to update the batch size in settings
        pass

    logger.info(f"Using sitemap URL: {sitemap_url}")

    try:
        # Run the ingestion pipeline
        ingestion_record = run_ingestion_pipeline(sitemap_url)

        logger.info(f"Ingestion completed with status: {ingestion_record.status}")
        logger.info(f"Processed URLs: {ingestion_record.processed_urls}")
        logger.info(f"Successful chunks: {ingestion_record.successful_chunks}")
        logger.info(f"Failed chunks: {ingestion_record.failed_chunks}")

        if ingestion_record.error_details:
            logger.error(f"Error details: {ingestion_record.error_details}")

        # Exit with appropriate code based on status
        if ingestion_record.status == "failed":
            sys.exit(1)
        elif ingestion_record.failed_chunks > 0:
            logger.warning("Some chunks failed to process")
            sys.exit(2)  # Use different exit code for partial failure
        else:
            logger.info("Ingestion pipeline completed successfully!")
            sys.exit(0)

    except KeyboardInterrupt:
        logger.info("Ingestion interrupted by user")
        sys.exit(130)  # Standard exit code for Ctrl+C
    except Exception as e:
        logger.error(f"Unexpected error during ingestion: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()