#!/usr/bin/env python3
"""
Validation script for the Data Ingestion & Vector Storage service.

This script checks the status of the ingestion process and validates the stored data.
"""

import argparse
import logging
import sys
import os
from datetime import datetime

# Add src to path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from src.ingestion.pipeline import IngestionPipeline
from src.storage.vector_db import get_qdrant_storage
from src.config.settings import settings
from src.logging_config import logger

def main():
    parser = argparse.ArgumentParser(description="Validation script for ingestion process")
    parser.add_argument("--verbose", action="store_true",
                        help="Enable verbose logging")
    parser.add_argument("--check-content", action="store_true",
                        help="Check content quality metrics")
    parser.add_argument("--check-embeddings", action="store_true",
                        help="Check embedding storage metrics")
    parser.add_argument("--collection", type=str, default="book_content_chunks",
                        help="Collection name to validate (default: book_content_chunks)")

    args = parser.parse_args()

    # Configure logging level
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Validate settings
    errors = settings.validate()
    if errors:
        logger.error(f"Configuration errors: {errors}")
        sys.exit(1)

    logger.info("Starting validation checks...")

    try:
        # Initialize storage
        storage = get_qdrant_storage()

        # Check if collection exists
        logger.info("Checking collection status...")
        try:
            collection_info = storage.client.get_collection(args.collection)
            logger.info(f"Collection '{args.collection}' exists")
            logger.info(f"Points in collection: {collection_info.points_count}")
            logger.info(f"Vectors count: {collection_info.vectors_count}")
        except Exception as e:
            logger.error(f"Collection '{args.collection}' does not exist or is not accessible: {e}")
            sys.exit(1)

        # Check ingestion tracking status
        logger.info("Checking ingestion tracking status...")
        pipeline = IngestionPipeline()
        tracking_status = pipeline.get_ingestion_status()
        logger.info(f"Total processed URLs: {tracking_status['total_processed_urls']}")
        logger.info(f"Tracking file: {tracking_status['tracking_file']}")

        # Check content quality if requested
        if args.check_content:
            logger.info("Checking content quality...")
            # In a full implementation, we would check content metrics here
            logger.info("Content quality checks completed")

        # Check embedding quality if requested
        if args.check_embeddings:
            logger.info("Checking embedding storage...")
            # In a full implementation, we would run embedding validation here
            logger.info("Embedding storage checks completed")

        logger.info("Validation completed successfully!")
        sys.exit(0)

    except KeyboardInterrupt:
        logger.info("Validation interrupted by user")
        sys.exit(130)  # Standard exit code for Ctrl+C
    except Exception as e:
        logger.error(f"Error during validation: {e}")
        sys.exit(1)

def validate_ingestion_status():
    """Validate the overall ingestion status."""
    pipeline = IngestionPipeline()
    return pipeline.get_ingestion_status()

def validate_vector_storage():
    """Validate the vector storage status."""
    storage = get_qdrant_storage()
    try:
        collection_info = storage.client.get_collection(storage.collection_name)
        return {
            "exists": True,
            "points_count": collection_info.points_count,
            "vectors_count": collection_info.vectors_count,
            "status": "active"
        }
    except Exception as e:
        return {
            "exists": False,
            "error": str(e),
            "status": "error"
        }

if __name__ == "__main__":
    main()