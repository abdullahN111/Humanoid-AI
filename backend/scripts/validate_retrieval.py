#!/usr/bin/env python3
"""
Main validation script for the Retrieval Pipeline Validation system.

This script executes the validation pipeline to ensure the RAG system
accurately returns relevant book content from the vector database.
"""

import argparse
import logging
import sys
import os
from datetime import datetime

# Add src to path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from src.validation.engine import get_validation_engine
from src.validation.reporter import get_validation_reporter
from src.tests.datasets import get_test_dataset
from src.validation.connection import get_qdrant_connection
from src.config.settings import settings

def setup_logging(verbose: bool = False):
    """Setup logging configuration."""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('logs/validation.log'),
            logging.StreamHandler(sys.stdout)
        ]
    )

def main():
    parser = argparse.ArgumentParser(description="Validate RAG retrieval pipeline")
    parser.add_argument("--batch-size", type=int, default=10,
                        help="Number of queries to process in each batch (default: 10)")
    parser.add_argument("--similarity-threshold", type=float, default=0.7,
                        help="Minimum similarity score for relevant results (default: 0.7)")
    parser.add_argument("--test-scenario", type=str, default="all",
                        choices=["all", "semantic", "metadata", "consistency"],
                        help="Test scenario to run (default: all)")
    parser.add_argument("--verbose", action="store_true",
                        help="Enable verbose logging")
    parser.add_argument("--output-file", type=str,
                        help="File to save the validation report (optional)")

    args = parser.parse_args()

    # Setup logging
    setup_logging(args.verbose)
    logger = logging.getLogger(__name__)

    # Validate settings
    errors = settings.validate()
    if errors:
        logger.error(f"Configuration errors: {errors}")
        sys.exit(1)

    logger.info("Starting retrieval pipeline validation...")

    try:
        # Initialize components
        qdrant_conn = get_qdrant_connection()

        # Validate connection to Qdrant
        if not qdrant_conn.validate_connection():
            logger.error("Failed to connect to Qdrant collection")
            sys.exit(1)

        # Get test dataset
        test_dataset = get_test_dataset()

        # Select queries based on test scenario
        if args.test_scenario == "semantic":
            queries = test_dataset.get_semantic_queries()
        elif args.test_scenario == "metadata":
            queries = test_dataset.get_metadata_filter_queries()
        elif args.test_scenario == "consistency":
            queries = test_dataset.get_consistency_verification_queries()
        else:  # "all"
            queries = test_dataset.get_all_queries()

        if not queries:
            logger.error(f"No queries found for test scenario: {args.test_scenario}")
            sys.exit(1)

        logger.info(f"Loaded {len(queries)} queries for validation")

        # Initialize validation engine
        validation_engine = get_validation_engine()

        # Execute validation
        logger.info("Executing validation queries...")
        results = validation_engine.execute_batch_queries(queries, batch_size=args.batch_size)

        # Generate report
        logger.info("Generating validation report...")
        reporter = get_validation_reporter()
        report = reporter.generate_detailed_report(results)

        # Print report to console
        reporter.print_report_to_console(report)

        # Save report to file if specified
        if args.output_file:
            success = reporter.save_report_to_file(report, args.output_file)
            if success:
                logger.info(f"Report saved to {args.output_file}")
            else:
                logger.error(f"Failed to save report to {args.output_file}")
                sys.exit(1)

        # Determine overall validation result
        relevant_count = sum(1 for result in results if result.is_relevant)
        total_count = len(results)
        success_rate = relevant_count / total_count if total_count > 0 else 0

        logger.info(f"Validation completed: {relevant_count}/{total_count} ({success_rate:.2%}) queries validated successfully")

        # Exit with appropriate code based on results
        if success_rate < 0.7:  # Less than 70% success rate is considered failure
            logger.warning("Validation success rate is below 70%, indicating potential issues")
            sys.exit(2)  # Different exit code for partial failure
        else:
            logger.info("Validation completed successfully!")
            sys.exit(0)

    except KeyboardInterrupt:
        logger.info("Validation interrupted by user")
        sys.exit(130)  # Standard exit code for Ctrl+C
    except Exception as e:
        logger.error(f"Unexpected error during validation: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()