#!/usr/bin/env python3
"""
Report generation script for the Retrieval Pipeline Validation system.

This script generates comprehensive validation reports from existing validation data.
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
            logging.FileHandler('logs/report_generation.log'),
            logging.StreamHandler(sys.stdout)
        ]
    )

def main():
    parser = argparse.ArgumentParser(description="Generate validation reports")
    parser.add_argument("--report-type", type=str, default="comprehensive",
                        choices=["simple", "detailed", "comprehensive"],
                        help="Type of report to generate (default: comprehensive)")
    parser.add_argument("--output-file", type=str, required=True,
                        help="File to save the validation report")
    parser.add_argument("--verbose", action="store_true",
                        help="Enable verbose logging")
    parser.add_argument("--validation-results-file", type=str,
                        help="JSON file containing existing validation results to report on")

    args = parser.parse_args()

    # Setup logging
    setup_logging(args.verbose)
    logger = logging.getLogger(__name__)

    # Validate settings
    errors = settings.validate()
    if errors:
        logger.error(f"Configuration errors: {errors}")
        sys.exit(1)

    logger.info("Starting validation report generation...")

    try:
        # Initialize components
        qdrant_conn = get_qdrant_connection()

        # Validate connection to Qdrant
        if not qdrant_conn.validate_connection():
            logger.error("Failed to connect to Qdrant collection")
            sys.exit(1)

        # Initialize validation reporter
        reporter = get_validation_reporter()

        if args.validation_results_file:
            # Generate report from existing validation results file
            import json
            with open(args.validation_results_file, 'r', encoding='utf-8') as f:
                validation_results = json.load(f)

            # Generate report based on the loaded results
            if args.report_type == "simple":
                # For simple reports from existing data, we'd need to reconstruct the objects
                # This is a simplified approach
                report = {
                    "report_generated_at": datetime.now().isoformat(),
                    "report_type": "simple",
                    "original_data_file": args.validation_results_file,
                    "summary": {
                        "total_results": len(validation_results) if isinstance(validation_results, list) else 0,
                        "status": "loaded from existing data"
                    }
                }
            elif args.report_type == "detailed":
                # Similar approach for detailed reports
                report = {
                    "report_generated_at": datetime.now().isoformat(),
                    "report_type": "detailed",
                    "original_data_file": args.validation_results_file,
                    "summary": {
                        "total_results": len(validation_results) if isinstance(validation_results, list) else 0,
                        "status": "loaded from existing data"
                    }
                }
            else:  # comprehensive
                report = {
                    "report_generated_at": datetime.now().isoformat(),
                    "report_type": "comprehensive",
                    "original_data_file": args.validation_results_file,
                    "validation_results": validation_results,
                    "status": "comprehensive report from existing data"
                }
        else:
            # Generate new validation and report
            logger.info("Generating new validation results for report...")

            # Get test dataset
            test_dataset = get_test_dataset()
            queries = test_dataset.get_all_queries()

            if not queries:
                logger.error("No queries found in test dataset")
                sys.exit(1)

            # Initialize validation engine
            validation_engine = get_validation_engine()

            # Execute validation for sample queries
            logger.info(f"Executing validation for {len(queries)} queries...")
            results = validation_engine.execute_batch_queries(queries, batch_size=5)

            # Generate report based on the report type
            if args.report_type == "simple":
                report = reporter.generate_simple_report(results)
            elif args.report_type == "detailed":
                # For detailed report, we need to calculate accuracy metrics as well
                # This is a simplified approach - in a real system, we'd calculate proper metrics
                from src.validation.metrics import AccuracyMetric
                dummy_metrics = None  # In a real implementation, we'd calculate these
                report = reporter.generate_detailed_report(results, dummy_metrics)
            else:  # comprehensive
                report = reporter.generate_comprehensive_report()

        # Save report to file
        success = reporter.save_report_to_file(report, args.output_file)
        if success:
            logger.info(f"Report successfully generated and saved to {args.output_file}")

            # Print report to console
            reporter.print_report_to_console(report)
        else:
            logger.error(f"Failed to save report to {args.output_file}")
            sys.exit(1)

        logger.info("Report generation completed successfully!")
        sys.exit(0)

    except KeyboardInterrupt:
        logger.info("Report generation interrupted by user")
        sys.exit(130)  # Standard exit code for Ctrl+C
    except Exception as e:
        logger.error(f"Unexpected error during report generation: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()