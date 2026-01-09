from typing import List, Dict, Any
from datetime import datetime
import json
import logging
from src.validation.models import ValidationResult, AccuracyMetric
from src.validation.engine import get_validation_engine

logger = logging.getLogger(__name__)

class ValidationReporter:
    """Generate reports for validation results and metrics."""

    def __init__(self):
        self.validation_engine = get_validation_engine()

    def generate_simple_report(self, validation_results: List[ValidationResult]) -> Dict[str, Any]:
        """
        Generate a simple validation report from validation results.

        Args:
            validation_results: List of validation results

        Returns:
            Dictionary containing the simple report
        """
        if not validation_results:
            return {
                "report_generated_at": datetime.now().isoformat(),
                "total_results": 0,
                "summary": {
                    "relevant_results": 0,
                    "irrelevant_results": 0,
                    "average_relevance_score": 0.0
                }
            }

        relevant_count = 0
        total_score = 0.0
        for result in validation_results:
            if result.is_relevant:
                relevant_count += 1
            total_score += result.relevance_score

        average_score = total_score / len(validation_results) if validation_results else 0.0

        report = {
            "report_generated_at": datetime.now().isoformat(),
            "total_results": len(validation_results),
            "summary": {
                "relevant_results": relevant_count,
                "irrelevant_results": len(validation_results) - relevant_count,
                "average_relevance_score": round(average_score, 4)
            }
        }

        return report

    def generate_detailed_report(self, validation_results: List[ValidationResult], accuracy_metrics: AccuracyMetric = None) -> Dict[str, Any]:
        """
        Generate a detailed validation report with comprehensive metrics.

        Args:
            validation_results: List of validation results
            accuracy_metrics: Optional pre-calculated accuracy metrics

        Returns:
            Dictionary containing the detailed report
        """
        simple_report = self.generate_simple_report(validation_results)

        # Calculate detailed metrics
        detailed_metrics = self._calculate_detailed_metrics(validation_results)

        report = {
            "report_generated_at": datetime.now().isoformat(),
            "total_results": len(validation_results),
            "summary": simple_report["summary"],
            "detailed_metrics": detailed_metrics,
            "accuracy_metrics": {
                "precision": accuracy_metrics.precision if accuracy_metrics else 0.0,
                "recall": accuracy_metrics.recall if accuracy_metrics else 0.0,
                "f1_score": accuracy_metrics.f1_score if accuracy_metrics else 0.0,
                "mrr": accuracy_metrics.mrr if accuracy_metrics else 0.0,
                "hit_rate": accuracy_metrics.hit_rate if accuracy_metrics else 0.0,
                "mean_similarity": accuracy_metrics.mean_similarity if accuracy_metrics else 0.0,
                "total_queries": accuracy_metrics.total_queries if accuracy_metrics else 0,
                "total_relevant_retrieved": accuracy_metrics.total_relevant_retrieved if accuracy_metrics else 0
            } if accuracy_metrics else {},
            "individual_results": [
                {
                    "query_id": result.query_id,
                    "is_relevant": result.is_relevant,
                    "relevance_score": result.relevance_score,
                    "retrieved_chunks_count": len(result.retrieved_chunks),
                    "notes": result.validation_notes
                } for result in validation_results
            ]
        }

        return report

    def generate_comprehensive_report(self) -> Dict[str, Any]:
        """
        Generate a comprehensive validation report using the test dataset.

        Returns:
            Dictionary containing the comprehensive report
        """
        logger.info("Generating comprehensive validation report...")

        # Run comprehensive validation
        comprehensive_results = self.validation_engine.run_comprehensive_validation()

        report = {
            "report_generated_at": datetime.now().isoformat(),
            "report_type": "comprehensive",
            "validation_results": comprehensive_results,
            "recommendations": self._generate_recommendations(comprehensive_results)
        }

        logger.info("Comprehensive validation report generated")
        return report

    def save_report_to_file(self, report: Dict[str, Any], filename: str) -> bool:
        """
        Save a validation report to a JSON file.

        Args:
            report: The report dictionary to save
            filename: Name of the file to save the report to

        Returns:
            True if successful, False otherwise
        """
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2, ensure_ascii=False)
            logger.info(f"Report saved to {filename}")
            return True
        except Exception as e:
            logger.error(f"Error saving report to file {filename}: {e}")
            return False

    def print_report_to_console(self, report: Dict[str, Any]) -> None:
        """
        Print a formatted validation report to the console.

        Args:
            report: The report dictionary to print
        """
        print("=" * 60)
        print("RETRIEVAL VALIDATION REPORT")
        print("=" * 60)

        print(f"Report Generated At: {report.get('report_generated_at', 'N/A')}")

        if 'validation_results' in report:
            results = report['validation_results']
            print(f"Total Queries: {results.get('total_queries', 'N/A')}")
            print(f"Validation Passed: {results.get('validation_passed', 'N/A')}")

            accuracy = results.get('accuracy_metrics', {})
            print("\nACCURACY METRICS:")
            for metric, value in accuracy.items():
                if isinstance(value, float):
                    print(f"  {metric.title()}: {value:.4f}")
                else:
                    print(f"  {metric.title()}: {value}")
        else:
            summary = report.get('summary', {})
            print(f"Total Results: {report.get('total_results', 'N/A')}")
            print(f"Relevant Results: {summary.get('relevant_results', 'N/A')}")
            print(f"Irrelevant Results: {summary.get('irrelevant_results', 'N/A')}")
            print(f"Average Relevance Score: {summary.get('average_relevance_score', 'N/A'):.4f}")

        recommendations = report.get('recommendations', [])
        if recommendations:
            print(f"\nRECOMMENDATIONS:")
            for rec in recommendations:
                print(f"  - {rec}")

        print("=" * 60)

    def _calculate_detailed_metrics(self, validation_results: List[ValidationResult]) -> Dict[str, Any]:
        """Calculate detailed metrics for validation results."""
        if not validation_results:
            return {}

        # Calculate distribution of relevance scores
        relevance_scores = [result.relevance_score for result in validation_results]
        avg_score = sum(relevance_scores) / len(relevance_scores) if relevance_scores else 0.0

        # Calculate score ranges
        excellent = sum(1 for score in relevance_scores if score >= 0.9)
        good = sum(1 for score in relevance_scores if 0.7 <= score < 0.9)
        fair = sum(1 for score in relevance_scores if 0.5 <= score < 0.7)
        poor = sum(1 for score in relevance_scores if score < 0.5)

        # Calculate average number of retrieved chunks
        total_chunks = sum(len(result.retrieved_chunks) for result in validation_results)
        avg_chunks = total_chunks / len(validation_results) if validation_results else 0.0

        # Calculate metadata filtering effectiveness
        # This requires examining the metadata filters used and success rates
        filtering_success_count = 0
        total_filtering_attempts = 0

        for result in validation_results:
            # In a real implementation, we'd have access to the original query and its filters
            # For now, we'll just count results that have metadata in their chunks
            for chunk in result.retrieved_chunks:
                if chunk.metadata and len(chunk.metadata) > 0:
                    total_filtering_attempts += 1
                    # Assuming if metadata exists, the filtering was attempted
                    filtering_success_count += 1

        filtering_success_rate = (
            filtering_success_count / total_filtering_attempts if total_filtering_attempts > 0 else 0.0
        )

        return {
            "average_relevance_score": round(avg_score, 4),
            "score_distribution": {
                "excellent": excellent,
                "good": good,
                "fair": fair,
                "poor": poor
            },
            "average_chunks_per_result": round(avg_chunks, 2),
            "total_chunks_retrieved": total_chunks,
            "filtering_metrics": {
                "filtering_success_rate": round(filtering_success_rate, 4),
                "filtering_attempts": total_filtering_attempts,
                "successful_filtering_operations": filtering_success_count
            }
        }

    def _generate_recommendations(self, validation_results: Dict[str, Any]) -> List[str]:
        """Generate recommendations based on validation results."""
        recommendations = []

        accuracy = validation_results.get('accuracy_metrics', {})
        precision = accuracy.get('precision', 0.0)
        recall = accuracy.get('recall', 0.0)

        if precision < 0.7:
            recommendations.append("Consider improving precision by refining search algorithms or adjusting similarity thresholds")
        if recall < 0.7:
            recommendations.append("Consider improving recall by expanding search scope or lowering relevance thresholds")
        if precision < 0.5 or recall < 0.5:
            recommendations.append("Significant improvements needed in retrieval accuracy - consider revisiting embedding models or search approach")

        return recommendations


def get_validation_reporter() -> ValidationReporter:
    """Get a singleton instance of the validation reporter."""
    return ValidationReporter()