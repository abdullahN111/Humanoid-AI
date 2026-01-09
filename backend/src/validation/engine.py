from typing import List, Dict, Any
import logging
from src.validation.search import get_similarity_search
from src.validation.metrics import get_validation_metrics
from src.validation.models import RetrievalQuery, RetrievedChunk, ValidationResult, AccuracyMetric
from src.tests.datasets import get_test_dataset
from src.validation.consistency import get_content_consistency_validator

logger = logging.getLogger(__name__)

class ValidationEngine:
    """Main validation engine that orchestrates the retrieval validation process."""

    def __init__(self):
        self.search = get_similarity_search()
        self.metrics = get_validation_metrics()
        self.dataset = get_test_dataset()
        self.consistency_validator = get_content_consistency_validator()

    def execute_single_query(self, retrieval_query: RetrievalQuery) -> ValidationResult:
        """
        Execute a single retrieval query and validate the results.

        Args:
            retrieval_query: The query to execute and validate

        Returns:
            ValidationResult containing the validation outcome
        """
        try:
            # Validate the query first
            validation_errors = retrieval_query.validate()
            if validation_errors:
                logger.error(f"Invalid retrieval query: {validation_errors}")
                return ValidationResult(
                    query_id=retrieval_query.id,
                    is_relevant=False,
                    relevance_score=0.0,
                    validation_notes=f"Invalid query: {', '.join(validation_errors)}"
                )

            # Perform the search using the query text
            retrieved_chunks = self.search.search_by_text(
                query_text=retrieval_query.query_text,
                limit=10,  # Retrieve more results than needed for thorough validation
                metadata_filters=retrieval_query.metadata_filters
            )

            # Handle empty results case
            if not retrieved_chunks:
                logger.warning(f"No results retrieved for query: {retrieval_query.query_text[:50]}...")
                return ValidationResult(
                    query_id=retrieval_query.id,
                    retrieved_chunks=[],
                    is_relevant=False,
                    relevance_score=0.0,
                    validation_notes="No results retrieved for the query"
                )

            # Validate the retrieved results against expected results
            validation_result = self._validate_retrieved_chunks(
                retrieval_query, retrieved_chunks
            )

            logger.info(f"Query '{retrieval_query.query_text[:50]}...' validation completed: {len(retrieved_chunks)} chunks retrieved")
            return validation_result

        except Exception as e:
            logger.error(f"Error executing query '{retrieval_query.query_text}': {e}")
            return ValidationResult(
                query_id=retrieval_query.id,
                is_relevant=False,
                relevance_score=0.0,
                validation_notes=f"Error during execution: {str(e)}"
            )

    def detect_edge_cases(self, retrieval_query: RetrievalQuery, retrieved_chunks: List[RetrievedChunk]) -> List[str]:
        """
        Detect edge cases in the validation results.

        Args:
            retrieval_query: The original retrieval query
            retrieved_chunks: List of retrieved chunks

        Returns:
            List of detected edge cases
        """
        edge_cases = []

        # Check for empty results
        if not retrieved_chunks:
            edge_cases.append("empty_results: Query returned no results")

        # Check for very short queries
        if len(retrieval_query.query_text.strip()) < 3:
            edge_cases.append("short_query: Query is very short (< 3 characters)")

        # Check for very long queries
        if len(retrieval_query.query_text) > 500:
            edge_cases.append("long_query: Query is very long (> 500 characters)")

        # Check for very low similarity scores
        if retrieved_chunks:
            avg_similarity = sum(chunk.similarity_score for chunk in retrieved_chunks) / len(retrieved_chunks)
            if avg_similarity < 0.3:
                edge_cases.append(f"low_similarity: Average similarity score is very low ({avg_similarity:.2f})")

        # Check for potential hallucination (results completely unrelated to query)
        if retrieved_chunks and not self._query_content_related(retrieval_query.query_text, retrieved_chunks):
            edge_cases.append("potential_hallucination: Retrieved content may not be related to query")

        # Check for too many results
        if len(retrieved_chunks) > 50:
            edge_cases.append(f"too_many_results: Retrieved {len(retrieved_chunks)} chunks (possibly too many)")

        # Check for metadata filtering issues
        if retrieval_query.metadata_filters:
            expected_source = retrieval_query.metadata_filters.get('source_url')
            if expected_source:
                matching_chunks = [chunk for chunk in retrieved_chunks if chunk.source_url == expected_source]
                if len(matching_chunks) == 0:
                    edge_cases.append(f"filter_mismatch: No chunks from expected source {expected_source}")

        return edge_cases

    def _query_content_related(self, query: str, chunks: List[RetrievedChunk]) -> bool:
        """
        Check if the retrieved chunks are related to the query content.

        Args:
            query: The query text
            chunks: List of retrieved chunks

        Returns:
            True if chunks seem related to the query, False otherwise
        """
        if not chunks:
            return False

        query_lower = query.lower()
        # Extract key terms from query
        key_terms = [term.strip() for term in query_lower.split() if len(term) > 3]

        if not key_terms:
            return True  # Can't evaluate with no key terms

        # Check if any key terms appear in the retrieved content
        for chunk in chunks:
            chunk_lower = chunk.content.lower()
            for term in key_terms:
                if term in chunk_lower:
                    return True  # Found at least one match

        return False  # No matches found between query and retrieved content

    def execute_batch_queries(self, retrieval_queries: List[RetrievalQuery], batch_size: int = 10) -> List[ValidationResult]:
        """
        Execute a batch of retrieval queries.

        Args:
            retrieval_queries: List of queries to execute
            batch_size: Number of queries to process in each batch

        Returns:
            List of ValidationResult objects
        """
        results = []

        for i in range(0, len(retrieval_queries), batch_size):
            batch = retrieval_queries[i:i + batch_size]
            logger.info(f"Processing batch {i//batch_size + 1}/{(len(retrieval_queries)-1)//batch_size + 1}")

            for query in batch:
                result = self.execute_single_query(query)
                results.append(result)

        logger.info(f"Batch execution completed: {len(results)} queries processed")
        return results

    def validate_retrieval_accuracy(self, retrieval_queries: List[RetrievalQuery]) -> AccuracyMetric:
        """
        Validate the overall retrieval accuracy across multiple queries.

        Args:
            retrieval_queries: List of queries to validate

        Returns:
            AccuracyMetric with overall performance metrics
        """
        # Execute all queries
        validation_results = self.execute_batch_queries(retrieval_queries)

        # Extract retrieved chunks and expected results for metric calculation
        all_retrieved = []
        all_expected = []

        for query in retrieval_queries:
            # For each query, retrieve the results and compare with expected
            retrieved_chunks = self.search.search_by_text(
                query_text=query.query_text,
                limit=10,
                metadata_filters=query.metadata_filters
            )

            all_retrieved.append(retrieved_chunks)

            # Convert expected results to RetrievedChunk format for comparison
            expected_chunks = self._convert_expected_to_chunks(query.expected_results)
            all_expected.append(expected_chunks)

        # Calculate accuracy metrics
        accuracy_metric = self.metrics.calculate_accuracy_metrics(all_retrieved, all_expected)

        logger.info(f"Retrieval accuracy validation completed: {accuracy_metric.precision:.2f} precision, {accuracy_metric.recall:.2f} recall")
        return accuracy_metric

    def _validate_retrieved_chunks(self, retrieval_query: RetrievalQuery, retrieved_chunks: List[RetrievedChunk]) -> ValidationResult:
        """Validate retrieved chunks against expected results."""
        if not retrieved_chunks:
            return ValidationResult(
                query_id=retrieval_query.id,
                retrieved_chunks=[],
                is_relevant=False,
                relevance_score=0.0,
                validation_notes="No chunks retrieved for the query"
            )

        # Determine if results are relevant by comparing with expected results
        relevant_count = 0
        total_expected = len(retrieval_query.expected_results)

        for expected in retrieval_query.expected_results:
            for retrieved in retrieved_chunks:
                if self._matches_expected_result(retrieved, expected):
                    relevant_count += 1
                    break

        # Calculate relevance score
        relevance_score = relevant_count / total_expected if total_expected > 0 else 0.0
        is_relevant = relevance_score > 0.5  # Threshold for relevance

        # Calculate detailed accuracy metrics for this query
        expected_chunks = self._convert_expected_to_chunks(retrieval_query.expected_results)
        precision = self.metrics.calculate_precision(retrieved_chunks, expected_chunks)
        recall = self.metrics.calculate_recall(retrieved_chunks, expected_chunks)
        f1_score = self.metrics.calculate_f1_score(precision, recall)

        # Prepare accuracy metrics
        accuracy_metrics = {
            "precision": precision,
            "recall": recall,
            "f1_score": f1_score,
            "relevant_count": relevant_count,
            "total_expected": total_expected,
            "total_retrieved": len(retrieved_chunks)
        }

        validation_result = ValidationResult(
            query_id=retrieval_query.id,
            retrieved_chunks=retrieved_chunks,
            is_relevant=is_relevant,
            relevance_score=relevance_score,
            accuracy_metrics=accuracy_metrics,
            validation_notes=f"Retrieved {len(retrieved_chunks)} chunks, {relevant_count}/{total_expected} matched expected results"
        )

        return validation_result

    def _matches_expected_result(self, retrieved_chunk: RetrievedChunk, expected_result: Dict[str, Any]) -> bool:
        """Check if a retrieved chunk matches an expected result."""
        # Check if content contains expected keywords
        if "content_keywords" in expected_result:
            content_lower = retrieved_chunk.content.lower()
            for keyword in expected_result["content_keywords"]:
                if keyword.lower() not in content_lower:
                    return False

        # Check if source URL matches
        if "source_url" in expected_result:
            if expected_result["source_url"] != retrieved_chunk.source_url:
                return False

        # Check if exact content matches
        if "exact_content" in expected_result:
            if expected_result["exact_content"] != retrieved_chunk.content:
                return False

        # Check if page title matches
        if "expected_title" in expected_result:
            if expected_result["expected_title"] != retrieved_chunk.page_title:
                return False

        return True

    def validate_url_filtering(self, url_filter: str, query_text: str) -> List[RetrievedChunk]:
        """
        Validate that the system properly filters results by URL.

        Args:
            url_filter: The URL to filter by
            query_text: The query text to search for

        Returns:
            List of retrieved chunks that match both the query and the URL filter
        """
        # Perform search with URL filter
        metadata_filters = {"source_url": url_filter}

        retrieved_chunks = self.search.search_by_text(
            query_text=query_text,
            limit=10,
            metadata_filters=metadata_filters
        )

        # Verify all results come from the specified URL
        filtered_chunks = []
        for chunk in retrieved_chunks:
            if chunk.source_url == url_filter:
                filtered_chunks.append(chunk)

        logger.info(f"URL filtering validation: {len(filtered_chunks)}/{len(retrieved_chunks)} results from {url_filter}")
        return filtered_chunks

    def validate_section_filtering(self, section_filter: str, query_text: str) -> List[RetrievedChunk]:
        """
        Validate that the system properly filters results by section.

        Args:
            section_filter: The section to filter by
            query_text: The query text to search for

        Returns:
            List of retrieved chunks that match both the query and the section filter
        """
        # Perform search with section filter
        metadata_filters = {"section": section_filter}

        retrieved_chunks = self.search.search_by_text(
            query_text=query_text,
            limit=10,
            metadata_filters=metadata_filters
        )

        # Verify all results come from the specified section
        # Note: The section metadata may be stored differently in the payload
        # This assumes section is stored as a top-level field in the payload
        filtered_chunks = []
        for chunk in retrieved_chunks:
            # Check if the section field in metadata matches the filter
            if chunk.metadata and chunk.metadata.get("section") == section_filter:
                filtered_chunks.append(chunk)
            # Also check if the source_url contains the section in its path
            elif section_filter in chunk.source_url:
                filtered_chunks.append(chunk)

        logger.info(f"Section filtering validation: {len(filtered_chunks)}/{len(retrieved_chunks)} results from section {section_filter}")
        return filtered_chunks

    def _convert_expected_to_chunks(self, expected_results: List[Dict[str, Any]]) -> List[RetrievedChunk]:
        """Convert expected results to RetrievedChunk format for comparison."""
        chunks = []
        for i, expected in enumerate(expected_results):
            chunk = RetrievedChunk(
                id=f"expected-{i}",
                content=expected.get("exact_content", ""),
                source_url=expected.get("source_url", ""),
                page_title=expected.get("expected_title", ""),
                similarity_score=1.0,  # Expected results are considered perfect
                chunk_order=0,
                retrieval_rank=i+1
            )
            chunks.append(chunk)
        return chunks

    def run_comprehensive_validation(self) -> Dict[str, Any]:
        """
        Run a comprehensive validation using the test dataset.

        Returns:
            Dictionary containing validation results and metrics
        """
        logger.info("Starting comprehensive validation...")

        # Get all semantic queries for testing
        semantic_queries = self.dataset.get_semantic_queries()

        # Execute validation
        accuracy_metric = self.validate_retrieval_accuracy(semantic_queries)

        # Compile results
        results = {
            "validation_run_at": "2026-01-05T10:00:00Z",  # In a real implementation, use datetime.now()
            "total_queries": len(semantic_queries),
            "accuracy_metrics": {
                "precision": accuracy_metric.precision,
                "recall": accuracy_metric.recall,
                "f1_score": accuracy_metric.f1_score,
                "mrr": accuracy_metric.mrr,
                "hit_rate": accuracy_metric.hit_rate,
                "mean_similarity": accuracy_metric.mean_similarity
            },
            "validation_passed": accuracy_metric.precision >= 0.7 and accuracy_metric.recall >= 0.7
        }

        logger.info(f"Comprehensive validation completed. Passed: {results['validation_passed']}")
        return results

    def validate_content_consistency(self, retrieved_chunks: List[RetrievedChunk]) -> Dict[str, Any]:
        """
        Validate the consistency of retrieved content with original sources.

        Args:
            retrieved_chunks: List of retrieved chunks to validate

        Returns:
            Dictionary with consistency validation results
        """
        logger.info(f"Starting content consistency validation for {len(retrieved_chunks)} chunks")

        # Use the consistency validator to validate all chunks
        results = self.consistency_validator.validate_multiple_chunks_consistency(retrieved_chunks)

        logger.info(f"Content consistency validation completed: {results['consistent_chunks']}/{results['total_chunks']} consistent")
        return results

    def validate_retrieved_chunks_comprehensive(self, retrieval_query: RetrievalQuery, retrieved_chunks: List[RetrievedChunk]) -> ValidationResult:
        """
        Perform comprehensive validation including semantic relevance, metadata filtering, and content consistency.

        Args:
            retrieval_query: The original retrieval query
            retrieved_chunks: List of retrieved chunks to validate

        Returns:
            ValidationResult with comprehensive validation results
        """
        # Perform the basic validation
        basic_validation = self._validate_retrieved_chunks(retrieval_query, retrieved_chunks)

        # Perform content consistency validation
        consistency_results = self.consistency_validator.perform_comprehensive_consistency_check(retrieved_chunks)

        # Update the validation result with consistency information
        basic_validation.accuracy_metrics.update({
            "content_consistency_rate": consistency_results["overall_consistency_rate"],
            "content_consistency_score": consistency_results["overall_consistency_score"],
            "source_validation_rate": consistency_results["source_validation"]["source_validation_rate"],
            "consistent_chunks": consistency_results["content_validation"]["consistent_chunks"],
            "total_consistent_chunks": consistency_results["content_validation"]["total_chunks"]
        })

        # Update validation notes with consistency information
        basic_validation.validation_notes += f"; Consistency: {consistency_results['overall_consistency_rate']:.2f}, Sources: {consistency_results['source_validation']['source_validation_rate']:.2f}"

        logger.info(f"Comprehensive validation completed for query {retrieval_query.id}")
        return basic_validation


def get_validation_engine() -> ValidationEngine:
    """Get a singleton instance of the validation engine."""
    return ValidationEngine()