from typing import List, Dict, Any
from src.validation.models import RetrievedChunk, AccuracyMetric
import logging

logger = logging.getLogger(__name__)

class ValidationMetrics:
    """Validation metrics calculation for retrieval accuracy assessment."""

    def __init__(self):
        pass

    def calculate_precision(self, retrieved_chunks: List[RetrievedChunk], relevant_chunks: List[RetrievedChunk]) -> float:
        """
        Calculate precision: proportion of retrieved documents that are relevant.

        Args:
            retrieved_chunks: List of chunks retrieved by the system
            relevant_chunks: List of chunks that are actually relevant

        Returns:
            Precision score (0-1)
        """
        if not retrieved_chunks:
            return 0.0

        relevant_retrieved_count = 0
        for retrieved in retrieved_chunks:
            for relevant in relevant_chunks:
                if self._chunks_match(retrieved, relevant):
                    relevant_retrieved_count += 1
                    break

        precision = relevant_retrieved_count / len(retrieved_chunks)
        return precision

    def calculate_recall(self, retrieved_chunks: List[RetrievedChunk], relevant_chunks: List[RetrievedChunk]) -> float:
        """
        Calculate recall: proportion of relevant documents that are retrieved.

        Args:
            retrieved_chunks: List of chunks retrieved by the system
            relevant_chunks: List of chunks that are actually relevant

        Returns:
            Recall score (0-1)
        """
        if not relevant_chunks:
            return 1.0 if not retrieved_chunks else 0.0

        relevant_retrieved_count = 0
        for relevant in relevant_chunks:
            for retrieved in retrieved_chunks:
                if self._chunks_match(relevant, retrieved):
                    relevant_retrieved_count += 1
                    break

        recall = relevant_retrieved_count / len(relevant_chunks)
        return recall

    def calculate_f1_score(self, precision: float, recall: float) -> float:
        """
        Calculate F1 score: harmonic mean of precision and recall.

        Args:
            precision: Precision score
            recall: Recall score

        Returns:
            F1 score (0-1)
        """
        if precision + recall == 0:
            return 0.0

        f1_score = 2 * (precision * recall) / (precision + recall)
        return f1_score

    def calculate_mrr(self, retrieved_chunks: List[RetrievedChunk], relevant_chunks: List[RetrievedChunk]) -> float:
        """
        Calculate Mean Reciprocal Rank (MRR).

        Args:
            retrieved_chunks: List of retrieved chunks in ranked order
            relevant_chunks: List of chunks that are actually relevant

        Returns:
            MRR score (0-1)
        """
        for i, retrieved in enumerate(retrieved_chunks):
            rank = i + 1  # Rank starts from 1
            for relevant in relevant_chunks:
                if self._chunks_match(retrieved, relevant):
                    # Return reciprocal of the rank of the first relevant result
                    return 1.0 / rank

        # If no relevant chunks are found, return 0
        return 0.0

    def calculate_hit_rate(self, retrieval_results: List[List[RetrievedChunk]], relevant_results: List[List[RetrievedChunk]]) -> float:
        """
        Calculate hit rate: proportion of queries that return at least one relevant result.

        Args:
            retrieval_results: List of retrieved results for each query
            relevant_results: List of relevant results for each query

        Returns:
            Hit rate score (0-1)
        """
        if not retrieval_results:
            return 0.0

        hits = 0
        for retrieved_list, relevant_list in zip(retrieval_results, relevant_results):
            # Check if at least one relevant result was retrieved
            has_hit = False
            for retrieved in retrieved_list:
                for relevant in relevant_list:
                    if self._chunks_match(retrieved, relevant):
                        has_hit = True
                        break
                if has_hit:
                    break

            if has_hit:
                hits += 1

        hit_rate = hits / len(retrieval_results)
        return hit_rate

    def calculate_mean_similarity(self, retrieved_chunks: List[RetrievedChunk]) -> float:
        """
        Calculate mean similarity score of retrieved results.

        Args:
            retrieved_chunks: List of retrieved chunks with similarity scores

        Returns:
            Mean similarity score
        """
        if not retrieved_chunks:
            return 0.0

        total_similarity = sum(chunk.similarity_score for chunk in retrieved_chunks)
        mean_similarity = total_similarity / len(retrieved_chunks)
        return mean_similarity

    def _chunks_match(self, chunk1: RetrievedChunk, chunk2: RetrievedChunk, threshold: float = 0.9) -> bool:
        """
        Determine if two chunks match based on content similarity.

        Args:
            chunk1: First chunk to compare
            chunk2: Second chunk to compare
            threshold: Similarity threshold for considering chunks as matching

        Returns:
            True if chunks are considered matching, False otherwise
        """
        # For now, we'll use a simple content similarity check
        # In practice, this might involve more sophisticated text similarity measures
        content1 = chunk1.content.strip().lower()
        content2 = chunk2.content.strip().lower()

        # Simple exact match check first
        if content1 == content2:
            return True

        # For more nuanced matching, we might want to use text similarity algorithms
        # For now, return False if not exact match
        # TODO: Implement more sophisticated text similarity matching
        return False

    def calculate_accuracy_metrics(self, retrieval_results: List[List[RetrievedChunk]],
                                 expected_results: List[List[RetrievedChunk]]) -> AccuracyMetric:
        """
        Calculate comprehensive accuracy metrics.

        Args:
            retrieval_results: List of retrieved results for each query
            expected_results: List of expected results for each query

        Returns:
            AccuracyMetric object with all calculated metrics
        """
        if not retrieval_results or not expected_results:
            return AccuracyMetric()

        # Calculate metrics across all query results
        total_precision = 0
        total_recall = 0
        total_queries = len(retrieval_results)

        all_retrieved = []
        all_expected = []

        for retrieved_list, expected_list in zip(retrieval_results, expected_results):
            all_retrieved.extend(retrieved_list)
            all_expected.extend(expected_list)

            precision = self.calculate_precision(retrieved_list, expected_list)
            recall = self.calculate_recall(retrieved_list, expected_list)

            total_precision += precision
            total_recall += recall

        # Calculate averages
        avg_precision = total_precision / total_queries if total_queries > 0 else 0
        avg_recall = total_recall / total_queries if total_queries > 0 else 0

        # Calculate overall F1 score
        f1_score = self.calculate_f1_score(avg_precision, avg_recall)

        # Calculate MRR across all results
        mrr = 0
        if retrieval_results and expected_results:
            for retrieved_list, expected_list in zip(retrieval_results, expected_results):
                mrr += self.calculate_mrr(retrieved_list, expected_list)
            mrr /= len(retrieval_results)

        # Calculate hit rate
        hit_rate = self.calculate_hit_rate(retrieval_results, expected_results)

        # Calculate mean similarity
        all_retrieved_flat = [chunk for sublist in retrieval_results for chunk in sublist]
        mean_similarity = self.calculate_mean_similarity(all_retrieved_flat)

        # Calculate total relevant retrieved
        total_relevant_retrieved = 0
        for retrieved_list, expected_list in zip(retrieval_results, expected_results):
            for retrieved in retrieved_list:
                for expected in expected_list:
                    if self._chunks_match(retrieved, expected):
                        total_relevant_retrieved += 1
                        break

        accuracy_metric = AccuracyMetric(
            precision=avg_precision,
            recall=avg_recall,
            f1_score=f1_score,
            mrr=mrr,
            hit_rate=hit_rate,
            mean_similarity=mean_similarity,
            total_queries=total_queries,
            total_relevant_retrieved=total_relevant_retrieved,
            total_expected_retrieved=len([chunk for sublist in expected_results for chunk in sublist])
        )

        return accuracy_metric


def get_validation_metrics() -> ValidationMetrics:
    """Get a singleton instance of the validation metrics calculator."""
    return ValidationMetrics()