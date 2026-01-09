"""
Unit tests for the metrics module.

These tests verify the functionality of the validation metrics calculation components.
"""
import unittest
from src.validation.metrics import ValidationMetrics
from src.validation.models import RetrievedChunk


class TestMetrics(unittest.TestCase):
    """Test cases for the metrics module."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.metrics = ValidationMetrics()

    def test_calculate_precision_perfect_match(self):
        """Test precision calculation with perfect match."""
        # Create retrieved chunks
        retrieved = [
            RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.9),
            RetrievedChunk(id="2", content="Content 2", source_url="url2", similarity_score=0.8)
        ]

        # Create relevant chunks (same as retrieved)
        relevant = [
            RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.9),
            RetrievedChunk(id="2", content="Content 2", source_url="url2", similarity_score=0.8)
        ]

        precision = self.metrics.calculate_precision(retrieved, relevant)
        self.assertEqual(precision, 1.0)

    def test_calculate_precision_partial_match(self):
        """Test precision calculation with partial match."""
        # Create retrieved chunks
        retrieved = [
            RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.9),
            RetrievedChunk(id="2", content="Content 2", source_url="url2", similarity_score=0.8),
            RetrievedChunk(id="3", content="Content 3", source_url="url3", similarity_score=0.7)
        ]

        # Create relevant chunks (only 2 of 3 match)
        relevant = [
            RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.9),
            RetrievedChunk(id="3", content="Content 3", source_url="url3", similarity_score=0.7)
        ]

        precision = self.metrics.calculate_precision(retrieved, relevant)
        self.assertEqual(precision, 2/3)  # 2 relevant out of 3 retrieved

    def test_calculate_precision_no_match(self):
        """Test precision calculation with no match."""
        # Create retrieved chunks
        retrieved = [
            RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.9)
        ]

        # Create relevant chunks (different IDs)
        relevant = [
            RetrievedChunk(id="2", content="Content 2", source_url="url2", similarity_score=0.8)
        ]

        precision = self.metrics.calculate_precision(retrieved, relevant)
        self.assertEqual(precision, 0.0)

    def test_calculate_precision_empty_retrieved(self):
        """Test precision calculation with empty retrieved list."""
        retrieved = []
        relevant = [
            RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.9)
        ]

        precision = self.metrics.calculate_precision(retrieved, relevant)
        self.assertEqual(precision, 0.0)

    def test_calculate_recall_perfect_match(self):
        """Test recall calculation with perfect match."""
        # Create retrieved chunks
        retrieved = [
            RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.9),
            RetrievedChunk(id="2", content="Content 2", source_url="url2", similarity_score=0.8)
        ]

        # Create relevant chunks (same as retrieved)
        relevant = [
            RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.9),
            RetrievedChunk(id="2", content="Content 2", source_url="url2", similarity_score=0.8)
        ]

        recall = self.metrics.calculate_recall(retrieved, relevant)
        self.assertEqual(recall, 1.0)

    def test_calculate_recall_partial_match(self):
        """Test recall calculation with partial match."""
        # Create retrieved chunks (only 2 of 3 relevant items retrieved)
        retrieved = [
            RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.9),
            RetrievedChunk(id="3", content="Content 3", source_url="url3", similarity_score=0.7)
        ]

        # Create relevant chunks (3 items that should be retrieved)
        relevant = [
            RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.9),
            RetrievedChunk(id="2", content="Content 2", source_url="url2", similarity_score=0.8),
            RetrievedChunk(id="3", content="Content 3", source_url="url3", similarity_score=0.7)
        ]

        recall = self.metrics.calculate_recall(retrieved, relevant)
        self.assertEqual(recall, 2/3)  # 2 retrieved out of 3 relevant

    def test_calculate_recall_no_match(self):
        """Test recall calculation with no match."""
        # Create retrieved chunks
        retrieved = [
            RetrievedChunk(id="4", content="Content 4", source_url="url4", similarity_score=0.6)
        ]

        # Create relevant chunks (none match)
        relevant = [
            RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.9)
        ]

        recall = self.metrics.calculate_recall(retrieved, relevant)
        self.assertEqual(recall, 0.0)

    def test_calculate_recall_empty_relevant(self):
        """Test recall calculation with empty relevant list."""
        retrieved = [
            RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.9)
        ]
        relevant = []

        recall = self.metrics.calculate_recall(retrieved, relevant)
        self.assertEqual(recall, 1.0)  # Per specification: return 1.0 if relevant is empty

    def test_calculate_f1_score_perfect(self):
        """Test F1 score calculation with perfect precision and recall."""
        f1_score = self.metrics.calculate_f1_score(1.0, 1.0)
        self.assertEqual(f1_score, 1.0)

    def test_calculate_f1_score_imperfect(self):
        """Test F1 score calculation with imperfect precision and recall."""
        # F1 = 2 * (precision * recall) / (precision + recall)
        # F1 = 2 * (0.8 * 0.6) / (0.8 + 0.6) = 2 * 0.48 / 1.4 = 0.96 / 1.4 ≈ 0.6857
        f1_score = self.metrics.calculate_f1_score(0.8, 0.6)
        self.assertAlmostEqual(f1_score, 0.6857, places=4)

    def test_calculate_f1_score_zero_precision_recall(self):
        """Test F1 score calculation when both precision and recall are zero."""
        f1_score = self.metrics.calculate_f1_score(0.0, 0.0)
        self.assertEqual(f1_score, 0.0)

    def test_calculate_mrr_first_result_relevant(self):
        """Test MRR calculation when first result is relevant."""
        retrieved = [
            RetrievedChunk(id="relevant1", content="Relevant content 1", source_url="url1", similarity_score=0.9),
            RetrievedChunk(id="irrelevant", content="Irrelevant content", source_url="url2", similarity_score=0.3),
            RetrievedChunk(id="relevant2", content="Relevant content 2", source_url="url3", similarity_score=0.8)
        ]

        # Only the first item is relevant to itself
        relevant = [
            RetrievedChunk(id="relevant1", content="Relevant content 1", source_url="url1", similarity_score=0.9)
        ]

        mrr = self.metrics.calculate_mrr(retrieved, relevant)
        # Rank of first relevant result is 1, so MRR = 1/1 = 1.0
        self.assertEqual(mrr, 1.0)

    def test_calculate_mrr_second_result_relevant(self):
        """Test MRR calculation when second result is relevant."""
        retrieved = [
            RetrievedChunk(id="irrelevant", content="Irrelevant content", source_url="url1", similarity_score=0.3),
            RetrievedChunk(id="relevant", content="Relevant content", source_url="url2", similarity_score=0.9),
            RetrievedChunk(id="other", content="Other content", source_url="url3", similarity_score=0.7)
        ]

        # The relevant item is the second one
        relevant = [
            RetrievedChunk(id="relevant", content="Relevant content", source_url="url2", similarity_score=0.9)
        ]

        mrr = self.metrics.calculate_mrr(retrieved, relevant)
        # Rank of first relevant result is 2, so MRR = 1/2 = 0.5
        self.assertEqual(mrr, 0.5)

    def test_calculate_mrr_no_relevant_results(self):
        """Test MRR calculation when no relevant results are found."""
        retrieved = [
            RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.5),
            RetrievedChunk(id="2", content="Content 2", source_url="url2", similarity_score=0.4)
        ]

        # No relevant results in the list
        relevant = [
            RetrievedChunk(id="3", content="Relevant content", source_url="url3", similarity_score=0.9)
        ]

        mrr = self.metrics.calculate_mrr(retrieved, relevant)
        # No relevant results found, so MRR = 0
        self.assertEqual(mrr, 0.0)

    def test_calculate_hit_rate_all_hits(self):
        """Test hit rate calculation when all queries return relevant results."""
        retrieval_results = [
            [RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.9)],
            [RetrievedChunk(id="2", content="Content 2", source_url="url2", similarity_score=0.8)]
        ]

        relevant_results = [
            [RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.9)],
            [RetrievedChunk(id="2", content="Content 2", source_url="url2", similarity_score=0.8)]
        ]

        hit_rate = self.metrics.calculate_hit_rate(retrieval_results, relevant_results)
        self.assertEqual(hit_rate, 1.0)  # 2/2 queries have hits

    def test_calculate_hit_rate_partial_hits(self):
        """Test hit rate calculation when some queries return relevant results."""
        retrieval_results = [
            [RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.9)],  # Has relevant result
            [RetrievedChunk(id="2", content="Content 2", source_url="url2", similarity_score=0.3)]   # No relevant result
        ]

        relevant_results = [
            [RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.9)],
            [RetrievedChunk(id="3", content="Different content", source_url="url3", similarity_score=0.9)]  # Different from retrieved
        ]

        hit_rate = self.metrics.calculate_hit_rate(retrieval_results, relevant_results)
        self.assertEqual(hit_rate, 0.5)  # 1/2 queries have hits

    def test_calculate_hit_rate_no_hits(self):
        """Test hit rate calculation when no queries return relevant results."""
        retrieval_results = [
            [RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.3)],
            [RetrievedChunk(id="2", content="Content 2", source_url="url2", similarity_score=0.2)]
        ]

        relevant_results = [
            [RetrievedChunk(id="3", content="Relevant content 1", source_url="url3", similarity_score=0.9)],
            [RetrievedChunk(id="4", content="Relevant content 2", source_url="url4", similarity_score=0.8)]
        ]

        hit_rate = self.metrics.calculate_hit_rate(retrieval_results, relevant_results)
        self.assertEqual(hit_rate, 0.0)  # 0/2 queries have hits

    def test_calculate_mean_similarity(self):
        """Test mean similarity calculation."""
        chunks = [
            RetrievedChunk(id="1", content="Content 1", source_url="url1", similarity_score=0.9),
            RetrievedChunk(id="2", content="Content 2", source_url="url2", similarity_score=0.7),
            RetrievedChunk(id="3", content="Content 3", source_url="url3", similarity_score=0.5)
        ]

        mean_sim = self.metrics.calculate_mean_similarity(chunks)
        expected_mean = (0.9 + 0.7 + 0.5) / 3
        self.assertAlmostEqual(mean_sim, expected_mean, places=4)

    def test_calculate_mean_similarity_empty_list(self):
        """Test mean similarity calculation with empty list."""
        chunks = []
        mean_sim = self.metrics.calculate_mean_similarity(chunks)
        self.assertEqual(mean_sim, 0.0)

    def test_chunks_match_exact_content(self):
        """Test that chunks with exact content match."""
        chunk1 = RetrievedChunk(content="Exact same content", source_url="url1", similarity_score=0.9)
        chunk2 = RetrievedChunk(content="Exact same content", source_url="url1", similarity_score=0.8)

        result = self.metrics._chunks_match(chunk1, chunk2)
        self.assertTrue(result)

    def test_chunks_match_different_content(self):
        """Test that chunks with different content don't match."""
        chunk1 = RetrievedChunk(content="Different content 1", source_url="url1", similarity_score=0.9)
        chunk2 = RetrievedChunk(content="Different content 2", source_url="url2", similarity_score=0.8)

        result = self.metrics._chunks_match(chunk1, chunk2)
        self.assertFalse(result)

    def test_calculate_accuracy_metrics(self):
        """Test comprehensive accuracy metrics calculation."""
        # Create test data: 2 queries with multiple results each
        retrieval_results = [
            [
                RetrievedChunk(id="q1_r1", content="Query 1 Result 1", source_url="url1", similarity_score=0.9),
                RetrievedChunk(id="q1_r2", content="Query 1 Result 2", source_url="url2", similarity_score=0.7)
            ],
            [
                RetrievedChunk(id="q2_r1", content="Query 2 Result 1", source_url="url3", similarity_score=0.8)
            ]
        ]

        expected_results = [
            [
                RetrievedChunk(id="q1_e1", content="Query 1 Result 1", source_url="url1", similarity_score=0.9)
            ],
            [
                RetrievedChunk(id="q2_e1", content="Query 2 Result 1", source_url="url3", similarity_score=0.8)
            ]
        ]

        accuracy_metric = self.metrics.calculate_accuracy_metrics(retrieval_results, expected_results)

        # Verify the metric object was created
        self.assertIsNotNone(accuracy_metric)
        self.assertEqual(accuracy_metric.total_queries, 2)

        # Note: The actual values will depend on the _chunks_match implementation
        # which currently only does exact content matching


if __name__ == '__main__':
    unittest.main()