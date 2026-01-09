"""
Integration tests for the validation pipeline.

These tests verify the end-to-end functionality of the retrieval validation system.
"""
import unittest
from unittest.mock import patch, MagicMock
from src.validation.engine import ValidationEngine
from src.validation.models import RetrievalQuery, RetrievedChunk


class TestValidationIntegration(unittest.TestCase):
    """Integration test cases for the validation pipeline."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Mock all dependencies to isolate the validation engine functionality
        self.qdrant_patcher = patch('src.validation.engine.get_qdrant_connection')
        self.mock_qdrant_conn = self.qdrant_patcher.start()
        self.mock_conn_instance = MagicMock()
        self.mock_qdrant_conn.return_value = self.mock_conn_instance

        self.search_patcher = patch('src.validation.engine.get_similarity_search')
        self.mock_search = self.search_patcher.start()
        self.mock_search_instance = MagicMock()
        self.mock_search.return_value = self.mock_search_instance

        self.metrics_patcher = patch('src.validation.engine.get_validation_metrics')
        self.mock_metrics = self.metrics_patcher.start()
        self.mock_metrics_instance = MagicMock()
        self.mock_metrics.return_value = self.mock_metrics_instance

        self.dataset_patcher = patch('src.validation.engine.get_test_dataset')
        self.mock_dataset = self.dataset_patcher.start()
        self.mock_dataset_instance = MagicMock()
        self.mock_dataset.return_value = self.mock_dataset_instance

        self.consistency_patcher = patch('src.validation.engine.get_content_consistency_validator')
        self.mock_consistency = self.consistency_patcher.start()
        self.mock_consistency_instance = MagicMock()
        self.mock_consistency.return_value = self.mock_consistency_instance

        # Create validation engine instance
        self.engine = ValidationEngine()

    def tearDown(self):
        """Clean up after each test method."""
        self.qdrant_patcher.stop()
        self.search_patcher.stop()
        self.metrics_patcher.stop()
        self.dataset_patcher.stop()
        self.consistency_patcher.stop()

    def test_execute_single_query_success(self):
        """Test successful execution of a single retrieval query."""
        # Create a test query
        test_query = RetrievalQuery(
            query_text="What are the key components of humanoid robotics?",
            expected_results=[{"content_keywords": ["actuators", "sensors"]}]
        )

        # Mock the search to return some results
        mock_chunk = RetrievedChunk(
            content="The key components of humanoid robotics include actuators and sensors.",
            source_url="https://example.com",
            page_title="Robot Components",
            similarity_score=0.85
        )
        self.mock_search_instance.search_by_text.return_value = [mock_chunk]

        # Mock the validation metrics
        self.mock_metrics_instance.calculate_precision.return_value = 1.0
        self.mock_metrics_instance.calculate_recall.return_value = 0.8
        self.mock_metrics_instance.calculate_f1_score.return_value = 0.89

        # Execute the query
        result = self.engine.execute_single_query(test_query)

        # Verify the result
        self.assertIsNotNone(result)
        self.assertTrue(result.is_relevant)
        self.assertGreater(result.relevance_score, 0.5)
        self.assertEqual(len(result.retrieved_chunks), 1)
        self.assertIn("actuators", result.retrieved_chunks[0].content.lower())

    def test_execute_single_query_empty_results(self):
        """Test execution of a query that returns no results."""
        # Create a test query
        test_query = RetrievalQuery(
            query_text="A very specific query with no matches",
            expected_results=[]
        )

        # Mock the search to return no results
        self.mock_search_instance.search_by_text.return_value = []

        # Execute the query
        result = self.engine.execute_single_query(test_query)

        # Verify the result
        self.assertIsNotNone(result)
        self.assertFalse(result.is_relevant)
        self.assertEqual(result.relevance_score, 0.0)
        self.assertEqual(len(result.retrieved_chunks), 0)
        self.assertIn("No chunks retrieved", result.validation_notes)

    def test_execute_batch_queries(self):
        """Test execution of a batch of queries."""
        # Create test queries
        queries = [
            RetrievalQuery(query_text="Query 1", expected_results=[]),
            RetrievalQuery(query_text="Query 2", expected_results=[])
        ]

        # Mock the search to return results for each query
        self.mock_search_instance.search_by_text.side_effect = [
            [RetrievedChunk(content="Result for query 1", source_url="url1", similarity_score=0.8)],
            [RetrievedChunk(content="Result for query 2", source_url="url2", similarity_score=0.9)]
        ]

        # Execute batch queries
        results = self.engine.execute_batch_queries(queries, batch_size=2)

        # Verify the results
        self.assertEqual(len(results), 2)
        for result in results:
            self.assertIsNotNone(result)
            self.assertTrue(result.is_relevant)

    def test_validate_retrieval_accuracy(self):
        """Test retrieval accuracy validation."""
        # Create test queries
        queries = [
            RetrievalQuery(query_text="Test query 1", expected_results=[{"content_keywords": ["keyword1"]}]),
            RetrievalQuery(query_text="Test query 2", expected_results=[{"content_keywords": ["keyword2"]}])
        ]

        # Mock the search to return appropriate results
        self.mock_search_instance.search_by_text.side_effect = [
            [RetrievedChunk(content="Content with keyword1", source_url="url1", similarity_score=0.8)],
            [RetrievedChunk(content="Content with keyword2", source_url="url2", similarity_score=0.9)]
        ]

        # Mock the metrics calculation
        from src.validation.models import AccuracyMetric
        mock_accuracy = AccuracyMetric(
            precision=0.85,
            recall=0.80,
            f1_score=0.82,
            mrr=0.88,
            hit_rate=0.95,
            mean_similarity=0.85
        )
        self.mock_metrics_instance.calculate_accuracy_metrics.return_value = mock_accuracy

        # Validate retrieval accuracy
        accuracy_result = self.engine.validate_retrieval_accuracy(queries)

        # Verify the accuracy result
        self.assertIsNotNone(accuracy_result)
        self.assertEqual(accuracy_result.precision, 0.85)
        self.assertEqual(accuracy_result.recall, 0.80)
        self.assertEqual(accuracy_result.f1_score, 0.82)

    def test_validate_url_filtering(self):
        """Test URL-based filtering validation."""
        url_filter = "https://example.com/specific-page"
        query_text = "Query about specific topic"

        # Mock the search to return results from the specified URL
        mock_chunk = RetrievedChunk(
            content="Content from specific page",
            source_url=url_filter,
            page_title="Specific Page",
            similarity_score=0.85
        )
        self.mock_search_instance.search_by_text.return_value = [mock_chunk]

        # Validate URL filtering
        results = self.engine.validate_url_filtering(url_filter, query_text)

        # Verify the results
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].source_url, url_filter)

    def test_validate_section_filtering(self):
        """Test section-based filtering validation."""
        section_filter = "robot-components"
        query_text = "Query about robot components"

        # Mock the search to return results from the specified section
        mock_chunk = RetrievedChunk(
            content="Content from robot components section",
            source_url=f"https://example.com/docs/{section_filter}/page",
            page_title="Components Page",
            similarity_score=0.8,
            metadata={"section": section_filter}
        )
        self.mock_search_instance.search_by_text.return_value = [mock_chunk]

        # Validate section filtering
        results = self.engine.validate_section_filtering(section_filter, query_text)

        # Verify the results
        self.assertEqual(len(results), 1)
        self.assertIn(section_filter, results[0].source_url)

    def test_validate_content_consistency_integration(self):
        """Test content consistency validation integration."""
        # Create test chunks
        test_chunks = [
            RetrievedChunk(
                id="chunk1",
                content="Original content from source",
                source_url="https://example.com/test1",
                page_title="Test Page 1"
            ),
            RetrievedChunk(
                id="chunk2",
                content="Another piece of content",
                source_url="https://example.com/test2",
                page_title="Test Page 2"
            )
        ]

        # Mock the consistency validator
        mock_consistency_result = {
            "total_chunks": 2,
            "consistent_chunks": 2,
            "inconsistent_chunks": 0,
            "average_similarity": 0.9,
            "consistency_rate": 1.0,
            "validation_results": [
                {"chunk_id": "chunk1", "is_consistent": True, "similarity_score": 0.95, "notes": "Consistent"},
                {"chunk_id": "chunk2", "is_consistent": True, "similarity_score": 0.85, "notes": "Consistent"}
            ]
        }
        self.mock_consistency_instance.validate_multiple_chunks_consistency.return_value = mock_consistency_result

        # Validate content consistency
        results = self.engine.validate_content_consistency(test_chunks)

        # Verify the results
        self.assertEqual(results["total_chunks"], 2)
        self.assertEqual(results["consistent_chunks"], 2)
        self.assertEqual(results["consistency_rate"], 1.0)

    def test_validate_retrieved_chunks_comprehensive(self):
        """Test comprehensive validation of retrieved chunks."""
        # Create test query and chunks
        test_query = RetrievalQuery(
            query_text="Comprehensive test query",
            expected_results=[{"content_keywords": ["test", "comprehensive"]}]
        )

        test_chunks = [
            RetrievedChunk(
                id="comp-chunk1",
                content="Comprehensive test content",
                source_url="https://example.com/test",
                page_title="Test Page",
                similarity_score=0.85
            )
        ]

        # Mock the basic validation
        from src.validation.models import ValidationResult
        basic_result = ValidationResult(
            query_id=test_query.id,
            retrieved_chunks=test_chunks,
            is_relevant=True,
            relevance_score=0.9,
            accuracy_metrics={},
            validation_notes="Basic validation passed"
        )

        with patch.object(self.engine, '_validate_retrieved_chunks', return_value=basic_result):
            # Mock the consistency validation
            consistency_result = {
                "overall_consistency_rate": 1.0,
                "overall_consistency_score": 0.9,
                "source_validation": {"source_validation_rate": 1.0}
            }
            self.mock_consistency_instance.perform_comprehensive_consistency_check.return_value = consistency_result

            # Perform comprehensive validation
            result = self.engine.validate_retrieved_chunks_comprehensive(test_query, test_chunks)

            # Verify the result
            self.assertIsNotNone(result)
            self.assertTrue(result.is_relevant)
            self.assertIn("Consistency:", result.validation_notes)

    def test_run_comprehensive_validation(self):
        """Test comprehensive validation run."""
        # Create mock queries
        mock_queries = [
            RetrievalQuery(query_text="Test query 1", expected_results=[]),
            RetrievalQuery(query_text="Test query 2", expected_results=[])
        ]
        self.mock_dataset_instance.get_semantic_queries.return_value = mock_queries

        # Mock the validation process
        self.mock_search_instance.search_by_text.side_effect = [
            [RetrievedChunk(content="Result 1", source_url="url1", similarity_score=0.8)],
            [RetrievedChunk(content="Result 2", source_url="url2", similarity_score=0.9)]
        ]

        # Mock the accuracy metrics
        from src.validation.models import AccuracyMetric
        mock_accuracy = AccuracyMetric(
            precision=0.85,
            recall=0.80,
            f1_score=0.82,
            mrr=0.88,
            hit_rate=0.95,
            mean_similarity=0.85,
            total_queries=2
        )
        self.mock_metrics_instance.calculate_accuracy_metrics.return_value = mock_accuracy

        # Run comprehensive validation
        results = self.engine.run_comprehensive_validation()

        # Verify the results
        self.assertIsNotNone(results)
        self.assertIn("validation_run_at", results)
        self.assertIn("accuracy_metrics", results)
        self.assertIn("validation_passed", results)
        self.assertEqual(results["total_queries"], 2)
        self.assertTrue(isinstance(results["validation_passed"], bool))


if __name__ == '__main__':
    unittest.main()