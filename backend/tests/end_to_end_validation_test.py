"""
End-to-end integration tests for the complete validation pipeline.

These tests verify the entire flow from query input to validation result output.
"""
import unittest
from unittest.mock import patch, MagicMock
from src.validation.engine import ValidationEngine
from src.validation.models import RetrievalQuery
from src.validation.reporter import ValidationReporter
from src.tests.datasets import ValidationTestDataset


class TestEndToEndValidation(unittest.TestCase):
    """End-to-end test cases for the complete validation pipeline."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Mock all external dependencies to isolate the validation pipeline
        self.patches = []

        # Mock Qdrant connection
        qdrant_patcher = patch('src.validation.engine.get_qdrant_connection')
        self.mock_qdrant_conn = qdrant_patcher.start()
        self.mock_conn_instance = MagicMock()
        self.mock_qdrant_conn.return_value = self.mock_conn_instance
        self.patches.append(qdrant_patcher)

        # Mock search functionality
        search_patcher = patch('src.validation.engine.get_similarity_search')
        self.mock_search = search_patcher.start()
        self.mock_search_instance = MagicMock()
        self.mock_search.return_value = self.mock_search_instance
        self.patches.append(search_patcher)

        # Mock validation metrics
        metrics_patcher = patch('src.validation.engine.get_validation_metrics')
        self.mock_metrics = metrics_patcher.start()
        self.mock_metrics_instance = MagicMock()
        self.mock_metrics.return_value = self.mock_metrics_instance
        self.patches.append(metrics_patcher)

        # Mock test dataset
        dataset_patcher = patch('src.validation.engine.get_test_dataset')
        self.mock_dataset = dataset_patcher.start()
        self.mock_dataset_instance = MagicMock()
        self.mock_dataset.return_value = self.mock_dataset_instance
        self.patches.append(dataset_patcher)

        # Mock consistency validator
        consistency_patcher = patch('src.validation.engine.get_content_consistency_validator')
        self.mock_consistency = consistency_patcher.start()
        self.mock_consistency_instance = MagicMock()
        self.mock_consistency.return_value = self.mock_consistency_instance
        self.patches.append(consistency_patcher)

    def tearDown(self):
        """Clean up after each test method."""
        for patcher in self.patches:
            patcher.stop()

    def test_end_to_end_validation_pipeline_semantic_query(self):
        """Test the complete end-to-end validation pipeline for semantic query validation."""
        # Setup mock data
        test_query = RetrievalQuery(
            query_text="What are the key components of humanoid robotics?",
            test_scenario="semantic validation",
            expected_results=[
                {"content_keywords": ["actuators", "sensors", "control systems"]}
            ]
        )

        # Mock the search results
        mock_chunk = MagicMock()
        mock_chunk.id = "test-chunk-1"
        mock_chunk.content = "The key components of humanoid robotics include actuators, sensors, and control systems."
        mock_chunk.source_url = "https://humanoidai.vercel.app/docs/components"
        mock_chunk.page_title = "Key Components of Humanoid Robotics"
        mock_chunk.similarity_score = 0.85
        mock_chunk.chunk_order = 1
        mock_chunk.retrieval_rank = 1

        self.mock_search_instance.search_by_text.return_value = [mock_chunk]

        # Mock validation metrics
        self.mock_metrics_instance.calculate_precision.return_value = 1.0
        self.mock_metrics_instance.calculate_recall.return_value = 0.9
        self.mock_metrics_instance.calculate_f1_score.return_value = 0.95
        self.mock_metrics_instance.calculate_mrr.return_value = 1.0
        self.mock_metrics_instance.calculate_hit_rate.return_value = 1.0
        self.mock_metrics_instance.calculate_mean_similarity.return_value = 0.85

        # Mock consistency validation
        self.mock_consistency_instance.perform_comprehensive_consistency_check.return_value = {
            "overall_consistency_rate": 1.0,
            "overall_consistency_score": 0.95,
            "source_validation": {"source_validation_rate": 1.0},
            "content_validation": {
                "total_chunks": 1,
                "consistent_chunks": 1,
                "inconsistent_chunks": 0,
                "average_similarity": 0.95
            }
        }

        # Create and run the validation engine
        engine = ValidationEngine()

        # Execute the single query
        result = engine.execute_single_query(test_query)

        # Verify the result
        self.assertIsNotNone(result)
        self.assertTrue(result.is_relevant)
        self.assertGreater(result.relevance_score, 0.5)
        self.assertEqual(len(result.retrieved_chunks), 1)
        self.assertIn("actuators", result.retrieved_chunks[0].content.lower())
        self.assertIn("sensors", result.retrieved_chunks[0].content.lower())

        # Verify the validation metrics were applied
        self.assertIn("precision", result.accuracy_metrics)
        self.assertIn("content_consistency_rate", result.accuracy_metrics)

    def test_end_to_end_validation_pipeline_with_metadata_filtering(self):
        """Test the complete end-to-end validation pipeline with metadata filtering."""
        # Setup mock data with metadata filters
        test_query = RetrievalQuery(
            query_text="What are the specifications of the robot actuators?",
            test_scenario="metadata filtering validation",
            metadata_filters={"source_url": "https://humanoidai.vercel.app/docs/actuator-specs"},
            expected_results=[
                {"content_keywords": ["actuator", "specifications", "torque", "power"]}
            ]
        )

        # Mock the search results with proper metadata filtering
        mock_chunk = MagicMock()
        mock_chunk.id = "filtered-chunk-1"
        mock_chunk.content = "Actuator specifications include maximum torque of 100 Nm and power consumption of 50W."
        mock_chunk.source_url = "https://humanoidai.vercel.app/docs/actuator-specs"
        mock_chunk.page_title = "Actuator Specifications"
        mock_chunk.similarity_score = 0.92
        mock_chunk.chunk_order = 0
        mock_chunk.retrieval_rank = 1
        mock_chunk.metadata = {"section": "technical-specs", "module": "actuation"}

        self.mock_search_instance.search_by_text.return_value = [mock_chunk]

        # Mock validation metrics
        self.mock_metrics_instance.calculate_precision.return_value = 0.95
        self.mock_metrics_instance.calculate_recall.return_value = 0.9
        self.mock_metrics_instance.calculate_f1_score.return_value = 0.92
        self.mock_metrics_instance.calculate_accuracy_metrics.return_value = MagicMock(
            precision=0.95, recall=0.9, f1_score=0.92, mrr=0.9, hit_rate=1.0, mean_similarity=0.92
        )

        # Create and run the validation engine
        engine = ValidationEngine()

        # Execute the single query with metadata filtering
        result = engine.execute_single_query(test_query)

        # Verify the result includes metadata filtering
        self.assertIsNotNone(result)
        self.assertTrue(result.is_relevant)
        self.assertGreater(result.relevance_score, 0.5)
        self.assertEqual(len(result.retrieved_chunks), 1)

        # Verify the chunk comes from the expected URL (filtering worked)
        self.assertEqual(result.retrieved_chunks[0].source_url, "https://humanoidai.vercel.app/docs/actuator-specs")
        self.assertIn("torque", result.retrieved_chunks[0].content.lower())
        self.assertIn("power", result.retrieved_chunks[0].content.lower())

    def test_end_to_end_validation_pipeline_batch_processing(self):
        """Test the complete end-to-end validation pipeline with batch processing."""
        # Setup mock queries
        test_queries = [
            RetrievalQuery(
                query_text="Query 1 about robotics components",
                test_scenario="batch validation",
                expected_results=[{"content_keywords": ["component"]}]
            ),
            RetrievalQuery(
                query_text="Query 2 about control systems",
                test_scenario="batch validation",
                expected_results=[{"content_keywords": ["control"]}]
            )
        ]

        # Mock search results for each query
        self.mock_search_instance.search_by_text.side_effect = [
            [MagicMock(**{
                'id': 'chunk1', 'content': 'Robotics components include actuators and sensors',
                'source_url': 'url1', 'page_title': 'Components', 'similarity_score': 0.85,
                'chunk_order': 0, 'retrieval_rank': 1
            })],
            [MagicMock(**{
                'id': 'chunk2', 'content': 'Control systems manage robot behavior',
                'source_url': 'url2', 'page_title': 'Controls', 'similarity_score': 0.88,
                'chunk_order': 0, 'retrieval_rank': 1
            })]
        ]

        # Mock validation metrics
        self.mock_metrics_instance.calculate_accuracy_metrics.return_value = MagicMock(
            precision=0.9, recall=0.85, f1_score=0.87, mrr=0.88, hit_rate=1.0, mean_similarity=0.865
        )

        # Create and run the validation engine
        engine = ValidationEngine()

        # Execute batch queries
        results = engine.execute_batch_queries(test_queries, batch_size=2)

        # Verify the results
        self.assertEqual(len(results), 2)
        for result in results:
            self.assertIsNotNone(result)
            self.assertTrue(result.is_relevant)
            self.assertGreater(result.relevance_score, 0.5)
            self.assertEqual(len(result.retrieved_chunks), 1)

        # Verify comprehensive validation
        accuracy_metrics = engine.validate_retrieval_accuracy(test_queries)
        self.assertIsNotNone(accuracy_metrics)
        self.assertGreaterEqual(accuracy_metrics.precision, 0.8)

    def test_end_to_end_validation_reporting(self):
        """Test the complete end-to-end validation with reporting."""
        # Setup mock data
        test_query = RetrievalQuery(
            query_text="Comprehensive validation query",
            test_scenario="full pipeline validation",
            expected_results=[{"content_keywords": ["comprehensive", "validation"]}]
        )

        # Mock the search results
        mock_chunk = MagicMock()
        mock_chunk.id = "report-chunk-1"
        mock_chunk.content = "Comprehensive validation includes semantic, metadata, and consistency validation."
        mock_chunk.source_url = "https://humanoidai.vercel.app/docs/validation"
        mock_chunk.page_title = "Comprehensive Validation"
        mock_chunk.similarity_score = 0.9
        mock_chunk.chunk_order = 0
        mock_chunk.retrieval_rank = 1

        self.mock_search_instance.search_by_text.return_value = [mock_chunk]

        # Mock validation metrics
        self.mock_metrics_instance.calculate_accuracy_metrics.return_value = MagicMock(
            precision=0.92, recall=0.91, f1_score=0.915, mrr=0.9, hit_rate=1.0, mean_similarity=0.9
        )

        # Mock consistency validation
        self.mock_consistency_instance.perform_comprehensive_consistency_check.return_value = {
            "overall_consistency_rate": 1.0,
            "overall_consistency_score": 0.95,
            "source_validation": {"source_validation_rate": 1.0},
            "content_validation": {
                "total_chunks": 1,
                "consistent_chunks": 1,
                "inconsistent_chunks": 0,
                "average_similarity": 0.95
            }
        }

        # Create validation engine and execute
        engine = ValidationEngine()
        result = engine.execute_single_query(test_query)

        # Create reporter and generate report
        reporter = ValidationReporter()
        simple_report = reporter.generate_simple_report([result])
        detailed_report = reporter.generate_detailed_report([result])
        comprehensive_report = reporter.generate_comprehensive_report()

        # Verify reports are generated properly
        self.assertIsNotNone(simple_report)
        self.assertIn("report_generated_at", simple_report)
        self.assertIn("summary", simple_report)

        self.assertIsNotNone(detailed_report)
        self.assertIn("report_generated_at", detailed_report)
        self.assertIn("detailed_metrics", detailed_report)
        self.assertIn("accuracy_metrics", detailed_report)

        self.assertIsNotNone(comprehensive_report)
        self.assertIn("report_generated_at", comprehensive_report)
        self.assertIn("validation_results", comprehensive_report)

    def test_end_to_end_validation_with_edge_case_detection(self):
        """Test the complete end-to-end validation pipeline detecting edge cases."""
        # Setup mock data for an edge case (short query)
        test_query = RetrievalQuery(
            query_text="AI",  # Very short query that might cause issues
            test_scenario="edge case detection",
            expected_results=[{"content_keywords": ["artificial", "intelligence"]}]
        )

        # Mock the search results
        mock_chunk = MagicMock()
        mock_chunk.id = "edge-case-chunk-1"
        mock_chunk.content = "Artificial Intelligence is a broad field."
        mock_chunk.source_url = "https://humanoidai.vercel.app/docs/ai"
        mock_chunk.page_title = "AI Overview"
        mock_chunk.similarity_score = 0.75  # Moderate similarity for edge case
        mock_chunk.chunk_order = 0
        mock_chunk.retrieval_rank = 1

        self.mock_search_instance.search_by_text.return_value = [mock_chunk]

        # Create and run the validation engine
        engine = ValidationEngine()

        # Execute the query
        result = engine.execute_single_query(test_query)

        # Use the edge case detection method to verify edge cases are caught
        edge_cases = engine.detect_edge_cases(test_query, [mock_chunk])

        # Verify that edge case detection works
        self.assertIsInstance(edge_cases, list)
        # The short query should be detected as an edge case
        short_query_detected = any("short_query" in case for case in edge_cases)
        self.assertTrue(short_query_detected, "Short query edge case should be detected")

        # Verify the result is still processed
        self.assertIsNotNone(result)
        self.assertGreaterEqual(result.relevance_score, 0.0)  # Even failed queries return results

    def test_end_to_end_validation_consistency_verification(self):
        """Test the complete end-to-end validation pipeline with consistency verification."""
        # Setup mock data
        test_query = RetrievalQuery(
            query_text="Verify consistency of this content",
            test_scenario="consistency verification",
            expected_results=[{"exact_content": "This content should remain unchanged."}]
        )

        # Mock the search results
        mock_chunk = MagicMock()
        mock_chunk.id = "consistency-chunk-1"
        mock_chunk.content = "This content should remain unchanged."
        mock_chunk.source_url = "https://humanoidai.vercel.app/docs/consistency-test"
        mock_chunk.page_title = "Consistency Test"
        mock_chunk.similarity_score = 0.95
        mock_chunk.chunk_order = 0
        mock_chunk.retrieval_rank = 1

        self.mock_search_instance.search_by_text.return_value = [mock_chunk]

        # Mock consistency validation to confirm consistency
        self.mock_consistency_instance.validate_content_consistency.return_value = (
            True,  # is_consistent
            0.98,  # similarity_score
            "Content is consistent with original source"
        )

        self.mock_consistency_instance.perform_comprehensive_consistency_check.return_value = {
            "overall_consistency_rate": 1.0,
            "overall_consistency_score": 0.98,
            "source_validation": {"source_validation_rate": 1.0},
            "content_validation": {
                "total_chunks": 1,
                "consistent_chunks": 1,
                "inconsistent_chunks": 0,
                "average_similarity": 0.98
            }
        }

        # Create and run the validation engine with comprehensive validation
        engine = ValidationEngine()
        comprehensive_result = engine.validate_retrieved_chunks_comprehensive(test_query, [mock_chunk])

        # Verify consistency verification was performed
        self.assertIsNotNone(comprehensive_result)
        self.assertIn("content_consistency_rate", comprehensive_result.accuracy_metrics)
        self.assertIn("source_validation_rate", comprehensive_result.accuracy_metrics)

        # Verify the consistency rate is high (since content matches)
        consistency_rate = comprehensive_result.accuracy_metrics["content_consistency_rate"]
        self.assertGreaterEqual(consistency_rate, 0.95)


if __name__ == '__main__':
    unittest.main()