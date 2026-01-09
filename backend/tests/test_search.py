"""
Unit tests for the search module.

These tests verify the functionality of the similarity search components.
"""
import unittest
from unittest.mock import patch, MagicMock
from src.validation.search import SimilaritySearch
from src.validation.connection import QdrantConnection
from src.validation.models import RetrievedChunk


class TestSearch(unittest.TestCase):
    """Test cases for the search module."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Create a mock Qdrant connection
        self.mock_qdrant_conn = MagicMock(spec=QdrantConnection)
        self.mock_client = MagicMock()
        self.mock_qdrant_conn.get_client.return_value = self.mock_client
        self.mock_qdrant_conn.get_collection_name.return_value = "test_collection"

        # Patch the get_qdrant_connection function
        self.qdrant_patcher = patch('src.validation.search.get_qdrant_connection')
        self.mock_get_qdrant_conn = self.qdrant_patcher.start()
        self.mock_get_qdrant_conn.return_value = self.mock_qdrant_conn

        # Patch the get_cohere_client function
        self.cohere_patcher = patch('src.validation.search.get_cohere_client')
        self.mock_get_cohere_client = self.cohere_patcher.start()
        self.mock_cohere_client = MagicMock()
        self.mock_cohere_client.generate_embedding.return_value = [0.1, 0.2, 0.3]
        self.mock_get_cohere_client.return_value = self.mock_cohere_client

        # Create search instance
        self.search = SimilaritySearch()

    def tearDown(self):
        """Clean up after each test method."""
        self.qdrant_patcher.stop()
        self.cohere_patcher.stop()

    def test_similarity_search_initialization(self):
        """Test SimilaritySearch initialization."""
        self.assertIsNotNone(self.search)
        self.assertEqual(self.search.collection_name, "test_collection")
        self.assertEqual(self.search.client, self.mock_client)

    def test_search_by_text_success(self):
        """Test successful similarity search by text."""
        # Mock the search results
        mock_result = MagicMock()
        mock_result.id = "test-id"
        mock_result.score = 0.85
        mock_result.payload = {
            "content": "Test content",
            "source_url": "https://example.com",
            "page_title": "Test Page",
            "chunk_order": 0
        }
        self.mock_client.search.return_value = [mock_result]

        # Perform search
        results = self.search.search_by_text("test query", limit=5)

        # Verify the search was called correctly
        self.mock_client.search.assert_called_once()
        args, kwargs = self.mock_client.search.call_args
        self.assertEqual(kwargs["collection_name"], "test_collection")
        self.assertEqual(kwargs["limit"], 5)

        # Verify the result was converted correctly
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].content, "Test content")
        self.assertEqual(results[0].source_url, "https://example.com")
        self.assertEqual(results[0].similarity_score, 0.85)

    def test_search_by_text_with_filters(self):
        """Test similarity search with metadata filters."""
        # Mock the search results
        mock_result = MagicMock()
        mock_result.id = "test-id"
        mock_result.score = 0.9
        mock_result.payload = {
            "content": "Filtered content",
            "source_url": "https://filtered.com",
            "page_title": "Filtered Page",
            "chunk_order": 0
        }
        self.mock_client.search.return_value = [mock_result]

        # Perform search with filters
        filters = {"source_url": "https://filtered.com"}
        results = self.search.search_by_text("test query", limit=3, metadata_filters=filters)

        # Verify the search was called with filters
        self.mock_client.search.assert_called_once()
        args, kwargs = self.mock_client.search.call_args
        self.assertIsNotNone(kwargs.get("query_filter"))

        # Verify the result
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].content, "Filtered content")

    def test_search_by_embedding_success(self):
        """Test successful similarity search by embedding."""
        # Mock the search results
        mock_result = MagicMock()
        mock_result.id = "embedded-result-id"
        mock_result.score = 0.75
        mock_result.payload = {
            "content": "Embedded content",
            "source_url": "https://embedded.com",
            "page_title": "Embedded Page",
            "chunk_order": 1
        }
        self.mock_client.search.return_value = [mock_result]

        # Perform search by embedding
        query_embedding = [0.1, 0.2, 0.3]
        results = self.search.search_by_embedding(query_embedding, limit=2)

        # Verify the search was called correctly
        self.mock_client.search.assert_called_once()
        args, kwargs = self.mock_client.search.call_args
        self.assertEqual(kwargs["collection_name"], "test_collection")
        self.assertEqual(kwargs["limit"], 2)

        # Verify the result was converted correctly
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].content, "Embedded content")
        self.assertEqual(results[0].similarity_score, 0.75)

    def test_prepare_metadata_filters(self):
        """Test preparing metadata filters."""
        # Test with string filter
        filters = {"source_url": "https://example.com"}
        result = self.search._prepare_metadata_filters(filters)
        self.assertIsNotNone(result)
        self.assertIn("must", result)

        # Test with no filters
        result = self.search._prepare_metadata_filters(None)
        self.assertIsNone(result)

        # Test with empty filters
        result = self.search._prepare_metadata_filters({})
        self.assertIsNone(result)

    def test_convert_to_retrieved_chunk(self):
        """Test converting search result to RetrievedChunk."""
        # Create a mock search result
        mock_result = MagicMock()
        mock_result.id = "chunk-id"
        mock_result.score = 0.8
        mock_result.payload = {
            "content": "Test content for conversion",
            "source_url": "https://convert.com",
            "page_title": "Convert Page",
            "chunk_order": 2
        }

        # Convert to RetrievedChunk
        chunk = self.search._convert_to_retrieved_chunk(mock_result, 1)

        # Verify the conversion
        self.assertIsInstance(chunk, RetrievedChunk)
        self.assertEqual(chunk.id, "chunk-id")
        self.assertEqual(chunk.content, "Test content for conversion")
        self.assertEqual(chunk.source_url, "https://convert.com")
        self.assertEqual(chunk.similarity_score, 0.8)
        self.assertEqual(chunk.retrieval_rank, 1)

    def test_validate_search_functionality_success(self):
        """Test search functionality validation."""
        # Mock a successful search result
        mock_result = MagicMock()
        mock_result.id = "validation-id"
        mock_result.score = 0.9
        mock_result.payload = {
            "content": "Validation content",
            "source_url": "https://validation.com",
            "page_title": "Validation Page",
            "chunk_order": 0
        }

        # Patch the search method to return a result
        with patch.object(self.search, 'search_by_text') as mock_search:
            mock_search.return_value = [RetrievedChunk(content="Validation content", source_url="https://validation.com", page_title="Validation Page")]

            # Validate search functionality
            result = self.search.validate_search_functionality()

            # Verify the result
            self.assertTrue(result)

    def test_validate_search_functionality_failure(self):
        """Test search functionality validation when no results returned."""
        # Patch the search method to return no results
        with patch.object(self.search, 'search_by_text') as mock_search:
            mock_search.return_value = []

            # Validate search functionality
            result = self.search.validate_search_functionality()

            # Verify the result
            self.assertFalse(result)


if __name__ == '__main__':
    unittest.main()