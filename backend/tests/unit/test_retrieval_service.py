import pytest
from unittest.mock import Mock, patch
from src.services.retrieval import RetrievalService
from src.models.agent import RetrievedContent


class TestRetrievalService:
    """Unit tests for the RetrievalService class."""

    @pytest.fixture
    def retrieval_service(self):
        """Create an instance of RetrievalService for testing."""
        # Mock the dependencies to avoid actual Qdrant connections
        with patch('src.services.retrieval.QdrantConnection') as mock_qdrant_conn, \
             patch('src.services.retrieval.CohereClient') as mock_cohere_client:

            mock_client = Mock()
            mock_qdrant_conn.return_value.get_client.return_value = mock_client
            mock_cohere_client.return_value = Mock()

            return RetrievalService()

    def test_retrieve_by_text(self, retrieval_service):
        """Test retrieving content by text query."""
        # Mock the Cohere client embed method
        with patch.object(retrieval_service.cohere_client, 'embed') as mock_embed, \
             patch.object(retrieval_service.qdrant_client, 'search') as mock_search:

            # Mock embedding response
            mock_embed_response = Mock()
            mock_embed_response.embeddings = [[0.1, 0.2, 0.3]]
            mock_embed.return_value = mock_embed_response

            # Mock search results
            mock_hit = Mock()
            mock_hit.id = "test-id"
            mock_hit.score = 0.85
            mock_hit.payload = {
                "content": "Test content for humanoid robotics",
                "source_url": "https://test.com",
                "page_title": "Test Page",
                "chunk_order": 0
            }
            mock_search.return_value = [mock_hit]

            # Perform the retrieval
            results = retrieval_service.retrieve_by_text("test query", top_k=1, threshold=0.7)

            # Verify the results
            assert len(results) == 1
            assert isinstance(results[0], RetrievedContent)
            assert results[0].id == "test-id"
            assert results[0].content == "Test content for humanoid robotics"
            assert results[0].source_url == "https://test.com"
            assert results[0].similarity_score == 0.85

    def test_retrieve_by_embedding(self, retrieval_service):
        """Test retrieving content by embedding vector."""
        with patch.object(retrieval_service.qdrant_client, 'search') as mock_search:
            # Mock search results
            mock_hit = Mock()
            mock_hit.id = "test-id-2"
            mock_hit.score = 0.9
            mock_hit.payload = {
                "content": "Test content from embedding",
                "source_url": "https://test2.com",
                "page_title": "Test Page 2",
                "chunk_order": 1
            }
            mock_search.return_value = [mock_hit]

            # Perform the retrieval
            results = retrieval_service.retrieve_by_embedding([0.1, 0.2, 0.3], top_k=1, threshold=0.7)

            # Verify the results
            assert len(results) == 1
            assert isinstance(results[0], RetrievedContent)
            assert results[0].id == "test-id-2"
            assert results[0].content == "Test content from embedding"
            assert results[0].source_url == "https://test2.com"
            assert results[0].similarity_score == 0.9

    def test_validate_connection_success(self, retrieval_service):
        """Test successful connection validation."""
        with patch.object(retrieval_service.qdrant_client, 'get_collections') as mock_get_collections, \
             patch.object(retrieval_service.qdrant_client, 'search') as mock_search:

            # Mock collections response
            mock_collection = Mock()
            mock_collection.name = retrieval_service.collection_name
            mock_get_collections.return_value = Mock(collections=[mock_collection])

            # Mock search response
            mock_search.return_value = []

            # Validate connection
            result = retrieval_service.validate_connection()

            assert result is True

    def test_validate_connection_failure_no_collection(self, retrieval_service):
        """Test connection validation failure when collection doesn't exist."""
        with patch.object(retrieval_service.qdrant_client, 'get_collections') as mock_get_collections:
            # Mock collections response with no matching collection
            mock_collection = Mock()
            mock_collection.name = "other-collection"
            mock_get_collections.return_value = Mock(collections=[mock_collection])

            # Validate connection
            result = retrieval_service.validate_connection()

            assert result is False

    def test_retrieve_by_text_error_handling(self, retrieval_service):
        """Test error handling in retrieve_by_text method."""
        with patch.object(retrieval_service.cohere_client, 'embed') as mock_embed:
            # Mock an exception
            mock_embed.side_effect = Exception("Test error")

            # Perform the retrieval - should return empty list
            results = retrieval_service.retrieve_by_text("test query")

            # Verify the results
            assert len(results) == 0

    def test_retrieve_by_embedding_error_handling(self, retrieval_service):
        """Test error handling in retrieve_by_embedding method."""
        with patch.object(retrieval_service.qdrant_client, 'search') as mock_search:
            # Mock an exception
            mock_search.side_effect = Exception("Test error")

            # Perform the retrieval - should return empty list
            results = retrieval_service.retrieve_by_embedding([0.1, 0.2, 0.3])

            # Verify the results
            assert len(results) == 0