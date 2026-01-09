import pytest
import asyncio
from unittest.mock import Mock, AsyncMock, patch
from src.agents.agent_service import AgentService
from src.models.agent import RetrievedContent
from src.config.settings import settings


class TestAgentService:
    """Unit tests for the AgentService class."""

    @pytest.fixture
    def agent_service(self):
        """Create an instance of AgentService for testing."""
        return AgentService()

    @pytest.mark.asyncio
    async def test_process_query_success(self, agent_service):
        """Test successful query processing."""
        # Mock the RAG agent's process_query method
        with patch.object(agent_service.rag_agent, 'process_query') as mock_process_query:
            # Create mock response
            mock_response = Mock()
            mock_response.answer = "This is a test answer"
            mock_response.sources = []
            mock_response.confidence_score = 0.9
            mock_response.validation_notes = "Test validation notes"
            mock_process_query.return_value = mock_response

            # Process a test query
            result = await agent_service.process_query(
                query_text="What are the key components of humanoid robotics?",
                user_context={"test": True}
            )

            # Verify the result
            assert result.answer == "This is a test answer"
            assert result.confidence_score == 0.9
            mock_process_query.assert_called_once()

    @pytest.mark.asyncio
    async def test_process_query_with_sources(self, agent_service):
        """Test query processing with retrieved sources."""
        with patch.object(agent_service.rag_agent, 'process_query') as mock_process_query:
            # Create mock response with sources
            mock_source = RetrievedContent(
                id="test-source-1",
                content="Test content for humanoid robotics",
                source_url="https://test.com",
                page_title="Test Page",
                similarity_score=0.8,
                chunk_order=0,
                retrieval_rank=1
            )

            mock_response = Mock()
            mock_response.answer = "Based on the content, the key components are..."
            mock_response.sources = [mock_source]
            mock_response.confidence_score = 0.85
            mock_response.validation_notes = "Test validation notes"
            mock_process_query.return_value = mock_response

            # Process a test query
            result = await agent_service.process_query(
                query_text="What are the key components of humanoid robotics?",
                retrieval_params={"top_k": 3, "threshold": 0.7}
            )

            # Verify the result
            assert result.answer == "Based on the content, the key components are..."
            assert len(result.sources) == 1
            assert result.sources[0].id == "test-source-1"
            assert result.confidence_score == 0.85

    def test_validate_query_empty(self, agent_service):
        """Test query validation with empty query."""
        result = agent_service.validate_query("")
        assert result is not None  # Should return error message
        assert "cannot be empty" in result.lower()

    def test_validate_query_short(self, agent_service):
        """Test query validation with very short query."""
        result = agent_service.validate_query("A")
        assert result is not None  # Should return error message
        assert "at least 1 character" in result.lower()

    def test_validate_query_long(self, agent_service):
        """Test query validation with very long query."""
        long_query = "A" * 1001  # More than 1000 characters
        result = agent_service.validate_query(long_query)
        assert result is not None  # Should return error message
        assert "less than 1000 characters" in result.lower()

    def test_validate_query_valid(self, agent_service):
        """Test query validation with valid query."""
        result = agent_service.validate_query("What are the key components?")
        assert result is None  # Should return None for valid query

    @pytest.mark.asyncio
    async def test_health_check(self, agent_service):
        """Test the health check functionality."""
        # Mock the RAG agent's process_query method for the health check
        with patch.object(agent_service.rag_agent, 'process_query') as mock_process_query:
            mock_response = Mock()
            mock_response.answer = "Hello"
            mock_response.sources = []
            mock_response.confidence_score = 1.0
            mock_response.validation_notes = "Test notes"
            mock_process_query.return_value = mock_response

            health_result = await agent_service.health_check()

            assert "status" in health_result
            assert "details" in health_result
            assert "timestamp" in health_result
            assert health_result["status"] in ["healthy", "degraded"]

    def test_get_agent_capabilities(self, agent_service):
        """Test getting agent capabilities."""
        capabilities = agent_service.get_agent_capabilities()

        assert "model" in capabilities
        assert "max_retrieved_chunks" in capabilities
        assert "retrieval_threshold" in capabilities
        assert "grounding_required" in capabilities
        assert "supported_features" in capabilities
        assert isinstance(capabilities["supported_features"], list)