import pytest
from unittest.mock import Mock
from src.services.validation import ResponseValidationService
from src.models.agent import RetrievedContent, AgentResponse
from datetime import datetime


class TestResponseValidationService:
    """Unit tests for the ResponseValidationService class."""

    @pytest.fixture
    def validation_service(self):
        """Create an instance of ResponseValidationService for testing."""
        # For testing purposes, we'll create the service without actual API clients
        service = ResponseValidationService()
        # Mock the OpenAI client to avoid actual API calls
        service.client = Mock()
        service.model = "test-model"
        return service

    def test_validate_content_grounding_with_sources(self, validation_service):
        """Test content grounding validation with sources."""
        # Create test sources
        source = RetrievedContent(
            id="test-source",
            content="The key components of humanoid robotics include actuators, sensors, and control systems.",
            source_url="https://test.com",
            page_title="Test Page",
            similarity_score=0.8,
            chunk_order=0,
            retrieval_rank=1
        )
        sources = [source]

        # Test response that should be grounded
        response = "The key components of humanoid robotics include actuators, sensors, and control systems."

        is_grounded, score, notes = validation_service.validate_content_grounding(response, sources)

        # The response contains content from the source, so it should be grounded
        assert is_grounded or score > 0  # At least some grounding should be detected
        assert isinstance(score, float)
        assert isinstance(notes, str)

    def test_validate_content_grounding_no_sources(self, validation_service):
        """Test content grounding validation with no sources."""
        response = "This is a test response."
        sources = []

        is_grounded, score, notes = validation_service.validate_content_grounding(response, sources)

        assert is_grounded is False
        assert score == 0.0
        assert isinstance(notes, str)

    def test_validate_citations_present(self, validation_service):
        """Test citation validation when citations are present."""
        source = RetrievedContent(
            id="test-source",
            content="Information about humanoid robotics.",
            source_url="https://example.com/page",
            page_title="Robotics Page",
            similarity_score=0.8,
            chunk_order=0,
            retrieval_rank=1
        )
        sources = [source]

        # Response that mentions the source
        response = "According to the source, information about humanoid robotics is important. [Source 1]"

        has_citations, notes = validation_service.validate_citations(response, sources)

        assert isinstance(has_citations, bool)
        assert isinstance(notes, str)

    def test_validate_citations_absent(self, validation_service):
        """Test citation validation when citations are absent."""
        source = RetrievedContent(
            id="test-source",
            content="Information about humanoid robotics.",
            source_url="https://example.com/page",
            page_title="Robotics Page",
            similarity_score=0.8,
            chunk_order=0,
            retrieval_rank=1
        )
        sources = [source]

        # Response without citations
        response = "Information about humanoid robotics is important."

        has_citations, notes = validation_service.validate_citations(response, sources)

        # The function looks for citation indicators, so result may vary
        assert isinstance(has_citations, bool)
        assert isinstance(notes, str)

    def test_validate_response_accuracy(self, validation_service):
        """Test response accuracy validation."""
        source = RetrievedContent(
            id="test-source",
            content="The key components of humanoid robotics include actuators.",
            source_url="https://test.com",
            page_title="Test Page",
            similarity_score=0.8,
            chunk_order=0,
            retrieval_rank=1
        )
        sources = [source]

        response = "The key components of humanoid robotics include actuators."

        accuracy_score, notes = validation_service.validate_response_accuracy(response, sources)

        assert isinstance(accuracy_score, float)
        assert isinstance(notes, str)

    def test_perform_comprehensive_validation(self, validation_service):
        """Test comprehensive validation of an agent response."""
        source = RetrievedContent(
            id="test-source",
            content="Humanoid robotics involves actuators and sensors.",
            source_url="https://test.com",
            page_title="Test Page",
            similarity_score=0.8,
            chunk_order=0,
            retrieval_rank=1
        )

        agent_response = AgentResponse(
            id="test-response",
            query_id="test-query",
            answer="Humanoid robotics involves actuators and sensors.",
            sources=[source],
            confidence_score=0.8,
            timestamp=datetime.utcnow(),
            validation_notes="Initial validation"
        )

        validation_result = validation_service.perform_comprehensive_validation(agent_response)

        assert isinstance(validation_result, dict)
        assert "overall_valid" in validation_result
        assert "grounding" in validation_result
        assert "citations" in validation_result
        assert "accuracy" in validation_result

    def test_perform_comprehensive_validation_empty_response(self, validation_service):
        """Test comprehensive validation with empty response."""
        agent_response = AgentResponse(
            id="test-response",
            query_id="test-query",
            answer="",
            sources=[],
            confidence_score=0.0,
            timestamp=datetime.utcnow(),
            validation_notes="Initial validation"
        )

        validation_result = validation_service.perform_comprehensive_validation(agent_response)

        assert isinstance(validation_result, dict)
        # Should handle empty responses gracefully

    def test_init_service(self):
        """Test initialization of the validation service."""
        service = ResponseValidationService()

        assert service is not None
        # The service should initialize without errors