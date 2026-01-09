from typing import Dict, Any, Optional, List
import logging
from datetime import datetime
import uuid

from src.agents.rag_agent import RAGAgent
from src.models.agent import Query, RetrievedContent, AgentResponse
from src.models.api import APIRequest, APIResponse
from src.services.validation import validation_service

logger = logging.getLogger(__name__)


class AgentService:
    """Orchestration service for the RAG agent that handles the full query processing pipeline."""

    def __init__(self):
        """Initialize the agent service with the RAG agent."""
        self.rag_agent = RAGAgent()

    async def process_query(
        self,
        query_text: str,
        user_context: Optional[Dict[str, Any]] = None,
        retrieval_params: Optional[Dict[str, Any]] = None,
        grounding_required: bool = True
    ) -> AgentResponse:
        """
        Process a user query through the full RAG pipeline.

        Args:
            query_text: The user's query text
            user_context: Additional context about the user or session
            retrieval_params: Parameters for the retrieval process
            grounding_required: Whether strict grounding in content is required

        Returns:
            AgentResponse containing the answer and metadata
        """
        if user_context is None:
            user_context = {}
        if retrieval_params is None:
            from src.config.settings import settings
            retrieval_params = {
                "top_k": settings.MAX_RETRIEVED_CHUNKS,
                "threshold": settings.RETRIEVAL_THRESHOLD
            }

        query_id = str(uuid.uuid4())
        timestamp = datetime.utcnow()

        try:
            # Log the incoming query
            logger.info(f"Processing query {query_id} from user context: {user_context}")

            # Process the query through the RAG agent
            agent_response = self.rag_agent.process_query(
                query_text=query_text,
                retrieval_params=retrieval_params
            )

            # Perform comprehensive validation if grounding is required
            if grounding_required:
                validation_result = validation_service.perform_comprehensive_validation(agent_response)

                # Update validation notes with comprehensive validation results
                validation_notes = agent_response.validation_notes or ""
                validation_notes += f" | Comprehensive validation: Overall valid: {validation_result.get('overall_valid', False)}"

                # Add specific validation details
                grounding_result = validation_result.get('grounding', {})
                if 'score' in grounding_result:
                    validation_notes += f", Grounding score: {grounding_result['score']:.2f}"

                citation_result = validation_result.get('citations', {})
                if 'has_citations' in citation_result:
                    validation_notes += f", Has citations: {citation_result['has_citations']}"

                accuracy_result = validation_result.get('accuracy', {})
                if 'score' in accuracy_result:
                    validation_notes += f", Accuracy score: {accuracy_result['score']:.2f}"

                agent_response.validation_notes = validation_notes

                # If grounding is required and validation fails, adjust the response
                if not validation_result.get('overall_valid', True):
                    logger.warning(f"Query {query_id} failed comprehensive validation")
                    # For now, we'll still return the response but with validation warnings
                    # In a production system, you might want to handle this differently

            # Update the grounding requirement in validation notes
            if not grounding_required:
                # If grounding isn't required, we'll still provide it but note that
                agent_response.validation_notes = f"{agent_response.validation_notes} | Grounding requirement: not strictly enforced"

            logger.info(f"Successfully processed query {query_id}")
            return agent_response

        except Exception as e:
            logger.error(f"Error processing query '{query_text}': {str(e)}")

            # Create an error response
            error_response = AgentResponse(
                id=query_id,
                query_id=query_id,
                answer="Sorry, I encountered an error while processing your query. Please try again.",
                sources=[],
                confidence_score=0.0,
                timestamp=timestamp,
                validation_notes=f"Error processing query: {str(e)}"
            )

            return error_response

    def validate_query(self, query_text: str) -> Optional[str]:
        """
        Validate a query before processing.

        Args:
            query_text: The query text to validate

        Returns:
            Error message if validation fails, None if valid
        """
        if not query_text or not query_text.strip():
            return "Query text cannot be empty"

        if len(query_text.strip()) < 1:
            return "Query text must be at least 1 character long"

        if len(query_text.strip()) > 1000:
            return "Query text must be less than 1000 characters"

        return None  # Valid query

    async def health_check(self) -> Dict[str, Any]:
        """
        Perform a health check on the agent service.

        Returns:
            Health status information
        """
        try:
            # Check if the RAG agent is initialized
            if not self.rag_agent:
                return {
                    "status": "unavailable",
                    "details": {"agent": "not initialized"},
                    "timestamp": datetime.utcnow().isoformat()
                }

            # Test a simple operation
            test_response = self.rag_agent.process_query("Hello", retrieval_params={"top_k": 1, "threshold": 0.1})

            return {
                "status": "healthy",
                "details": {
                    "agent": "initialized",
                    "connection": "ok",
                    "test_query": "successful"
                },
                "timestamp": datetime.utcnow().isoformat()
            }

        except Exception as e:
            logger.error(f"Agent service health check failed: {str(e)}")
            return {
                "status": "degraded",
                "details": {"error": str(e)},
                "timestamp": datetime.utcnow().isoformat()
            }

    def get_agent_capabilities(self) -> Dict[str, Any]:
        """
        Get information about the agent's capabilities.

        Returns:
            Dictionary with agent capabilities information
        """
        from src.config.settings import settings

        return {
            "model": getattr(self.rag_agent, 'model', 'unknown'),
            "max_retrieved_chunks": settings.MAX_RETRIEVED_CHUNKS,
            "retrieval_threshold": settings.RETRIEVAL_THRESHOLD,
            "grounding_required": settings.GROUNDING_REQUIRED,
            "supported_features": [
                "semantic_search",
                "content_retrieval",
                "context_aware_generation",
                "source_citation"
            ]
        }