from fastapi import APIRouter, HTTPException, Depends
from typing import Dict, Any, List
from pydantic import BaseModel
import uuid
from datetime import datetime

from src.models.agent import Query, RetrievedContent, AgentResponse
from src.models.api import APIRequest, APIResponse
from src.agents.agent_service import AgentService
from src.utils.helpers import format_sources_for_response

router = APIRouter()


class AgentQueryRequest(BaseModel):
    """Request model for agent queries."""
    query: str
    user_context: Dict[str, Any] = {}
    retrieval_params: Dict[str, Any] = {
        "top_k": 5,
        "threshold": 0.7
    }
    grounding_required: bool = True


@router.post("/agent/query", summary="Submit a query to the RAG agent")
async def query_agent(request: AgentQueryRequest) -> APIResponse:
    """
    Submit a query to the RAG agent which retrieves relevant book content and generates a grounded response.

    Args:
        request: Query request containing the user's question and parameters

    Returns:
        Response containing the agent's answer and sources used
    """
    query_id = str(uuid.uuid4())
    timestamp = datetime.utcnow().isoformat()

    try:
        # Initialize the agent service
        agent_service = AgentService()

        # Process the query using the agent
        result = await agent_service.process_query(
            query_text=request.query,
            user_context=request.user_context,
            retrieval_params=request.retrieval_params,
            grounding_required=request.grounding_required
        )

        # Convert result to response format
        sources = []
        if hasattr(result, 'sources') and result.sources:
            for source in result.sources:
                sources.append({
                    "id": source.id,
                    "content": source.content[:200] + "..." if len(source.content) > 200 else source.content,  # Truncate for response
                    "source_url": source.source_url,
                    "page_title": source.page_title,
                    "similarity_score": source.similarity_score,
                    "chunk_order": source.chunk_order,
                    "retrieval_rank": source.retrieval_rank
                })

        return APIResponse(
            success=True,
            answer=result.answer if hasattr(result, 'answer') and result.answer else "No answer generated",
            sources=sources,
            timestamp=timestamp,
            query_id=query_id
        )

    except Exception as e:
        error_msg = f"Error processing query: {str(e)}"
        return APIResponse(
            success=False,
            answer="",
            sources=[],
            timestamp=timestamp,
            query_id=query_id,
            error=error_msg
        )


# New chat endpoint models to match frontend expectations
class QuestionRequest(BaseModel):
    """Request model for chat questions."""
    query: str
    user_context: Dict[str, Any] = {}
    retrieval_params: Dict[str, Any] = {
        "top_k": 5,
        "threshold": 0.7
    }
    grounding_required: bool = True


class AIResponse(BaseModel):
    """Response model for chat responses."""
    success: bool
    answer: str
    sources: List[Dict[str, Any]]
    timestamp: str
    query_id: str
    error: str = None


@router.post("/v1/chat/ask", summary="Chat endpoint for frontend")
async def chat_ask(request: QuestionRequest) -> AIResponse:
    """
    Chat endpoint that matches the frontend's expected API structure.

    Args:
        request: Chat request containing the user's question

    Returns:
        Response containing the agent's answer
    """
    query_id = str(uuid.uuid4())
    timestamp = datetime.utcnow().isoformat()

    try:
        # Initialize the agent service
        agent_service = AgentService()

        # Process the query using the agent
        result = await agent_service.process_query(
            query_text=request.query,
            user_context=request.user_context,
            retrieval_params=request.retrieval_params,
            grounding_required=request.grounding_required
        )

        # Convert result to response format
        sources = []
        if hasattr(result, 'sources') and result.sources:
            for source in result.sources:
                sources.append({
                    "id": source.id,
                    "content": source.content[:200] + "..." if len(source.content) > 200 else source.content,  # Truncate for response
                    "source_url": source.source_url,
                    "page_title": source.page_title,
                    "similarity_score": source.similarity_score,
                    "chunk_order": source.chunk_order,
                    "retrieval_rank": source.retrieval_rank
                })

        return AIResponse(
            success=True,
            answer=result.answer if hasattr(result, 'answer') and result.answer else "No answer generated",
            sources=sources,
            timestamp=timestamp,
            query_id=query_id
        )

    except Exception as e:
        error_msg = f"Error processing query: {str(e)}"
        return AIResponse(
            success=False,
            answer="",
            sources=[],
            timestamp=timestamp,
            query_id=query_id,
            error=error_msg
        )


@router.post("/v1/chat/stream-ask", summary="Streaming chat endpoint for frontend")
async def chat_stream_ask(request: QuestionRequest):
    """
    Streaming chat endpoint that matches the frontend's expected API structure.
    For now, this returns the same result as the regular endpoint, but could be enhanced for streaming later.

    Args:
        request: Chat request containing the user's question

    Returns:
        Response containing the agent's answer
    """
    # For now, reuse the same logic as the regular chat endpoint
    # In the future, this could be enhanced to return streaming responses
    return await chat_ask(request)