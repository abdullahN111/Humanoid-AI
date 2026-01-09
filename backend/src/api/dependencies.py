from typing import Optional
from fastapi import HTTPException, status
from src.config.settings import settings
from src.utils.helpers import validate_query_text
from src.agents.agent_service import AgentService


async def get_agent_service() -> AgentService:
    """
    Dependency to get the agent service instance.

    Returns:
        AgentService: Instance of the agent service
    """
    return AgentService()


def validate_api_key(api_key: Optional[str] = None) -> bool:
    """
    Validate the API key for authentication.

    Args:
        api_key: The API key to validate

    Returns:
        bool: True if the API key is valid, False otherwise
    """
    if not api_key:
        return False

    # In a real implementation, you'd validate against stored keys
    # For now, we'll just check if it matches any of our configured keys
    return api_key in [settings.OPENAI_API_KEY, settings.GEMINI_API_KEY]


def validate_query_input(query: str) -> None:
    """
    Validate the query input before processing.

    Args:
        query: The query string to validate

    Raises:
        HTTPException: If the query is invalid
    """
    error_msg = validate_query_text(query)
    if error_msg:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=error_msg
        )


def get_default_retrieval_params():
    """
    Get default retrieval parameters.

    Returns:
        Dict: Default retrieval parameters
    """
    return {
        "top_k": settings.MAX_RETRIEVED_CHUNKS,
        "threshold": settings.RETRIEVAL_THRESHOLD
    }