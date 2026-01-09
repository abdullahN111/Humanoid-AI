import logging
import sys
from datetime import datetime
from typing import Dict, Any, Optional
from src.config.settings import settings


def setup_logging():
    """Set up logging configuration for the application."""
    # Create a custom formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Create a handler for stdout
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(formatter)

    # Configure the root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG if settings.DEBUG else logging.INFO)
    root_logger.addHandler(handler)

    # Prevent duplicate logs if logging is configured multiple times
    root_logger.propagate = False

    # Set specific loggers to appropriate levels
    logging.getLogger("uvicorn").setLevel(logging.WARNING)
    logging.getLogger("fastapi").setLevel(logging.WARNING)
    logging.getLogger("qdrant_client").setLevel(logging.WARNING)
    logging.getLogger("httpx").setLevel(logging.WARNING)


def create_error_response(error_msg: str, status_code: int = 500) -> Dict[str, Any]:
    """Create a standardized error response."""
    return {
        "success": False,
        "error": error_msg,
        "status_code": status_code,
        "timestamp": datetime.utcnow().isoformat()
    }


def validate_query_text(query: str) -> Optional[str]:
    """Validate query text and return error message if invalid."""
    if not query or not query.strip():
        return "Query text cannot be empty"

    if len(query.strip()) < 1:
        return "Query text must be at least 1 character long"

    if len(query.strip()) > 1000:
        return "Query text must be less than 1000 characters"

    return None  # Valid query


def format_sources_for_response(sources: list) -> list:
    """Format sources for API response, truncating content if necessary."""
    formatted_sources = []
    for source in sources:
        formatted_source = {
            "id": getattr(source, 'id', ''),
            "content": getattr(source, 'content', '')[:200] + "..." if len(getattr(source, 'content', '')) > 200 else getattr(source, 'content', ''),
            "source_url": getattr(source, 'source_url', ''),
            "page_title": getattr(source, 'page_title', ''),
            "similarity_score": getattr(source, 'similarity_score', 0.0)
        }
        formatted_sources.append(formatted_source)

    return formatted_sources


# Initialize logging when this module is imported
setup_logging()