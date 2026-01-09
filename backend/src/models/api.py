from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime


class APIRequest(BaseModel):
    """Model representing structured request containing user query and metadata for the FastAPI endpoint."""
    query: str
    user_context: Optional[Dict[str, Any]] = {}
    retrieval_params: Optional[Dict[str, Any]] = {
        "top_k": 5,
        "threshold": 0.7
    }
    grounding_required: bool = True


class APIResponse(BaseModel):
    """Model representing structured response containing the agent's answer, sources, and additional metadata."""
    success: bool
    answer: str
    sources: List[Dict[str, Any]]
    timestamp: str
    query_id: str
    error: Optional[str] = None