from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime
from enum import Enum


class Query(BaseModel):
    """Model representing a user's question or request for information from the book content."""
    id: str
    text: str
    timestamp: datetime
    user_id: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = {}


class RetrievedContent(BaseModel):
    """Model representing book sections retrieved from the vector database that are relevant to the query."""
    id: str
    content: str
    source_url: str
    page_title: str
    similarity_score: float
    chunk_order: int
    retrieval_rank: int


class AgentResponse(BaseModel):
    """Model representing the AI-generated answer based on retrieved content, including citations and confidence indicators."""
    id: str
    query_id: str
    answer: str
    sources: List[RetrievedContent]
    confidence_score: Optional[float] = None
    timestamp: datetime
    validation_notes: Optional[str] = None