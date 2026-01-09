from dataclasses import dataclass
from typing import Optional, Dict, Any
from datetime import datetime
import uuid

@dataclass
class ContentChunk:
    """A segment of extracted text from a book page with associated metadata."""

    id: str = None  # Will be auto-generated if not provided
    content: str = ""
    source_url: str = ""
    page_title: str = ""
    content_hash: str = ""
    ingestion_timestamp: datetime = None
    chunk_order: int = 0
    metadata: Optional[Dict[str, Any]] = None

    def __post_init__(self):
        if self.id is None:
            self.id = str(uuid.uuid4())
        if self.ingestion_timestamp is None:
            self.ingestion_timestamp = datetime.now()
        if self.metadata is None:
            self.metadata = {}

    def validate(self) -> list[str]:
        """Validate the ContentChunk instance."""
        errors = []

        if not self.content.strip():
            errors.append("Content must not be empty")
        if not self.source_url:
            errors.append("Source URL must be provided")
        if self.chunk_order < 0:
            errors.append("Chunk order must be non-negative")

        return errors

@dataclass
class VectorEmbedding:
    """A numerical representation of content chunk text with metadata."""

    id: str = None  # Will be auto-generated if not provided
    chunk_id: str = ""
    vector: list[float] = None
    model_name: str = ""
    model_version: str = ""
    vector_size: int = 0
    created_at: datetime = None

    def __post_init__(self):
        if self.id is None:
            self.id = str(uuid.uuid4())
        if self.vector is None:
            self.vector = []
        if self.created_at is None:
            self.created_at = datetime.now()

    def validate(self) -> list[str]:
        """Validate the VectorEmbedding instance."""
        errors = []

        if not self.chunk_id:
            errors.append("Chunk ID must be provided")
        if not self.vector:
            errors.append("Vector must be provided")
        if not self.model_name:
            errors.append("Model name must be provided")
        if self.vector_size <= 0:
            errors.append("Vector size must be positive")
        if len(self.vector) != self.vector_size:
            errors.append(f"Vector length ({len(self.vector)}) must match vector_size ({self.vector_size})")

        return errors

@dataclass
class SourceReference:
    """Metadata linking embeddings back to the original book page."""

    chunk_id: str = ""
    source_url: str = ""
    page_title: str = ""
    section_title: Optional[str] = None
    content_preview: str = ""
    created_at: datetime = None

    def __post_init__(self):
        if self.created_at is None:
            self.created_at = datetime.now()
        if len(self.content_preview) > 200:
            self.content_preview = self.content_preview[:200]

    def validate(self) -> list[str]:
        """Validate the SourceReference instance."""
        errors = []

        if not self.chunk_id:
            errors.append("Chunk ID must be provided")
        if not self.source_url:
            errors.append("Source URL must be provided")

        return errors

@dataclass
class IngestionRecord:
    """Tracks ingestion runs and their status."""

    id: str = None  # Will be auto-generated if not provided
    start_time: datetime = None
    end_time: Optional[datetime] = None
    status: str = "pending"  # pending, running, completed, failed
    processed_urls: int = 0
    successful_chunks: int = 0
    failed_chunks: int = 0
    error_details: Optional[Dict[str, Any]] = None

    def __post_init__(self):
        if self.id is None:
            self.id = str(uuid.uuid4())
        if self.start_time is None:
            self.start_time = datetime.now()
        if self.error_details is None:
            self.error_details = {}

    def validate(self) -> list[str]:
        """Validate the IngestionRecord instance."""
        errors = []

        valid_statuses = ["pending", "running", "completed", "failed"]
        if self.status not in valid_statuses:
            errors.append(f"Status must be one of {valid_statuses}")
        if self.processed_urls < 0:
            errors.append("Processed URLs must be non-negative")
        if self.successful_chunks < 0:
            errors.append("Successful chunks must be non-negative")
        if self.failed_chunks < 0:
            errors.append("Failed chunks must be non-negative")
        if self.start_time and self.end_time and self.start_time > self.end_time:
            errors.append("Start time must be before end time")

        return errors