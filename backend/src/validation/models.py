from dataclasses import dataclass
from typing import Optional, Dict, Any, List
from datetime import datetime
import uuid

@dataclass
class RetrievalQuery:
    """A semantic search query with optional metadata filters used to test the retrieval system."""

    id: str = None  # Will be auto-generated if not provided
    query_text: str = ""
    expected_results: List[Dict[str, Any]] = None  # List of expected content chunks that should be retrieved
    metadata_filters: Optional[Dict[str, Any]] = None  # Optional filters to apply during retrieval (URL, section, module)
    test_scenario: str = ""  # Description of the test scenario
    created_at: datetime = None

    def __post_init__(self):
        if self.id is None:
            self.id = str(uuid.uuid4())
        if self.created_at is None:
            self.created_at = datetime.now()
        if self.expected_results is None:
            self.expected_results = []
        if self.metadata_filters is None:
            self.metadata_filters = {}

    def validate(self) -> list[str]:
        """Validate the RetrievalQuery instance."""
        errors = []

        if not self.query_text.strip():
            errors.append("Query text must not be empty")
        if self.id and len(self.id) < 10:  # Basic UUID validation
            errors.append("Invalid ID format")

        return errors

@dataclass
class RetrievedChunk:
    """A content chunk returned by the retrieval system with similarity score and source reference."""

    id: str = None  # Will be auto-generated if not provided
    content: str = ""
    source_url: str = ""
    page_title: str = ""
    similarity_score: float = 0.0
    chunk_order: int = 0
    retrieval_rank: int = 0
    metadata: Optional[Dict[str, Any]] = None

    def __post_init__(self):
        if self.id is None:
            self.id = str(uuid.uuid4())
        if self.metadata is None:
            self.metadata = {}

    def validate(self) -> list[str]:
        """Validate the RetrievedChunk instance."""
        errors = []

        if self.similarity_score < 0 or self.similarity_score > 1:
            errors.append("Similarity score must be between 0 and 1")
        if self.retrieval_rank <= 0:
            errors.append("Retrieval rank must be positive")
        if not self.content.strip():
            errors.append("Content must not be empty")
        if self.id and len(self.id) < 10:  # Basic UUID validation
            errors.append("Invalid ID format")

        return errors

@dataclass
class ValidationResult:
    """The outcome of validating a retrieved chunk against the original query and source content."""

    id: str = None  # Will be auto-generated if not provided
    query_id: str = ""
    retrieved_chunks: List[RetrievedChunk] = None
    is_relevant: bool = False
    relevance_score: float = 0.0
    accuracy_metrics: Optional[Dict[str, Any]] = None  # Detailed accuracy metrics (precision, recall, etc.)
    validation_notes: str = ""
    validated_at: datetime = None

    def __post_init__(self):
        if self.id is None:
            self.id = str(uuid.uuid4())
        if self.validated_at is None:
            self.validated_at = datetime.now()
        if self.retrieved_chunks is None:
            self.retrieved_chunks = []
        if self.accuracy_metrics is None:
            self.accuracy_metrics = {}

    def validate(self) -> list[str]:
        """Validate the ValidationResult instance."""
        errors = []

        if self.relevance_score < 0 or self.relevance_score > 1:
            errors.append("Relevance score must be between 0 and 1")
        if not self.query_id:
            errors.append("Query ID must be provided")
        if self.id and len(self.id) < 10:  # Basic UUID validation
            errors.append("Invalid ID format")

        return errors

@dataclass
class AccuracyMetric:
    """A quantitative measure of how well the retrieval system performs, including precision and recall measures."""

    id: str = None  # Will be auto-generated if not provided
    precision: float = 0.0
    recall: float = 0.0
    f1_score: float = 0.0
    mrr: float = 0.0  # Mean Reciprocal Rank
    hit_rate: float = 0.0  # Proportion of queries that return at least one relevant result
    mean_similarity: float = 0.0  # Average similarity score of retrieved results
    total_queries: int = 0
    total_relevant_retrieved: int = 0
    total_expected_retrieved: int = 0
    calculated_at: datetime = None

    def __post_init__(self):
        if self.id is None:
            self.id = str(uuid.uuid4())
        if self.calculated_at is None:
            self.calculated_at = datetime.now()

    def validate(self) -> list[str]:
        """Validate the AccuracyMetric instance."""
        errors = []

        for field_name in ['precision', 'recall', 'f1_score', 'mrr', 'hit_rate']:
            value = getattr(self, field_name)
            if value < 0 or value > 1:
                errors.append(f"{field_name} must be between 0 and 1")

        if self.total_queries < 0:
            errors.append("Total queries must be non-negative")
        if self.total_relevant_retrieved < 0:
            errors.append("Total relevant retrieved must be non-negative")
        if self.total_expected_retrieved < 0:
            errors.append("Total expected retrieved must be non-negative")
        if self.id and len(self.id) < 10:  # Basic UUID validation
            errors.append("Invalid ID format")

        return errors