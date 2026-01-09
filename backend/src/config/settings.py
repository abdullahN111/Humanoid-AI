import os
from dotenv import load_dotenv
from typing import Optional

# Load environment variables from .env file
load_dotenv()

class Settings:
    """Configuration settings for the RAG agent service."""

    # Qdrant Configuration
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "book_content")

    # OpenAI Configuration
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "")

    # Cohere Configuration
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")

    # Google Gemini Configuration (for external LLM in OpenAI Agent SDK)
    GEMINI_API_KEY: str = os.getenv("GEMINI_API_KEY", "")
    GEMINI_BASE_URL: str = os.getenv("GEMINI_BASE_URL", "https://generativelanguage.googleapis.com/v1beta/openai/")

    # Book Site Configuration
    BOOK_SITE_URL: str = os.getenv("BOOK_SITE_URL", "https://humanoidai.vercel.app/")
    SITEMAP_URL: str = os.getenv("SITEMAP_URL", "https://humanoidai.vercel.app/sitemap.xml")

    # Ingestion Configuration
    INGESTION_BATCH_SIZE: int = int(os.getenv("INGESTION_BATCH_SIZE", "10"))
    EMBEDDING_MODEL: str = os.getenv("EMBEDDING_MODEL", "embed-english-v3.0")

    # Agent Configuration
    MAX_RETRIEVED_CHUNKS: int = int(os.getenv("MAX_RETRIEVED_CHUNKS", "5"))
    RETRIEVAL_THRESHOLD: float = float(os.getenv("RETRIEVAL_THRESHOLD", "0.0"))  # Lower threshold for embeddings that can have negative similarities
    GROUNDING_REQUIRED: bool = os.getenv("GROUNDING_REQUIRED", "true").lower() == "true"

    # API Configuration
    API_PREFIX: str = os.getenv("API_PREFIX", "/api")
    DEBUG: bool = os.getenv("DEBUG", "false").lower() == "true"

    # Validation
    @classmethod
    def validate(cls) -> list[str]:
        """Validate that required environment variables are set."""
        errors = []

        if not cls.QDRANT_URL:
            errors.append("QDRANT_URL is required")
        if not cls.QDRANT_API_KEY:
            errors.append("QDRANT_API_KEY is required")
        if not cls.COHERE_API_KEY:
            errors.append("COHERE_API_KEY is required")
        # Either OPENAI_API_KEY or GEMINI_API_KEY must be provided for LLM access
        if not cls.OPENAI_API_KEY and not cls.GEMINI_API_KEY:
            errors.append("Either OPENAI_API_KEY or GEMINI_API_KEY is required")

        return errors

# Global settings instance
settings = Settings()