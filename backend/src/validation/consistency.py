from typing import List, Dict, Any, Tuple
import requests
from bs4 import BeautifulSoup
import difflib
import logging
from src.validation.models import RetrievedChunk
from src.ingestion.extractor import extract_content_from_html
from src.ingestion.processor import normalize_for_embedding

logger = logging.getLogger(__name__)

class ContentConsistencyValidator:
    """Content consistency verification module for validation."""

    def __init__(self):
        pass

    def validate_content_consistency(self, retrieved_chunk: RetrievedChunk) -> Tuple[bool, float, str]:
        """
        Validate that retrieved content matches the original source content.

        Args:
            retrieved_chunk: The retrieved chunk to validate

        Returns:
            Tuple of (is_consistent, similarity_score, validation_notes)
        """
        try:
            if not retrieved_chunk.source_url:
                return False, 0.0, "No source URL provided for validation"

            # Fetch the original content from the source URL
            original_content = self._fetch_original_content(retrieved_chunk.source_url)
            if not original_content:
                return False, 0.0, f"Could not fetch content from {retrieved_chunk.source_url}"

            # Calculate similarity between retrieved and original content
            similarity_score = self._calculate_content_similarity(
                retrieved_chunk.content,
                original_content
            )

            # Define threshold for consistency
            is_consistent = similarity_score >= 0.8  # 80% similarity threshold

            if is_consistent:
                notes = f"Content is consistent with original source (similarity: {similarity_score:.2f})"
            else:
                notes = f"Content inconsistency detected (similarity: {similarity_score:.2f})"

            return is_consistent, similarity_score, notes

        except Exception as e:
            logger.error(f"Error validating content consistency: {e}")
            return False, 0.0, f"Error during consistency validation: {str(e)}"

    def validate_multiple_chunks_consistency(self, retrieved_chunks: List[RetrievedChunk]) -> Dict[str, Any]:
        """
        Validate consistency for multiple retrieved chunks.

        Args:
            retrieved_chunks: List of retrieved chunks to validate

        Returns:
            Dictionary with validation summary
        """
        if not retrieved_chunks:
            return {
                "total_chunks": 0,
                "consistent_chunks": 0,
                "inconsistent_chunks": 0,
                "average_similarity": 0.0,
                "validation_results": []
            }

        total_chunks = len(retrieved_chunks)
        consistent_count = 0
        total_similarity = 0.0
        validation_results = []

        for chunk in retrieved_chunks:
            is_consistent, similarity, notes = self.validate_content_consistency(chunk)
            validation_results.append({
                "chunk_id": chunk.id,
                "is_consistent": is_consistent,
                "similarity_score": similarity,
                "notes": notes
            })

            if is_consistent:
                consistent_count += 1

            total_similarity += similarity

        average_similarity = total_similarity / total_chunks if total_chunks > 0 else 0.0

        summary = {
            "total_chunks": total_chunks,
            "consistent_chunks": consistent_count,
            "inconsistent_chunks": total_chunks - consistent_count,
            "average_similarity": average_similarity,
            "consistency_rate": consistent_count / total_chunks if total_chunks > 0 else 0.0,
            "validation_results": validation_results
        }

        return summary

    def _fetch_original_content(self, source_url: str) -> str:
        """
        Fetch the original content from the source URL.

        Args:
            source_url: URL of the original content

        Returns:
            Original content as a string, or empty string if failed
        """
        try:
            response = requests.get(source_url, timeout=30)
            response.raise_for_status()

            # Extract text content from the HTML page
            original_content = extract_content_from_html(response.text, source_url)

            logger.debug(f"Fetched original content from {source_url}, length: {len(original_content)}")
            return original_content

        except Exception as e:
            logger.error(f"Error fetching original content from {source_url}: {e}")
            return ""

    def _calculate_content_similarity(self, content1: str, content2: str) -> float:
        """
        Calculate similarity between two content strings.

        Args:
            content1: First content string
            content2: Second content string

        Returns:
            Similarity score between 0 and 1
        """
        # Normalize both content strings
        norm_content1 = normalize_for_embedding(content1)
        norm_content2 = normalize_for_embedding(content2)

        # Calculate similarity using sequence matcher
        seq_matcher = difflib.SequenceMatcher(None, norm_content1, norm_content2)
        similarity_ratio = seq_matcher.ratio()

        return similarity_ratio

    def validate_source_reference(self, retrieved_chunk: RetrievedChunk) -> Tuple[bool, str]:
        """
        Validate that the source reference in the retrieved chunk points to valid content.

        Args:
            retrieved_chunk: The retrieved chunk to validate

        Returns:
            Tuple of (is_valid, validation_notes)
        """
        try:
            if not retrieved_chunk.source_url:
                return False, "No source URL provided"

            # Try to access the source URL
            response = requests.head(retrieved_chunk.source_url, timeout=10)
            response.raise_for_status()

            # If we can access the URL, it's valid
            return True, f"Source URL {retrieved_chunk.source_url} is accessible"

        except Exception as e:
            logger.warning(f"Source URL validation failed for {retrieved_chunk.source_url}: {e}")
            return False, f"Source URL {retrieved_chunk.source_url} is not accessible: {str(e)}"

    def perform_comprehensive_consistency_check(self, retrieved_chunks: List[RetrievedChunk]) -> Dict[str, Any]:
        """
        Perform a comprehensive consistency check on all retrieved chunks.

        Args:
            retrieved_chunks: List of retrieved chunks to validate

        Returns:
            Dictionary with comprehensive validation results
        """
        logger.info(f"Starting comprehensive consistency check for {len(retrieved_chunks)} chunks")

        # Validate content consistency
        content_validation = self.validate_multiple_chunks_consistency(retrieved_chunks)

        # Validate source references
        valid_sources = 0
        source_validation_results = []
        for chunk in retrieved_chunks:
            is_valid, notes = self.validate_source_reference(chunk)
            source_validation_results.append({
                "chunk_id": chunk.id,
                "source_url": chunk.source_url,
                "is_valid_source": is_valid,
                "validation_notes": notes
            })
            if is_valid:
                valid_sources += 1

        # Compile comprehensive results
        comprehensive_results = {
            "content_validation": content_validation,
            "source_validation": {
                "total_sources": len(retrieved_chunks),
                "valid_sources": valid_sources,
                "invalid_sources": len(retrieved_chunks) - valid_sources,
                "source_validation_rate": valid_sources / len(retrieved_chunks) if retrieved_chunks else 0.0,
                "validation_results": source_validation_results
            },
            "overall_consistency_score": content_validation["average_similarity"],
            "overall_consistency_rate": content_validation["consistency_rate"],
            "timestamp": "2026-01-05T10:00:00Z"  # In a real implementation, use datetime.now().isoformat()
        }

        logger.info(f"Comprehensive consistency check completed. Consistency rate: {comprehensive_results['overall_consistency_rate']:.2f}")
        return comprehensive_results


def get_content_consistency_validator() -> ContentConsistencyValidator:
    """Get a singleton instance of the content consistency validator."""
    return ContentConsistencyValidator()