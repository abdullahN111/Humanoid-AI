from typing import List, Dict, Any, Tuple
import logging
from openai import OpenAI
from src.config.settings import settings
from src.models.agent import RetrievedContent, AgentResponse

logger = logging.getLogger(__name__)


class ResponseValidationService:
    """Service for validating agent responses for grounding and accuracy."""

    def __init__(self):
        """Initialize the validation service."""
        # Initialize OpenAI client for semantic similarity checks
        if settings.GEMINI_API_KEY and settings.GEMINI_BASE_URL:
            self.client = OpenAI(
                api_key=settings.GEMINI_API_KEY,
                base_url=settings.GEMINI_BASE_URL
            )
            self.model = "embedding-001"  # Gemini embedding model
        else:
            self.client = OpenAI(api_key=settings.OPENAI_API_KEY)
            self.model = "text-embedding-3-small"  # OpenAI embedding model

    def validate_content_grounding(self, response: str, sources: List[RetrievedContent]) -> Tuple[bool, float, str]:
        """
        Validate that the response is grounded in the provided sources.

        Args:
            response: The generated response to validate
            sources: List of sources used to generate the response

        Returns:
            Tuple of (is_grounded, confidence_score, validation_notes)
        """
        if not sources:
            return False, 0.0, "No sources provided for grounding validation"

        try:
            # Extract content from sources
            source_contents = [source.content for source in sources]
            source_texts = " ".join(source_contents)

            # Check for semantic similarity between response and source content
            response_lower = response.lower()
            source_lower = source_texts.lower()

            # Count how many words from the response appear in the sources
            response_words = set(response_lower.split())
            source_words = set(source_lower.split())

            if not response_words:
                return True, 1.0, "Empty response is considered valid"

            # Calculate overlap ratio
            common_words = response_words.intersection(source_words)
            overlap_ratio = len(common_words) / len(response_words)

            # Also perform a basic semantic check by looking for key phrases
            response_sentences = response.split('.')
            source_text = " ".join([source.content for source in sources])

            # Count how many sentences from the response appear in or are similar to the sources
            matching_sentences = 0
            for sentence in response_sentences:
                sentence = sentence.strip().lower()
                if sentence and len(sentence) > 10:  # Only check meaningful sentences
                    if sentence in source_text.lower():
                        matching_sentences += 1
                    else:
                        # Check for partial matches
                        words = sentence.split()
                        if len(words) > 0:
                            # Check if at least some words from the sentence appear in the source
                            sentence_words = set(words)
                            source_sentence_words = set(source_text.lower().split())
                            sentence_overlap = len(sentence_words.intersection(source_sentence_words))
                            if sentence_overlap / len(sentence_words) > 0.3:  # 30% overlap
                                matching_sentences += 1

            sentence_ratio = matching_sentences / len([s for s in response_sentences if len(s.strip()) > 10])

            # Combine metrics for overall grounding score
            grounding_score = (overlap_ratio * 0.4 + sentence_ratio * 0.6)
            is_grounded = grounding_score > 0.3  # Threshold for grounding

            notes = f"Grounding score: {grounding_score:.2f}, Word overlap: {overlap_ratio:.2f}, Sentence match: {sentence_ratio:.2f}"

            return is_grounded, grounding_score, notes

        except Exception as e:
            logger.error(f"Error validating content grounding: {str(e)}")
            return False, 0.0, f"Error during grounding validation: {str(e)}"

    def validate_citations(self, response: str, sources: List[RetrievedContent]) -> Tuple[bool, str]:
        """
        Validate that the response includes proper citations to the sources.

        Args:
            response: The generated response to validate
            sources: List of sources used to generate the response

        Returns:
            Tuple of (has_citations, validation_notes)
        """
        try:
            # Check if response mentions source information
            has_source_indicators = False
            response_lower = response.lower()

            # Look for citation indicators
            citation_indicators = [
                "according to", "source:", "cited:", "reference:", "as stated",
                "as mentioned", "from the", "the document", "the source"
            ]

            for indicator in citation_indicators:
                if indicator in response_lower:
                    has_source_indicators = True
                    break

            # Check if source URLs or titles are mentioned
            for source in sources:
                if source.source_url and source.source_url in response:
                    has_source_indicators = True
                    break
                if source.page_title and source.page_title.lower() in response_lower:
                    has_source_indicators = True
                    break

            notes = f"Citations validated: {has_source_indicators}"
            return has_source_indicators, notes

        except Exception as e:
            logger.error(f"Error validating citations: {str(e)}")
            return False, f"Error during citation validation: {str(e)}"

    def validate_response_accuracy(self, response: str, sources: List[RetrievedContent]) -> Tuple[float, str]:
        """
        Validate the accuracy of the response against the provided sources.

        Args:
            response: The generated response to validate
            sources: List of sources used to generate the response

        Returns:
            Tuple of (accuracy_score, validation_notes)
        """
        try:
            # Calculate accuracy based on consistency with sources
            if not sources:
                return 0.0, "No sources for accuracy validation"

            # For now, we'll use the grounding score as a proxy for accuracy
            # In a more sophisticated implementation, we could use fact-checking APIs
            is_grounded, grounding_score, grounding_notes = self.validate_content_grounding(response, sources)

            notes = f"Accuracy based on grounding: {grounding_notes}"
            return grounding_score, notes

        except Exception as e:
            logger.error(f"Error validating response accuracy: {str(e)}")
            return 0.0, f"Error during accuracy validation: {str(e)}"

    def perform_comprehensive_validation(self, agent_response: AgentResponse) -> Dict[str, Any]:
        """
        Perform comprehensive validation of an agent response.

        Args:
            agent_response: The agent response to validate

        Returns:
            Dictionary with comprehensive validation results
        """
        try:
            # Validate grounding
            is_grounded, grounding_score, grounding_notes = self.validate_content_grounding(
                agent_response.answer,
                agent_response.sources
            )

            # Validate citations
            has_citations, citation_notes = self.validate_citations(
                agent_response.answer,
                agent_response.sources
            )

            # Validate accuracy
            accuracy_score, accuracy_notes = self.validate_response_accuracy(
                agent_response.answer,
                agent_response.sources
            )

            # Overall validation result
            overall_valid = is_grounded and (has_citations or len(agent_response.sources) > 0)

            validation_result = {
                "overall_valid": overall_valid,
                "grounding": {
                    "is_grounded": is_grounded,
                    "score": grounding_score,
                    "notes": grounding_notes
                },
                "citations": {
                    "has_citations": has_citations,
                    "notes": citation_notes
                },
                "accuracy": {
                    "score": accuracy_score,
                    "notes": accuracy_notes
                },
                "validation_timestamp": agent_response.timestamp.isoformat() if hasattr(agent_response, 'timestamp') else None
            }

            return validation_result

        except Exception as e:
            logger.error(f"Error performing comprehensive validation: {str(e)}")
            return {
                "overall_valid": False,
                "error": f"Error during comprehensive validation: {str(e)}"
            }


# Global instance of the validation service
validation_service = ResponseValidationService()