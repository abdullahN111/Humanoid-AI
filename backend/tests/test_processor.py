"""
Unit tests for the processor module.

These tests verify the functionality of the text processing and chunking components.
"""
import unittest
from src.ingestion.processor import (
    normalize_text,
    advanced_normalize_text,
    normalize_for_embedding,
    chunk_text,
    find_sentence_boundary,
    create_content_chunk,
    process_content,
    validate_content_quality
)


class TestProcessor(unittest.TestCase):
    """Test cases for the processor module."""

    def test_normalize_text_basic(self):
        """Test basic text normalization."""
        raw_text = "   This   has   extra   spaces   "
        normalized = normalize_text(raw_text)
        self.assertEqual(normalized, "This has extra spaces")

    def test_normalize_text_quotes(self):
        """Test quote normalization."""
        raw_text = "This is a ''quoted'' text ``example''"
        normalized = normalize_text(raw_text)
        self.assertIn('"quoted"', normalized)
        self.assertIn('"example"', normalized)

    def test_advanced_normalize_text(self):
        """Test advanced text normalization."""
        raw_text = "This... has   irregular...   punctuation   and   spacing"
        normalized = advanced_normalize_text(raw_text)
        # Should normalize multiple dots to ellipsis and fix spacing
        self.assertIn("...", normalized)

    def test_normalize_for_embedding(self):
        """Test embedding-specific normalization."""
        raw_text = "Text with [special] characters & symbols!"
        normalized = normalize_for_embedding(raw_text)
        # Should preserve meaningful characters while cleaning up
        self.assertIn("characters", normalized)
        self.assertIn("symbols", normalized)

    def test_chunk_text_basic(self):
        """Test basic text chunking."""
        text = "This is a sample text for chunking. " * 10  # Create longer text
        chunks = chunk_text(text, chunk_size=50, overlap=10)

        self.assertGreater(len(chunks), 0)
        # Each chunk should be no larger than chunk_size
        for chunk in chunks:
            self.assertLessEqual(len(chunk), 50)

    def test_chunk_text_with_overlap(self):
        """Test text chunking with overlap."""
        text = "A B C D E F G H I J K L M N O P Q R S T U V W X Y Z"
        chunks = chunk_text(text, chunk_size=20, overlap=5)

        self.assertGreater(len(chunks), 1)
        # Verify overlap exists between chunks
        if len(chunks) > 1:
            # The end of first chunk should overlap with beginning of second
            first_chunk_end = chunks[0][-5:]  # Last 5 chars of first chunk
            second_chunk_start = chunks[1][:5]  # First 5 chars of second chunk
            # They should have some overlap
            self.assertTrue(any(c in second_chunk_start for c in first_chunk_end))

    def test_find_sentence_boundary(self):
        """Test finding sentence boundaries."""
        text = "This is a sentence. This is another sentence! Is this a question?"
        boundary_pos = find_sentence_boundary(text)

        # Should find a boundary position
        self.assertGreaterEqual(boundary_pos, 0)

    def test_validate_content_quality_good_content(self):
        """Test content quality validation with good content."""
        good_content = "This is meaningful content with actual information."
        is_quality = validate_content_quality(good_content)
        self.assertTrue(is_quality)

    def test_validate_content_quality_short_content(self):
        """Test content quality validation with short content."""
        short_content = "Hi"
        is_quality = validate_content_quality(short_content, min_length=10)
        self.assertFalse(is_quality)

    def test_validate_content_quality_repetitive_content(self):
        """Test content quality validation with repetitive content."""
        repetitive_content = "test test test test test test test test test test"
        is_quality = validate_content_quality(repetitive_content)
        # Should return False due to excessive repetition
        self.assertFalse(is_quality)

    def test_create_content_chunk(self):
        """Test creating a content chunk."""
        content = "Sample content for testing"
        chunk = create_content_chunk(content, "https://example.com", "Test Page")

        self.assertEqual(chunk.content, content)
        self.assertEqual(chunk.source_url, "https://example.com")
        self.assertEqual(chunk.page_title, "Test Page")
        self.assertIsNotNone(chunk.content_hash)
        self.assertEqual(chunk.chunk_order, 0)

    def test_process_content(self):
        """Test processing content into chunks."""
        text = "This is a longer text that will be processed into chunks. " * 5
        chunks = process_content(text, "https://example.com", "Test Page")

        # Should create at least one chunk
        self.assertGreater(len(chunks), 0)

        # Each chunk should have proper attributes
        for chunk in chunks:
            self.assertIsNotNone(chunk.id)
            self.assertEqual(chunk.source_url, "https://example.com")
            self.assertEqual(chunk.page_title, "Test Page")


if __name__ == '__main__':
    unittest.main()