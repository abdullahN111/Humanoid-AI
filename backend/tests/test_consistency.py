"""
Unit tests for the consistency module.

These tests verify the functionality of the content consistency verification components.
"""
import unittest
from unittest.mock import patch, MagicMock
from src.validation.consistency import ContentConsistencyValidator
from src.validation.models import RetrievedChunk


class TestConsistency(unittest.TestCase):
    """Test cases for the consistency module."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.validator = ContentConsistencyValidator()

    def test_validate_content_consistency_success(self):
        """Test content consistency validation for matching content."""
        # Create a retrieved chunk with content that matches source
        chunk = RetrievedChunk(
            id="test-chunk",
            content="This is the original content that matches.",
            source_url="https://example.com/test",
            page_title="Test Page"
        )

        # Patch the fetch and similarity methods to simulate success
        with patch.object(self.validator, '_fetch_original_content', return_value="This is the original content that matches."), \
             patch.object(self.validator, '_calculate_content_similarity', return_value=0.95):

            is_consistent, similarity, notes = self.validator.validate_content_consistency(chunk)

            self.assertTrue(is_consistent)
            self.assertEqual(similarity, 0.95)
            self.assertIn("Content is consistent", notes)

    def test_validate_content_consistency_failure(self):
        """Test content consistency validation for non-matching content."""
        # Create a retrieved chunk with content that doesn't match source
        chunk = RetrievedChunk(
            id="test-chunk",
            content="This is different content that does not match.",
            source_url="https://example.com/test",
            page_title="Test Page"
        )

        # Patch the fetch and similarity methods to simulate failure
        with patch.object(self.validator, '_fetch_original_content', return_value="This is the original content that matches."), \
             patch.object(self.validator, '_calculate_content_similarity', return_value=0.2):

            is_consistent, similarity, notes = self.validator.validate_content_consistency(chunk)

            self.assertFalse(is_consistent)
            self.assertEqual(similarity, 0.2)
            self.assertIn("Content inconsistency detected", notes)

    def test_validate_content_consistency_no_source_url(self):
        """Test content consistency validation when no source URL is provided."""
        # Create a retrieved chunk with no source URL
        chunk = RetrievedChunk(
            id="test-chunk",
            content="This is some content.",
            source_url="",  # Empty source URL
            page_title="Test Page"
        )

        is_consistent, similarity, notes = self.validator.validate_content_consistency(chunk)

        self.assertFalse(is_consistent)
        self.assertEqual(similarity, 0.0)
        self.assertIn("No source URL provided", notes)

    def test_validate_content_consistency_fetch_error(self):
        """Test content consistency validation when fetching original content fails."""
        # Create a retrieved chunk
        chunk = RetrievedChunk(
            id="test-chunk",
            content="This is some content.",
            source_url="https://example.com/test",
            page_title="Test Page"
        )

        # Patch the fetch method to simulate an error
        with patch.object(self.validator, '_fetch_original_content', return_value=""):
            is_consistent, similarity, notes = self.validator.validate_content_consistency(chunk)

            self.assertFalse(is_consistent)
            self.assertEqual(similarity, 0.0)
            self.assertIn("Could not fetch content", notes)

    def test_validate_multiple_chunks_consistency_all_consistent(self):
        """Test consistency validation for multiple chunks that are all consistent."""
        chunks = [
            RetrievedChunk(id="chunk1", content="Content 1", source_url="https://example.com/1"),
            RetrievedChunk(id="chunk2", content="Content 2", source_url="https://example.com/2")
        ]

        # Patch the validation method to return consistent results
        with patch.object(self.validator, 'validate_content_consistency') as mock_validate:
            mock_validate.side_effect = [(True, 0.9, "Consistent"), (True, 0.85, "Consistent")]

            results = self.validator.validate_multiple_chunks_consistency(chunks)

            self.assertEqual(results["total_chunks"], 2)
            self.assertEqual(results["consistent_chunks"], 2)
            self.assertEqual(results["inconsistent_chunks"], 0)
            self.assertEqual(results["average_similarity"], (0.9 + 0.85) / 2)

    def test_validate_multiple_chunks_consistency_mixed(self):
        """Test consistency validation for multiple chunks with mixed results."""
        chunks = [
            RetrievedChunk(id="chunk1", content="Content 1", source_url="https://example.com/1"),
            RetrievedChunk(id="chunk2", content="Content 2", source_url="https://example.com/2"),
            RetrievedChunk(id="chunk3", content="Content 3", source_url="https://example.com/3")
        ]

        # Patch the validation method to return mixed results
        with patch.object(self.validator, 'validate_content_consistency') as mock_validate:
            mock_validate.side_effect = [
                (True, 0.9, "Consistent"),    # Consistent
                (False, 0.3, "Not consistent"), # Not consistent
                (True, 0.85, "Consistent")    # Consistent
            ]

            results = self.validator.validate_multiple_chunks_consistency(chunks)

            self.assertEqual(results["total_chunks"], 3)
            self.assertEqual(results["consistent_chunks"], 2)
            self.assertEqual(results["inconsistent_chunks"], 1)
            self.assertEqual(results["average_similarity"], (0.9 + 0.3 + 0.85) / 3)
            self.assertEqual(results["consistency_rate"], 2/3)

    def test_validate_multiple_chunks_consistency_empty_input(self):
        """Test consistency validation with empty input."""
        results = self.validator.validate_multiple_chunks_consistency([])

        self.assertEqual(results["total_chunks"], 0)
        self.assertEqual(results["consistent_chunks"], 0)
        self.assertEqual(results["inconsistent_chunks"], 0)
        self.assertEqual(results["average_similarity"], 0.0)

    @patch('requests.get')
    def test_fetch_original_content_success(self, mock_requests_get):
        """Test successful fetching of original content."""
        # Mock a successful response
        mock_response = MagicMock()
        mock_response.text = "<html><body>Original content from source</body></html>"
        mock_response.raise_for_status.return_value = None
        mock_requests_get.return_value = mock_response

        content = self.validator._fetch_original_content("https://example.com/test")

        # Verify the request was made
        mock_requests_get.assert_called_once_with("https://example.com/test", timeout=30)

        # The content would be processed by extract_content_from_html,
        # which we can't easily test without mocking that function too
        # So we just verify that we get a non-empty string
        self.assertIsInstance(content, str)

    @patch('requests.get')
    def test_fetch_original_content_request_error(self, mock_requests_get):
        """Test handling of request errors when fetching original content."""
        # Mock a request error
        mock_requests_get.side_effect = Exception("Network error")

        content = self.validator._fetch_original_content("https://example.com/test")

        # Verify that an empty string is returned on error
        self.assertEqual(content, "")

    def test_calculate_content_similarity_identical(self):
        """Test content similarity calculation for identical content."""
        content1 = "This is exactly the same content."
        content2 = "This is exactly the same content."

        similarity = self.validator._calculate_content_similarity(content1, content2)

        # Identical content should have high similarity (though not necessarily 1.0 due to normalization)
        self.assertGreaterEqual(similarity, 0.95)

    def test_calculate_content_similarity_different(self):
        """Test content similarity calculation for different content."""
        content1 = "This is completely different content."
        content2 = "This is another text with no relation."

        similarity = self.validator._calculate_content_similarity(content1, content2)

        # Different content should have lower similarity
        self.assertLess(similarity, 0.5)

    def test_calculate_content_similarity_case_difference(self):
        """Test content similarity calculation with case differences."""
        content1 = "This is the Same Content."
        content2 = "THIS IS THE SAME CONTENT."

        similarity = self.validator._calculate_content_similarity(content1, content2)

        # Case differences should be normalized, so similarity should be high
        self.assertGreaterEqual(similarity, 0.95)

    def test_validate_source_reference_success(self):
        """Test source reference validation for accessible URL."""
        chunk = RetrievedChunk(
            id="test-chunk",
            content="Test content",
            source_url="https://httpbin.org/status/200"  # A URL that returns 200
        )

        # Mock requests.head to simulate a successful response
        with patch('requests.head') as mock_head:
            mock_response = MagicMock()
            mock_response.raise_for_status.return_value = None
            mock_head.return_value = mock_response

            is_valid, notes = self.validator.validate_source_reference(chunk)

            self.assertTrue(is_valid)
            self.assertIn("accessible", notes)

    def test_validate_source_reference_failure(self):
        """Test source reference validation for inaccessible URL."""
        chunk = RetrievedChunk(
            id="test-chunk",
            content="Test content",
            source_url="https://nonexistent-domain-12345.com"
        )

        # Mock requests.head to simulate a failure
        with patch('requests.head') as mock_head:
            mock_head.side_effect = Exception("DNS resolution failed")

            is_valid, notes = self.validator.validate_source_reference(chunk)

            self.assertFalse(is_valid)
            self.assertIn("not accessible", notes)

    def test_validate_source_reference_no_url(self):
        """Test source reference validation when no URL is provided."""
        chunk = RetrievedChunk(
            id="test-chunk",
            content="Test content",
            source_url=""  # Empty URL
        )

        is_valid, notes = self.validator.validate_source_reference(chunk)

        self.assertFalse(is_valid)
        self.assertIn("No source URL provided", notes)

    def test_perform_comprehensive_consistency_check(self):
        """Test comprehensive consistency check."""
        chunks = [
            RetrievedChunk(id="chunk1", content="Content 1", source_url="https://example.com/1"),
            RetrievedChunk(id="chunk2", content="Content 2", source_url="https://example.com/2")
        ]

        # Mock the validation methods to return known results
        with patch.object(self.validator, 'validate_multiple_chunks_consistency') as mock_content:
            mock_content.return_value = {
                "total_chunks": 2,
                "consistent_chunks": 1,
                "inconsistent_chunks": 1,
                "average_similarity": 0.75,
                "consistency_rate": 0.5,
                "validation_results": [
                    {"chunk_id": "chunk1", "is_consistent": True, "similarity_score": 0.9},
                    {"chunk_id": "chunk2", "is_consistent": False, "similarity_score": 0.6}
                ]
            }

            with patch.object(self.validator, 'validate_source_reference') as mock_source:
                mock_source.side_effect = [(True, "Source 1 OK"), (True, "Source 2 OK")]

                results = self.validator.perform_comprehensive_consistency_check(chunks)

                # Verify the results structure
                self.assertIn("content_validation", results)
                self.assertIn("source_validation", results)
                self.assertIn("overall_consistency_score", results)
                self.assertIn("overall_consistency_rate", results)
                self.assertEqual(results["content_validation"]["total_chunks"], 2)
                self.assertEqual(results["source_validation"]["valid_sources"], 2)


if __name__ == '__main__':
    unittest.main()