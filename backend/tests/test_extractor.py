"""
Unit tests for the extractor module.

These tests verify the functionality of the HTML content extraction components.
"""
import unittest
from src.ingestion.extractor import extract_content_from_html, extract_page_title, clean_extracted_text


class TestExtractor(unittest.TestCase):
    """Test cases for the extractor module."""

    def test_extract_content_from_html_basic(self):
        """Test basic HTML content extraction."""
        html_content = """
        <html>
            <head><title>Test Page</title></head>
            <body>
                <div class="content">
                    <p>This is the main content of the page.</p>
                    <p>It has multiple paragraphs.</p>
                </div>
                <nav>Navigation content should be excluded</nav>
            </body>
        </html>
        """
        content = extract_content_from_html(html_content, "https://example.com/test")

        # Check that main content is extracted
        self.assertIn("main content of the page", content)
        self.assertIn("multiple paragraphs", content)
        # Check that navigation content is excluded
        self.assertNotIn("Navigation content should be excluded", content)

    def test_extract_content_from_html_docusaurus_specific(self):
        """Test Docusaurus-specific content extraction."""
        html_content = """
        <html>
            <body>
                <main>
                    <div class="docItem">
                        <article class="markdown">
                            <h1>Documentation Title</h1>
                            <p>This is Docusaurus documentation content.</p>
                            <p>It should be properly extracted.</p>
                        </article>
                    </div>
                </main>
                <aside>Sidebar content to exclude</aside>
            </body>
        </html>
        """
        content = extract_content_from_html(html_content, "https://example.com/docs")

        # Check that documentation content is extracted
        self.assertIn("Docusaurus documentation content", content)
        self.assertIn("properly extracted", content)
        # Check that sidebar content is excluded
        self.assertNotIn("Sidebar content to exclude", content)

    def test_extract_page_title_from_title_tag(self):
        """Test extracting page title from title tag."""
        html_content = """
        <html>
            <head><title>Page Title</title></head>
            <body><p>Content</p></body>
        </html>
        """
        title = extract_page_title(html_content)
        self.assertEqual(title, "Page Title")

    def test_extract_page_title_from_h1_tag(self):
        """Test extracting page title from h1 tag when title tag is missing."""
        html_content = """
        <html>
            <body>
                <h1>Main Heading</h1>
                <p>Content</p>
            </body>
        </html>
        """
        title = extract_page_title(html_content)
        self.assertEqual(title, "Main Heading")

    def test_extract_page_title_empty_when_no_title_or_h1(self):
        """Test that empty string is returned when no title or h1 is found."""
        html_content = """
        <html>
            <body>
                <p>Just content with no title</p>
            </body>
        </html>
        """
        title = extract_page_title(html_content)
        self.assertEqual(title, "")

    def test_clean_extracted_text(self):
        """Test cleaning extracted text."""
        raw_text = "   This   is   messy   text   with   extra   spaces   "
        cleaned = clean_extracted_text(raw_text)
        # Should have single spaces between words
        self.assertEqual(cleaned, "This is messy text with extra spaces")

    def test_clean_extracted_text_special_chars(self):
        """Test cleaning text with special characters."""
        raw_text = "Text with\ttabs and\nnewlines and  multiple   spaces"
        cleaned = clean_extracted_text(raw_text)
        # Should normalize whitespace
        self.assertIn("Text with tabs and newlines and multiple spaces", cleaned)


if __name__ == '__main__':
    unittest.main()