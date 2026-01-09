import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, '.')

from src.ingestion.pipeline import IngestionPipeline
from src.ingestion.crawler import fetch_page_content_with_retry
from src.ingestion.extractor import extract_content_from_html, extract_page_title
from src.ingestion.processor import process_content

def test_single_url_processing():
    print("Testing single URL processing...")
    try:
        # Test with a single URL
        test_url = "https://humanoidai.vercel.app/blog"
        print(f"Processing URL: {test_url}")

        # Fetch page content
        html_content = fetch_page_content_with_retry(test_url)
        print(f"HTML content length: {len(html_content)}")

        # Extract content and title
        content = extract_content_from_html(html_content, test_url)
        page_title = extract_page_title(html_content)
        print(f"Extracted content length: {len(content)}")
        print(f"Page title: {page_title}")

        # Process content into chunks
        content_chunks = process_content(content, test_url, page_title)
        print(f"Created {len(content_chunks)} content chunks")

        # Print first few chunks for verification
        for i, chunk in enumerate(content_chunks[:2]):
            print(f"Chunk {i}: {len(chunk.content)} chars - {chunk.content[:100]}...")

    except Exception as e:
        print(f"Error processing URL: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_single_url_processing()