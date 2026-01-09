import hashlib
import logging
from typing import List, Tuple
import re
from src.storage.models import ContentChunk

logger = logging.getLogger(__name__)

def normalize_text(text: str) -> str:
    """
    Normalize text by cleaning whitespace and standardizing formatting.

    Args:
        text: Raw text to normalize

    Returns:
        Normalized text
    """
    # Replace multiple whitespace with single space
    text = re.sub(r'\s+', ' ', text)

    # Remove leading/trailing whitespace
    text = text.strip()

    # Standardize quotes and other special characters if needed
    text = text.replace("''", '"').replace("``", '"')

    return text

def advanced_normalize_text(text: str) -> str:
    """
    Advanced text normalization with more sophisticated cleaning.

    Args:
        text: Raw text to normalize

    Returns:
        Advanced normalized text
    """
    # Replace multiple whitespace with single space
    text = re.sub(r'\s+', ' ', text)

    # Normalize different types of quotes
    text = re.sub(r'[`\'\'""]', '"', text)  # Replace various quote types with standard quotes

    # Normalize hyphens and dashes
    text = re.sub(r'[‐‑–—]', '-', text)  # Replace various dash types with standard hyphen

    # Normalize ellipses
    text = re.sub(r'\.{3,}', '...', text)

    # Normalize whitespace around punctuation
    text = re.sub(r'\s*([,.!?;:])\s*', r'\1 ', text)

    # Remove extra spaces after punctuation
    text = re.sub(r'\s+', ' ', text)

    # Remove leading/trailing whitespace
    text = text.strip()

    return text

def normalize_for_embedding(text: str) -> str:
    """
    Normalize text specifically for embedding generation.

    Args:
        text: Raw text to normalize

    Returns:
        Normalized text optimized for embeddings
    """
    # Apply advanced normalization
    text = advanced_normalize_text(text)

    # Remove special characters that might interfere with embeddings
    # Keep letters, numbers, common punctuation, and symbols that carry meaning
    text = re.sub(r'[^\w\s\-\.,!?;:\'"(){}\[\]/\\=&%$#@*+\-=<>|—–…]+', ' ', text)

    # Normalize whitespace again after character removal
    text = re.sub(r'\s+', ' ', text)

    # Remove leading/trailing whitespace
    text = text.strip()

    return text

def chunk_text(content: str, chunk_size: int = 1000, overlap: int = 200) -> List[str]:
    """
    Split text into chunks with specified size and overlap.

    Args:
        content: Text to chunk
        chunk_size: Maximum size of each chunk
        overlap: Overlap between chunks to maintain context

    Returns:
        List of text chunks
    """
    if len(content) <= chunk_size:
        return [content] if content.strip() else []

    chunks = []
    start = 0

    while start < len(content):
        # Track previous start to prevent infinite loops
        previous_start = start

        # Determine the end position
        end = start + chunk_size

        # If we're near the end, take the remaining content
        if end >= len(content):
            chunks.append(content[start:])
            break

        # Try to find a sentence boundary within the last 200 characters
        chunk = content[start:end]
        sentence_end = find_sentence_boundary(chunk)

        if sentence_end > 0 and sentence_end < len(chunk):
            # Found a sentence boundary, use it
            actual_end = start + sentence_end
            chunks.append(content[start:actual_end])
            start = actual_end - overlap if overlap > 0 else actual_end
        else:
            # No good sentence boundary found, just take the chunk
            chunks.append(content[start:end])
            start = end - overlap if overlap > 0 else end

        # Ensure we're making progress to avoid infinite loops
        if start <= previous_start:
            start += 1  # Advance by at least 1 character to avoid infinite loop

    # Filter out empty chunks
    return [chunk for chunk in chunks if chunk.strip()]

def find_sentence_boundary(text: str) -> int:
    """
    Find a good sentence boundary within the text.

    Args:
        text: Text to search for sentence boundary

    Returns:
        Position of the sentence boundary, or -1 if not found
    """
    # Look for sentence-ending punctuation followed by whitespace
    sentence_endings = [r'[.!?]\s+', r'[.!?](?=\n)', r'[.!?](?=\r)', r'\n\s*\n', r'\r\s*\r']

    for pattern in sentence_endings:
        matches = list(re.finditer(pattern, text))
        if matches:
            # Return the position of the last match in the first half of the text
            # to avoid cutting too early
            for match in reversed(matches):
                pos = match.end()
                if pos <= len(text) // 2 + len(text) // 4:  # In the second half or earlier
                    return pos

    return -1

def create_content_chunk(content: str, source_url: str, page_title: str, chunk_order: int = 0) -> ContentChunk:
    """
    Create a ContentChunk object with proper metadata.

    Args:
        content: The content text
        source_url: URL of the source page
        page_title: Title of the source page
        chunk_order: Order of this chunk within the document

    Returns:
        ContentChunk object
    """
    # Create content hash for deduplication
    content_hash = hashlib.sha256(content.encode('utf-8')).hexdigest()

    chunk = ContentChunk(
        content=content,
        source_url=source_url,
        page_title=page_title,
        content_hash=content_hash,
        chunk_order=chunk_order
    )

    # Validate the chunk
    validation_errors = chunk.validate()
    if validation_errors:
        raise ValueError(f"Invalid ContentChunk: {validation_errors}")

    return chunk

def validate_content_quality(text: str, min_length: int = 20) -> bool:
    """
    Validate content quality based on various metrics.

    Args:
        text: Text to validate
        min_length: Minimum length of text to be considered valid

    Returns:
        True if content meets quality standards, False otherwise
    """
    if not text or len(text.strip()) < min_length:
        return False

    # Check if text contains meaningful content (not just special characters)
    # Count meaningful characters (letters and numbers)
    meaningful_chars = sum(1 for c in text if c.isalnum())
    total_chars = len(text.strip())

    if total_chars > 0 and meaningful_chars / total_chars < 0.3:  # Less than 30% meaningful characters
        return False

    # Check for excessive repetition (more than 50% of text is repeated)
    words = text.split()
    if len(words) > 0:
        unique_words = set(words)
        if len(unique_words) / len(words) < 0.2:  # Less than 20% unique words
            return False

    return True

def process_content(text: str, source_url: str, page_title: str) -> List[ContentChunk]:
    """
    Process content by normalizing, chunking, and creating ContentChunk objects.

    Args:
        text: Raw content text
        source_url: URL of the source page
        page_title: Title of the source page

    Returns:
        List of ContentChunk objects
    """
    # Normalize the text
    normalized_text = normalize_text(text)

    # Chunk the text
    text_chunks = chunk_text(normalized_text)

    # Create ContentChunk objects
    content_chunks = []
    for i, chunk_content in enumerate(text_chunks):
        if chunk_content.strip():  # Only create chunks with actual content
            # Validate content quality before creating chunk
            if validate_content_quality(chunk_content):
                chunk = create_content_chunk(chunk_content, source_url, page_title, i)
                content_chunks.append(chunk)
            else:
                # Log low-quality content for potential review
                logger.warning(f"Skipping low-quality content chunk from {source_url}, length: {len(chunk_content)}")

    return content_chunks