from bs4 import BeautifulSoup
from typing import List
import logging
import re

logger = logging.getLogger(__name__)

def extract_content_from_html(html_content: str, url: str = "") -> str:
    """
    Extract main content from HTML, excluding navigation and UI elements.

    Args:
        html_content: HTML content as a string
        url: URL of the page (for context, optional)

    Returns:
        Extracted text content
    """
    try:
        soup = BeautifulSoup(html_content, 'lxml')

        # Remove navigation and UI elements
        exclude_selectors = [
            'nav',  # Navigation elements
            'header',  # Header elements
            'footer',  # Footer elements
            'aside',  # Sidebar elements
            '.nav',  # Navigation classes
            '.header',  # Header classes
            '.footer',  # Footer classes
            '.sidebar',  # Sidebar classes
            '.toc',  # Table of contents
            '.table-of-contents',  # Table of contents alternative
            '.menu',  # Menu elements
            '.navbar',  # Navigation bar
            '.navigation',  # Navigation classes
            '.breadcrumb',  # Breadcrumb navigation
            '.pagination',  # Pagination elements
            '.ads',  # Advertisements
            '.advertisement',  # Advertisement classes
            '.social',  # Social media elements
            '.share',  # Share buttons
            '.comments',  # Comments section
            '.comment',  # Comment elements
            '.disqus',  # Disqus comments
            'script',  # Script elements
            'style',  # Style elements
            'noscript',  # Noscript elements
        ]

        for selector in exclude_selectors:
            for element in soup.select(selector):
                element.decompose()

        # For Docusaurus sites, target specific content containers
        # Try different selectors in order of preference
        content_selectors = [
            'main div[class*="docItem"]',  # Docusaurus documentation pages
            'article[class*="markdown"]',  # Docusaurus markdown articles
            'div[class*="container"] main',  # Main content in container
            'main',  # Main content area
            'article',  # General article content
            'div[class*="theme-doc"]',  # Docusaurus theme documentation
            'div[class*="doc"]',  # Documentation containers
            'div[class*="content"]',  # Content divs
        ]

        content = ""
        for selector in content_selectors:
            elements = soup.select(selector)
            if elements:
                # Combine text from all matching elements
                content = "\n".join([elem.get_text(separator=' ', strip=True) for elem in elements])
                break

        # If no specific content found, get all text
        if not content.strip():
            content = soup.get_text(separator=' ', strip=True)

        # Clean up the content
        content = clean_extracted_text(content)

        logger.info(f"Extracted {len(content)} characters from {url}")
        return content

    except Exception as e:
        logger.error(f"Error extracting content from {url}: {e}")
        raise

def extract_content_from_docusaurus_page(html_content: str, url: str = "") -> str:
    """
    Extract content specifically from Docusaurus pages using targeted selectors.

    Args:
        html_content: HTML content as a string
        url: URL of the page (for context, optional)

    Returns:
        Extracted text content
    """
    try:
        soup = BeautifulSoup(html_content, 'lxml')

        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()

        # Docusaurus-specific selectors in order of preference
        docusaurus_selectors = [
            'div[class*="docItem"]',  # Main documentation item container
            'article[class*="markdown"]',  # Docusaurus markdown content
            'div[class*="theme-doc-markdown"]',  # Docusaurus theme markdown
            'div[class*="main-wrapper"]',  # Docusaurus main wrapper
            'main[class*="container"]',  # Docusaurus main container
            'div[class*="doc-content"]',  # Documentation content area
            'div[class*="docs-content"]',  # Alternative docs content class
            'div[class*="content"]',  # General content area
        ]

        content = ""
        for selector in docusaurus_selectors:
            elements = soup.select(selector)
            if elements:
                # Combine text from all matching elements
                content = "\n".join([elem.get_text(separator=' ', strip=True) for elem in elements])
                break

        # If no specific content found, use general extraction
        if not content.strip():
            content = extract_content_from_html(html_content, url)

        # Clean up the content
        content = clean_extracted_text(content)

        logger.info(f"Extracted {len(content)} characters from Docusaurus page {url}")
        return content

    except Exception as e:
        logger.error(f"Error extracting content from Docusaurus page {url}: {e}")
        raise

def clean_extracted_text(text: str) -> str:
    """
    Clean up extracted text by removing extra whitespace and normalizing.

    Args:
        text: Raw extracted text

    Returns:
        Cleaned text
    """
    # Remove extra whitespace and normalize
    text = re.sub(r'\s+', ' ', text)
    text = text.strip()

    # Remove special characters that might be problematic
    # Keep letters, numbers, punctuation, and common symbols
    # Updated to support non-English characters (Latin Extended A and B, IPA Extensions)
    text = re.sub(r'[^\w\s\-\.,!?;:\'"(){}\[\]/\\=&%$#@*+\-=<>|\u00C0-\u017F\u0100-\u024F\u1E00-\u1EFF]+', ' ', text)

    return text

def normalize_non_english_content(text: str) -> str:
    """
    Normalize content that may contain non-English characters and special symbols.

    Args:
        text: Text that may contain non-English content

    Returns:
        Normalized text with special handling for non-English content
    """
    # Handle common non-English characters and normalize them
    # This function could be expanded based on specific requirements

    # Normalize accented characters (e.g., café -> cafe)
    import unicodedata
    text = unicodedata.normalize('NFKD', text)

    # Remove diacritical marks while preserving the base character
    text = ''.join(c for c in text if not unicodedata.combining(c))

    # Clean up the text
    text = clean_extracted_text(text)

    return text

def extract_page_title(html_content: str) -> str:
    """
    Extract the title from HTML content.

    Args:
        html_content: HTML content as a string

    Returns:
        Page title or empty string if not found
    """
    try:
        soup = BeautifulSoup(html_content, 'lxml')
        title_tag = soup.find('title')
        if title_tag:
            return title_tag.get_text().strip()

        # Fallback to h1 if title not found
        h1_tag = soup.find('h1')
        if h1_tag:
            return h1_tag.get_text().strip()

        return ""
    except Exception as e:
        logger.error(f"Error extracting title: {e}")
        return ""