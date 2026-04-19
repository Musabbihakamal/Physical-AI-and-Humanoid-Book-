"""
Content extractor module for the ingestion pipeline
"""

import re
from typing import Tuple

from bs4 import BeautifulSoup

from backend.rag_bot.config import DEFAULT_CONFIG


def extract_clean_content(html_content: str, url: str = "") -> Tuple[str, str, str]:
    """
    Extract clean text content from HTML, preserving structure.

    Args:
        html_content: Raw HTML content
        url: Source URL for context

    Returns:
        Tuple of (clean_text, page_title, section_title)
    """
    import logging
    logger = logging.getLogger(__name__)

    soup = BeautifulSoup(html_content, 'html.parser')

    # Remove common Docusaurus UI elements that aren't content
    selectors_to_remove = [
        'nav', 'header', 'footer', '.navbar', '.menu', '.sidebar',
        '.theme-edit-this-page', '.theme-last-updated', '.pagination-nav',
        '.toc', '.table-of-contents', '.theme-doc-breadcrumbs'
    ]

    for selector in selectors_to_remove:
        for element in soup.select(selector):
            element.decompose()

    # Extract page title
    title_tag = soup.find('title')
    page_title = title_tag.get_text().strip() if title_tag else ""

    # Look for main heading as section title
    main_heading = soup.find(['h1', 'h2'])
    section_title = main_heading.get_text().strip() if main_heading else page_title

    # Extract main content - look for main content containers
    main_content_selectors = [
        'main', '.main-wrapper', '.theme-doc-markdown',
        '.container.padding-vert--xl', '.doc-content'
    ]

    main_content = None
    for selector in main_content_selectors:
        main_content = soup.select_one(selector)
        if main_content:
            break

    if not main_content:
        # If no specific container found, use body
        main_content = soup.find('body')

    if main_content:
        # Extract text content preserving structure
        clean_text = ""

        # Process different elements to preserve structure
        for element in main_content.descendants:
            if element.name in ['h1', 'h2', 'h3', 'h4', 'h5', 'h6']:
                clean_text += f"\n\n{element.get_text().strip()}\n"
            elif element.name in ['p', 'div']:
                text = element.get_text().strip()
                if text and not any(parent.name in ['script', 'style'] for parent in element.parents):
                    clean_text += f"{text} "
            elif element.name in ['li']:
                clean_text += f"- {element.get_text().strip()} "
            elif element.name in ['code', 'pre']:
                clean_text += f"\n```\n{element.get_text().strip()}\n```\n"
            elif element.name == 'br':
                clean_text += "\n"
            elif element.name is None and hasattr(element, 'strip'):
                # Text nodes
                text = str(element).strip()
                if text:
                    clean_text += text + " "

        clean_text = re.sub(r'\s+', ' ', clean_text).strip()
    else:
        # Fallback to just getting all text
        clean_text = soup.get_text()
        clean_text = re.sub(r'\s+', ' ', clean_text).strip()

    logger.debug(f"Extracted content from {url}: {len(clean_text)} characters")

    return clean_text, page_title, section_title