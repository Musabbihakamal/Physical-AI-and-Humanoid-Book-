"""
Chunker module for the ingestion pipeline
"""

import hashlib
from datetime import datetime
from typing import Dict, List

from backend.rag_bot.config import DEFAULT_CONFIG


def generate_content_hash(content: str) -> str:
    """
    Generate SHA-256 hash of content for idempotency.

    Args:
        content: Content to hash

    Returns:
        SHA-256 hash string
    """
    return hashlib.sha256(content.encode()).hexdigest()


def chunk_text_with_overlap(
    text: str,
    min_words: int = 400,
    max_words: int = 700,
    overlap_percent: int = 10
) -> List[Dict]:
    """
    Split text into overlapping chunks.

    Args:
        text: Text to chunk
        min_words: Minimum number of words per chunk
        max_words: Maximum number of words per chunk
        overlap_percent: Percentage of overlap between chunks

    Returns:
        List of chunk dictionaries with content and metadata
    """
    import logging
    logger = logging.getLogger(__name__)

    if not text.strip():
        return []

    words = text.split()
    chunks = []

    # Calculate chunk parameters
    max_chunk_size = max_words
    overlap_size = int(max_chunk_size * overlap_percent / 100)

    start_idx = 0
    while start_idx < len(words):
        # Determine the end index for this chunk
        end_idx = start_idx + max_chunk_size

        # If this is the last chunk and it's too small, include it as is
        if end_idx >= len(words) and (end_idx - start_idx) >= min_words:
            chunk_words = words[start_idx:end_idx]
        elif end_idx >= len(words):
            # Last chunk is too small, extend it to include more words if possible
            chunk_words = words[start_idx:]
        else:
            # Regular chunk
            chunk_words = words[start_idx:end_idx]

        # Ensure the chunk meets minimum size requirements unless it's the very last chunk
        if len(chunk_words) < min_words and end_idx < len(words):
            # This chunk is too small and not the last chunk, skip it and move to next position
            start_idx = end_idx
            continue

        chunk_text = ' '.join(chunk_words)

        # Create chunk with metadata
        chunk_data = {
            "content": chunk_text,
            "word_count": len(chunk_words),
            "source_hash": generate_content_hash(chunk_text),
            "created_at": datetime.utcnow().isoformat()
        }

        chunks.append(chunk_data)

        # Move start index considering overlap
        if end_idx >= len(words):
            # Last chunk, no more chunks
            break

        # Move start index by (chunk_size - overlap_size) to achieve desired overlap
        start_idx = end_idx - overlap_size

        # Ensure we don't go backward in case overlap_size > chunk_size
        if start_idx <= start_idx:
            start_idx = start_idx + max_chunk_size

    logger.info(f"Split content into {len(chunks)} chunks (min: {min_words}, max: {max_words}, overlap: {overlap_percent}%)")
    return chunks