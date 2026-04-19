"""
Data models for the ingestion pipeline and RAG bot
"""

from dataclasses import dataclass
from typing import Dict, List, Optional


@dataclass
class PageData:
    """
    Represents a single page from the Docusaurus site during processing
    """
    url: str
    raw_html: str
    clean_text: str
    page_title: str
    section_title: str
    discovered_at: str


@dataclass
class ChunkData:
    """
    Represents a text chunk created from page content for embedding
    """
    id: str
    content: str
    url: str
    page_title: str
    section_title: str
    word_count: int
    embedding_vector: Optional[List[float]]
    source_hash: str
    created_at: str


@dataclass
class ProcessingState:
    """
    Tracks the current state of the ingestion process
    """
    visited_urls: set
    processed_pages: int
    generated_chunks: int
    stored_embeddings: int
    start_time: str
    current_stage: str
    errors: List[Dict]


@dataclass
class EmbeddingRequest:
    """
    Represents a batch of chunks to be sent to the Cohere API
    """
    chunks: List[ChunkData]
    model: str
    batch_size: int
    retry_count: int


@dataclass
class QdrantPayload:
    """
    Represents the payload structure for storing in Qdrant vector database
    """
    vector: List[float]
    payload: Dict
    id: str


@dataclass
class RagResponse:
    """
    Represents a response from the RAG bot
    """
    query: str
    answer: str
    context_chunks: List[Dict]
    sources: List[str]
    confidence_score: float