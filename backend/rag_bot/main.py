"""
RAG Bot main module - recreated from missing file
"""
import os
import logging
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
import cohere
from anthropic import Anthropic
from dotenv import load_dotenv

# Load environment variables from parent directory
load_dotenv('../.env')

logger = logging.getLogger(__name__)

# Default configuration
DEFAULT_CONFIG = {
    "qdrant_url": os.getenv("QDRANT_URL", "http://localhost:6333"),
    "qdrant_api_key": os.getenv("QDRANT_API_KEY"),
    "rag_cohere_model": "command",
    "cohere_api_key": os.getenv("COHERE_API_KEY"),
    "collection_name": "docusaurus_embeddings"
}

class RagBot:
    """RAG Bot for querying documentation"""

    def __init__(self, qdrant_url: str, cohere_model: str, collection_name: str = "docusaurus_embeddings"):
        self.qdrant_url = qdrant_url
        self.cohere_model = cohere_model
        self.collection_name = collection_name

        # Initialize clients
        try:
            self.qdrant_client = QdrantClient(
                url=qdrant_url,
                api_key=os.getenv("QDRANT_API_KEY")
            )
            logger.info(f"Connected to Qdrant at {qdrant_url}")
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {e}")
            raise

        try:
            self.cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
            logger.info("Connected to Cohere")
        except Exception as e:
            logger.error(f"Failed to connect to Cohere: {e}")
            raise

        try:
            import httpx
            # Create httpx client without proxy configuration
            http_client = httpx.Client()
            self.anthropic_client = Anthropic(
                api_key=os.getenv("ANTHROPIC_API_KEY"),
                http_client=http_client
            )
            logger.info("Connected to Anthropic")
        except Exception as e:
            logger.error(f"Failed to connect to Anthropic: {e}")
            raise

    def retrieve_relevant_chunks(self, query: str, limit: int = 5, threshold: float = 0.3) -> List[Dict[str, Any]]:
        """Retrieve relevant chunks from Qdrant"""
        try:
            # Generate embedding for query using Cohere
            response = self.cohere_client.embed(
                texts=[query],
                model="embed-english-v3.0",
                input_type="search_query"
            )
            query_embedding = response.embeddings[0]

            # Use query_points method for vector search
            search_result = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=limit,
                score_threshold=threshold
            )

            chunks = []
            for point in search_result.points:
                chunks.append({
                    "content": point.payload.get("content", ""),
                    "url": point.payload.get("url", ""),
                    "page_title": point.payload.get("page_title", ""),
                    "section_title": point.payload.get("section_title", ""),
                    "score": point.score
                })

            logger.info(f"Retrieved {len(chunks)} chunks for query: {query[:50]}...")
            return chunks

        except Exception as e:
            logger.error(f"Failed to retrieve chunks: {e}")
            return []

    def generate_response(self, query: str, chunks: List[Dict[str, Any]]) -> str:
        """Generate response using simple text processing (no API required)"""
        try:
            if not chunks:
                return "I couldn't find relevant information about your question in the book content. Please try rephrasing your question or ask about topics covered in the Physical AI and Humanoid Robotics book."

            # Simple but effective response generation without API calls
            response_parts = []

            # Add a brief introduction
            response_parts.append(f"Based on the book content, here's what I found about '{query}':\n")

            # Process and format the chunks
            for i, chunk in enumerate(chunks[:3], 1):  # Limit to top 3 chunks
                section_title = chunk.get('section_title', 'Unknown Section')
                page_title = chunk.get('page_title', 'Unknown Page')
                content = chunk.get('content', '')

                # Clean and truncate content
                clean_content = content.strip()
                if len(clean_content) > 300:
                    clean_content = clean_content[:300] + "..."

                response_parts.append(f"**{i}. From {section_title} ({page_title}):**")
                response_parts.append(f"{clean_content}\n")

            # Add helpful conclusion
            if len(chunks) > 3:
                response_parts.append(f"Found {len(chunks)} total relevant sections. The above shows the most relevant content.")

            response_parts.append("\n💡 **Tip:** For more detailed information, check the full chapters in the book!")

            final_response = "\n".join(response_parts)
            logger.info(f"Generated free response for query: {query[:50]}...")
            return final_response

        except Exception as e:
            logger.error(f"Failed to generate free response: {str(e)}", exc_info=True)
            return "I encountered an error while processing your question. Please try asking again or rephrase your question."