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
        """Generate response using Anthropic Claude API"""
        try:
            # Prepare context from chunks
            context = "\n\n".join([
                f"Source: {chunk['page_title']} - {chunk['section_title']}\n{chunk['content']}"
                for chunk in chunks
            ])

            message = self.anthropic_client.messages.create(
                model="claude-opus-4-7",
                max_tokens=1000,
                messages=[
                    {
                        "role": "user",
                        "content": f"""Based on the following documentation context, answer the user's question about Physical AI and Humanoid Robotics.

Context:
{context}

Question: {query}

Provide a helpful and accurate answer based on the context above. If the context doesn't contain enough information, say so."""
                    }
                ]
            )

            answer = message.content[0].text.strip()
            logger.info(f"Generated response for query: {query[:50]}...")
            return answer

        except Exception as e:
            logger.error(f"Failed to generate response: {str(e)}", exc_info=True)
            return "I'm sorry, I encountered an error while generating a response. Please try again."