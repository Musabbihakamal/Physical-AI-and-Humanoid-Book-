# Book + RAG Bot + Multi-Agent System - Backend

This is the backend service for the multi-agent book generation system, providing APIs for specialized AI agents including Research, Writer, Editor, RAG Engineer, Developer, Documentation, and Project Planner agents.

## Features

- Multi-agent architecture with specialized subagents
- RAG-based chatbot for book content interaction
- User profile management for personalized experiences
- Content generation and management
- API endpoints for all agent interactions

## Tech Stack

- Python 3.11+
- FastAPI for web framework
- SQLAlchemy for database ORM
- PostgreSQL for relational data
- Qdrant for vector storage (RAG)
- OpenAI API for embeddings and agent capabilities

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Set up environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your API keys and database URLs
   ```

3. Run the application:
   ```bash
   python -m src.main
   ```

## API Endpoints

- `/api/agents/` - Endpoints for all agent interactions
- `/api/rag/` - Endpoints for RAG functionality
- `/api/content/` - Endpoints for content management
- `/docs` - Interactive API documentation

## Environment Variables

- `DATABASE_URL` - PostgreSQL database URL
- `OPENAI_API_KEY` - OpenAI API key for embeddings and agent capabilities
- `QDRANT_URL` - Qdrant vector database URL
- `QDRANT_API_KEY` - Qdrant API key (if using cloud)
- `SECRET_KEY` - Secret key for JWT tokens

## Running Tests

```bash
pytest tests/
```