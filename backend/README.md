# Backend - Physical AI Book RAG System

FastAPI backend for the Physical AI & Humanoid Robotics book with RAG chatbot capabilities.

## Quick Start

### 1. Setup
```bash
setup.bat
```
This will:
- Create virtual environment
- Install all dependencies
- Verify installation

### 2. Ingest Content
```bash
ingest.bat
```
Crawls and ingests content from the deployed Docusaurus site into Qdrant vector database.

### 3. Start Server
```bash
start.bat
```
Starts the FastAPI backend on port 8001.

## Manual Commands

### Activate Virtual Environment
```bash
.venv\Scripts\activate
```

### Run Ingestion Pipeline
```bash
python main.py --mode ingest --url https://physical-ai-and-humanoid-book.vercel.app/
```

### Start Backend Server
```bash
set PORT=8001
python -m src.main
```

### Interactive Chat Mode
```bash
python main.py --mode chat
```

### Single Query Mode
```bash
python main.py --mode ask --query "What is ROS 2?"
```

## Project Structure

```
backend/
├── src/
│   ├── api/           # FastAPI routes
│   ├── config/        # Configuration management
│   ├── database/      # Database connections
│   ├── models/        # Data models
│   ├── services/      # Business logic
│   └── utils/         # Utility functions
├── rag_bot/           # RAG bot implementation
│   ├── chunker.py     # Text chunking
│   ├── crawler.py     # Web crawling
│   ├── embedder.py    # Embedding generation
│   ├── extractor.py   # Content extraction
│   └── storage.py     # Vector storage
├── main.py            # Ingestion pipeline & RAG CLI
├── requirements.txt   # Python dependencies
├── setup.bat          # Setup script
├── start.bat          # Start server script
└── ingest.bat         # Content ingestion script
```

## Environment Variables

Create a `.env` file with:

```env
# Cohere API
COHERE_API_KEY=your_cohere_api_key

# Qdrant Vector Database
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key

# Database
DATABASE_URL=postgresql://user:pass@host/db

# Server
PORT=8001
```

## API Endpoints

- `GET /health` - Health check
- `GET /api/rag/health` - RAG system health
- `POST /api/rag/query` - Query the RAG system
- `POST /api/auth/register` - User registration
- `POST /api/auth/login` - User login

Full API documentation: http://localhost:8001/docs

## Development

### Install Dependencies
```bash
pip install -r requirements.txt
```

### Run Tests
```bash
pytest
```

### Code Style
Follow PEP 8 guidelines. Use type hints where applicable.

## Deployment

See `docs/guides/DEPLOY_RAG_TO_PRODUCTION.md` for production deployment instructions.

## Support

- API Documentation: http://localhost:8001/docs
- Issues: Check backend terminal for error messages
- Logs: `logs/app.log`
