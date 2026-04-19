# RAG Bot - Retrieval Augmented Generation Chatbot

A question-answering bot that uses retrieved context from ingested Docusaurus documentation to generate accurate responses.

## Features

- **Semantic Search**: Finds relevant content using vector similarity
- **Contextual Responses**: Generates answers based on retrieved documentation
- **Interactive Chat**: Engaging chat interface for asking questions
- **Source Citations**: Provides links to source documentation
- **Configurable**: Adjustable parameters for chunk retrieval and response generation

## Prerequisites

Before running the RAG bot, you need:

1. **Qdrant Vector Database**: Running instance with ingested documentation
2. **Cohere API Key**: For embedding generation and response creation
3. **Ingested Content**: Run the ingestion pipeline first to populate the database

## Setup

1. Install required dependencies:
```bash
pip install -r requirements.txt
```

2. Create a `.env` file with your Cohere API key:
```bash
COHERE_API_KEY=your_cohere_api_key_here
```

3. Ensure you have run the ingestion pipeline to populate your Qdrant collection

## Usage

### Interactive Chat Mode
```bash
python main.py
```

### Single Query Mode
```bash
python main.py --query "Your question here"
```

### With Custom Parameters
```bash
python main.py \
  --collection "custom_collection" \
  --qdrant-url "http://your-qdrant-server:6333" \
  --cohere-model "command-r-plus" \
  --max-chunks 7 \
  --threshold 0.4
```

## Configuration Options

- `--collection`: Qdrant collection name (default: docusaurus_embeddings)
- `--qdrant-url`: Qdrant server URL (default: http://localhost:6333)
- `--cohere-model`: Cohere model for generation (default: command-r-plus)
- `--max-chunks`: Max context chunks to retrieve (default: 5)
- `--threshold`: Similarity threshold (default: 0.3)
- `--query`: Single query for non-interactive mode

## How It Works

1. **Query Embedding**: Converts user question to vector embedding
2. **Vector Search**: Finds semantically similar chunks in Qdrant
3. **Context Assembly**: Combines relevant chunks with metadata
4. **Response Generation**: Uses Cohere to generate contextual answer
5. **Source Attribution**: Provides citations to original sources

## Requirements

- Python 3.8+
- Access to Cohere API
- Qdrant vector database with ingested content
- Internet connection for API calls