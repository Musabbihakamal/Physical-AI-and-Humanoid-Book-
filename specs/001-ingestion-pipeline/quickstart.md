# Quickstart: Single-File Ingestion Pipeline

## Prerequisites
- Python 3.9+
- `uv` package manager
- Access to Cohere API (API key)
- Access to Qdrant vector database (local or remote)

## Setup
1. Create the backend directory:
   ```bash
   mkdir backend
   cd backend
   ```

2. Initialize the project with required dependencies:
   ```bash
   uv init
   uv add requests beautifulsoup4 cohere qdrant-client python-dotenv lxml tiktoken
   ```

3. Set up environment variables:
   ```bash
   # Create .env file with:
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=http://localhost:6333  # or your Qdrant server URL
   ```

## Usage
Run the ingestion pipeline:
```bash
python main.py --url https://your-docusaurus-site.com
```

## Configuration Options
- `--url`: Required. The URL of the Docusaurus site to ingest
- `--log-level`: Set logging level (DEBUG, INFO, WARNING, ERROR) [default: INFO]
- `--chunk-min-words`: Minimum chunk size in words [default: 400]
- `--chunk-max-words`: Maximum chunk size in words [default: 700]
- `--overlap-percent`: Percentage overlap between chunks [default: 10]
- `--batch-size`: Number of chunks to process per API call [default: 96]
- `--qdrant-url`: Qdrant server URL [default: http://localhost:6333]
- `--cohere-model`: Cohere embedding model [default: embed-english-v3.0]
- `--request-delay`: Delay between requests in seconds [default: 1]

## Expected Output
The pipeline will:
1. Discover all pages on the specified Docusaurus site
2. Extract clean text content while preserving structure
3. Split content into appropriately sized chunks with metadata
4. Generate embeddings using Cohere API
5. Store embeddings in Qdrant with associated metadata
6. Provide a summary of the ingestion process

## Troubleshooting
- If you encounter rate limits, increase the `--request-delay` value
- If memory usage is high, consider processing smaller sites or increasing system memory
- Ensure your Cohere API key has sufficient quota for the expected number of embeddings
- Check that your Qdrant server is running and accessible