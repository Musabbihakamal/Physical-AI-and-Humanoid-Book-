# RAG Bot Integration Guide

## Current Integration Status ✓

Your RAG bot is **fully integrated** with the book. Here's what's already set up:

### Architecture
```
Docusaurus Book (Port 3000)
    ↓
RAG Chat Widget (floating button on all pages)
    ↓
FastAPI Backend (Port 8001) → /api/rag/query
    ↓
RagBot → Qdrant Vector DB + Cohere API
    ↓
Intelligent answers with source citations
```

### Components Already Configured

✅ **Frontend Integration**
- `frontend/src/Root.js` - RAG widget loaded on all pages
- `frontend/src/components/RAGChatWidget/` - Chat interface
- `frontend/src/constants/apiConfig.js` - Backend connection

✅ **Backend API**
- `backend/src/api/rag_routes.py` - RAG endpoints
- `backend/main.py` - Ingestion pipeline + RAG bot
- Routes registered in `backend/src/api/main.py`

✅ **Configuration**
- Cohere API key configured
- Qdrant cloud instance configured
- Environment variables set in `.env`

## Quick Start Guide

### Step 1: Start the Frontend

```bash
# Terminal 1
cd "D:\PROJECT\HACKATHON 1"
npm run frontend
```

Frontend will run on `http://localhost:3000`

### Step 2: Ingest Book Content

Before the RAG bot can answer questions, you need to ingest your book content into the vector database:

```bash
# Terminal 2
cd "D:\PROJECT\HACKATHON 1\backend"
python main.py --mode ingest --url http://localhost:3000
```

This will:
- Crawl all pages from your Docusaurus site
- Extract text content from each page
- Split content into semantic chunks (400-700 words)
- Generate embeddings using Cohere
- Store in Qdrant vector database

**Expected output:**
```
INFO - Starting crawl from: http://localhost:3000
INFO - Discovered 50+ pages
INFO - Extracted text from 50+ pages
INFO - Created 200+ chunks
INFO - Generated embeddings for 200+ chunks
INFO - Stored 200+ chunks in Qdrant
INFO - Ingestion complete!
```

### Step 3: Start the Backend API

**Note:** Port 8000 is currently in use. Start backend on port 8001:

```bash
# Terminal 2 (after ingestion completes)
cd "D:\PROJECT\HACKATHON 1\backend"
set PORT=8001
python -m src.main
```

Or modify the frontend API config to use port 8001:

Edit `frontend/src/constants/apiConfig.js` line 21:
```javascript
if (hostname === 'localhost' || hostname === '127.0.0.1') {
  return `${protocol}//${hostname}:8001`;  // Changed from 8000 to 8001
}
```

Backend will run on `http://localhost:8001`

### Step 4: Use the RAG Chat Widget

1. Open browser to `http://localhost:3000`
2. Look for the floating chat button (bottom-right corner)
3. Click to open the chat widget
4. Ask questions about your book:
   - "What is ROS 2?"
   - "Explain NVIDIA Isaac Sim"
   - "How do I create a digital twin in Gazebo?"
   - "What are Vision-Language-Action systems?"

The bot will provide answers with source citations linking back to specific pages.

## Alternative: CLI Mode

You can also use the RAG bot from the command line:

### Interactive Chat Mode
```bash
cd "D:\PROJECT\HACKATHON 1\backend"
python main.py --mode chat
```

### Single Question Mode
```bash
python main.py --mode ask --query "What is ROS 2?"
```

## Verification Steps

### 1. Check Backend Health
```bash
curl http://localhost:8001/health
# Expected: {"status":"healthy","service":"book-system"}
```

### 2. Check RAG Endpoint
```bash
curl http://localhost:8001/api/rag/health
# Expected: {"status":"healthy","collection":"docusaurus_embeddings","ready":true}
```

### 3. Test RAG Query
```bash
curl -X POST http://localhost:8001/api/rag/query \
  -H "Content-Type: application/json" \
  -d '{"question":"What is ROS 2?","max_chunks":5,"threshold":0.3}'
```

## Troubleshooting

### Issue: "Backend not running" error in chat widget

**Solution:** Make sure backend is running on the correct port and frontend API config matches.

### Issue: "No relevant information found"

**Solution:** Run the ingestion pipeline first to populate the vector database.

### Issue: Port 8000 already in use

**Solution:** Use port 8001 as shown above, or stop the process on port 8000:
```bash
# Find process ID
netstat -ano | findstr :8000
# Kill process (replace PID with actual process ID)
taskkill /PID <PID> /F
```

### Issue: Cohere API errors

**Solution:** Verify your Cohere API key in `.env`:
```bash
COHERE_API_KEY=your_cohere_api_key_here
```

### Issue: Qdrant connection errors

**Solution:** Verify Qdrant credentials in `.env`:
```bash
QDRANT_URL=https://8326bd21-71bd-4306-9dfb-7719d5369ed2.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.f9vvic1mOncyz0WOS3AvLiqn7A7JAuZ15lGoyTOsfNs
```

## Advanced Configuration

### Customize Chunk Size
```bash
python main.py --mode ingest --url http://localhost:3000 \
  --chunk-min-words 300 \
  --chunk-max-words 600 \
  --overlap-percent 15
```

### Adjust RAG Parameters
```bash
python main.py --mode ask \
  --query "Your question" \
  --max-chunks 10 \
  --threshold 0.2
```

### Change Collection Name
```bash
python main.py --mode ingest --url http://localhost:3000 \
  --collection my_custom_collection
```

## Next Steps

1. **Ingest Content**: Run the ingestion pipeline to populate the vector database
2. **Start Backend**: Launch the FastAPI backend on port 8001
3. **Test Widget**: Open the frontend and try the chat widget
4. **Monitor Logs**: Check `backend/pipeline.log` for detailed logs
5. **Iterate**: Re-run ingestion when you update book content

## Architecture Details

### Ingestion Pipeline
1. **Crawler** - Discovers all pages on the Docusaurus site
2. **Extractor** - Extracts clean text from HTML
3. **Chunker** - Splits text into semantic chunks with overlap
4. **Embedder** - Generates embeddings using Cohere
5. **Storage** - Stores chunks and embeddings in Qdrant

### RAG Query Flow
1. User asks question in chat widget
2. Frontend sends POST to `/api/rag/query`
3. Backend generates query embedding
4. Qdrant performs semantic search
5. Top chunks retrieved based on similarity
6. Cohere generates answer using context
7. Response returned with source citations

## Support

- **API Documentation**: http://localhost:8001/docs
- **Logs**: `backend/pipeline.log`
- **Issues**: Check backend terminal for error messages
