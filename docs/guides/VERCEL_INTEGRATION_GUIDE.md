# RAG Bot Integration with Physical AI & Humanoid Robotics Book (Vercel)

## Overview

Integrate your RAG bot with the deployed Physical AI & Humanoid Robotics book at:
**https://physical-ai-and-humanoid-book.vercel.app/**

## Architecture

```
Vercel Deployment (Your Book)
    ↓
Web Crawler (discovers all pages)
    ↓
Content Extractor (clean text from HTML)
    ↓
Text Chunker (400-700 words, 10% overlap)
    ↓
Embedder (Cohere embed-english-v3.0)
    ↓
Qdrant Vector Database (Cloud)
    ↓
RAG Bot (Cohere command-r-plus)
    ↓
Chat Widget (Frontend) + CLI
```

## Step-by-Step Integration

### Step 1: Verify Environment Setup

Check your `.env` file has these credentials:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=https://your-qdrant-instance.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
```

### Step 2: Install Dependencies

```bash
cd "D:\PROJECT\HACKATHON 1\backend"
.venv\Scripts\activate
pip install -r requirements.txt
```

### Step 3: Ingest Content from Vercel

**Option A: Using batch script (Recommended)**

```bash
cd "D:\PROJECT\HACKATHON 1\backend"
ingest_vercel_book.bat
```

**Option B: Using Python directly**

```bash
cd "D:\PROJECT\HACKATHON 1\backend"
.venv\Scripts\activate
python main.py --mode ingest --url https://physical-ai-and-humanoid-book.vercel.app/
```

**What happens during ingestion:**

1. **Crawling** - Discovers all pages on your Vercel site
   - Follows all internal links
   - Respects robots.txt
   - Typical book has 50-100+ pages

2. **Extraction** - Extracts clean text from each page
   - Removes navigation, headers, footers
   - Preserves headings and structure
   - Keeps code blocks and lists

3. **Chunking** - Splits content into semantic chunks
   - 400-700 words per chunk
   - 10% overlap between chunks
   - Preserves context

4. **Embedding** - Generates vector embeddings
   - Uses Cohere embed-english-v3.0
   - 1024-dimensional vectors
   - Batch processing (96 texts/batch)

5. **Storage** - Stores in Qdrant cloud
   - Collection: `docusaurus_embeddings`
   - Includes metadata (URL, title, section)
   - Enables semantic search

**Expected output:**

```
INFO - Starting crawl from: https://physical-ai-and-humanoid-book.vercel.app/
INFO - Crawling completed. Discovered 50+ pages.
INFO - Extracted content from 50+ pages, creating 200+ chunks
INFO - Generated 200+ embeddings successfully
INFO - Stored 200+ embeddings in Qdrant
INFO - Pipeline completed in 120.45 seconds
```

**Time estimate:** 2-5 minutes depending on book size

### Step 4: Start the Backend API

```bash
cd "D:\PROJECT\HACKATHON 1\backend"
.venv\Scripts\activate
set PORT=8001
python -m src.main
```

Backend runs on: `http://localhost:8001`

**Verify backend is running:**

```bash
curl http://localhost:8001/health
# Expected: {"status":"healthy","service":"book-system"}

curl http://localhost:8001/api/rag/health
# Expected: {"status":"healthy","collection":"docusaurus_embeddings","ready":true}
```

### Step 5: Test the RAG Bot

**Option A: Interactive Chat (CLI)**

```bash
cd "D:\PROJECT\HACKATHON 1\backend"
.venv\Scripts\activate
python main.py --mode chat
```

**Try these questions:**
- "What is ROS 2?"
- "Explain NVIDIA Isaac Sim"
- "How do I create a digital twin in Gazebo?"
- "What are Vision-Language-Action systems?"
- "What topics are covered in Module 1?"
- "Explain the difference between physical AI and traditional AI"

**Option B: Single Question**

```bash
python main.py --mode ask --query "What is ROS 2?"
```

**Option C: Via API**

```bash
curl -X POST http://localhost:8001/api/rag/query ^
  -H "Content-Type: application/json" ^
  -d "{\"question\":\"What is ROS 2?\",\"max_chunks\":5,\"threshold\":0.3}"
```

### Step 6: Use the Frontend Chat Widget

The chat widget is already integrated in your frontend!

**Start the frontend:**

```bash
cd "D:\PROJECT\HACKATHON 1"
npm run frontend
```

**Access at:** `http://localhost:3000`

**Features:**
- Floating chat button (bottom-right corner)
- Real-time answers from your book
- Source citations with page links
- Conversation history

**Frontend configuration:**

The widget is configured in:
- `frontend/src/Root.js` - Widget loaded on all pages
- `frontend/src/components/RAGChatWidget/` - Chat interface
- `frontend/src/constants/apiConfig.js` - Backend connection

**Current backend URL:** `http://localhost:8001`

## Configuration Options

### Customize Chunking

```bash
python main.py --mode ingest ^
  --url https://physical-ai-and-humanoid-book.vercel.app/ ^
  --chunk-min-words 300 ^
  --chunk-max-words 600 ^
  --overlap-percent 15
```

### Adjust RAG Parameters

```bash
python main.py --mode ask ^
  --query "Your question" ^
  --max-chunks 10 ^
  --threshold 0.2
```

### Change Collection Name

```bash
python main.py --mode ingest ^
  --url https://physical-ai-and-humanoid-book.vercel.app/ ^
  --collection physical_ai_book
```

## Troubleshooting

### Issue: "No pages discovered"

**Possible causes:**
- Network connectivity issues
- Vercel site is down
- robots.txt blocking crawler

**Solution:**
```bash
# Test if site is accessible
curl https://physical-ai-and-humanoid-book.vercel.app/
```

### Issue: "Cohere API error"

**Solution:** Check your API key in `.env`:
```bash
# Verify key is set
cd "D:\PROJECT\HACKATHON 1\backend"
type .env | findstr COHERE_API_KEY
```

### Issue: "Qdrant connection error"

**Solution:** Verify Qdrant credentials:
```bash
# Test connection
python -c "from qdrant_client import QdrantClient; client = QdrantClient(url='https://8326bd21-71bd-4306-9dfb-7719d5369ed2.us-east4-0.gcp.cloud.qdrant.io', api_key='eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.f9vvic1mOncyz0WOS3AvLiqn7A7JAuZ15lGoyTOsfNs'); print('Connected!')"
```

### Issue: "Chat widget shows 'Backend not running'"

**Solutions:**
1. Ensure backend is running on port 8001
2. Check `frontend/src/constants/apiConfig.js` has correct port
3. Verify CORS settings in backend

### Issue: "No relevant information found"

**Solution:** Re-run ingestion to ensure content is in database:
```bash
cd "D:\PROJECT\HACKATHON 1\backend"
ingest_vercel_book.bat
```

## Re-ingestion (When You Update Your Book)

When you update content on Vercel, re-run ingestion:

```bash
cd "D:\PROJECT\HACKATHON 1\backend"
.venv\Scripts\activate
python main.py --mode ingest --url https://physical-ai-and-humanoid-book.vercel.app/
```

The system will:
- Skip duplicate content (using content hashing)
- Add new pages
- Update modified pages

## Production Deployment

To deploy the RAG bot to production, see:
- `DEPLOY_RAG_TO_PRODUCTION.md` - Full deployment guide
- Deploy backend to Railway/Render/Fly.io
- Configure Vercel environment variables
- Update frontend to use production backend URL

## Files Reference

- `backend/ingest_vercel_book.bat` - Ingest from Vercel
- `backend/main.py` - RAG bot + ingestion pipeline
- `backend/rag_bot/` - RAG bot modules
- `frontend/src/components/RAGChatWidget/` - Chat widget
- `backend/pipeline.log` - Detailed logs

## API Endpoints

- `GET /health` - Backend health check
- `GET /api/rag/health` - RAG system health
- `POST /api/rag/query` - Ask questions
- `GET /docs` - Interactive API documentation

## Example Questions for Your Book

Based on Physical AI & Humanoid Robotics:

**Module 1: Foundations**
- "What is physical AI?"
- "How does physical AI differ from traditional AI?"
- "What are the key components of physical AI systems?"

**Module 2: ROS 2**
- "What is ROS 2 and why is it important?"
- "How do I create a ROS 2 workspace?"
- "Explain ROS 2 nodes and topics"

**Module 3: Simulation**
- "What is NVIDIA Isaac Sim?"
- "How do I create a digital twin in Gazebo?"
- "Compare Isaac Sim and Gazebo"

**Module 4: Hardware**
- "What sensors are used in humanoid robots?"
- "Explain actuators in robotics"
- "What is the Unitree G1 robot?"

**Module 5: AI Integration**
- "What are Vision-Language-Action models?"
- "How does reinforcement learning apply to robotics?"
- "Explain embodied AI"

**Module 6: Applications**
- "What are real-world applications of humanoid robots?"
- "How are humanoid robots used in healthcare?"
- "What are the ethical considerations in humanoid robotics?"

## Next Steps

1. ✅ Run ingestion: `ingest_vercel_book.bat`
2. ✅ Start backend: `set PORT=8001 && python -m src.main`
3. ✅ Test CLI chat: `python main.py --mode chat`
4. ✅ Start frontend: `npm run frontend`
5. ✅ Test chat widget at `http://localhost:3000`
6. 🚀 Deploy to production (optional)

Enjoy your intelligent Physical AI & Humanoid Robotics assistant! 🤖📚
