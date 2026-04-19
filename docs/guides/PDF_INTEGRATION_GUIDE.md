# RAG Bot Integration with PDF Textbook - Quick Start Guide

## Overview

Your RAG bot is now configured to work with your PDF textbook: `d:\Hackathon I_ Physical AI & Humanoid Robotics Textbook.pdf`

## Prerequisites

✅ Backend with RAG bot implementation
✅ Qdrant cloud instance configured
✅ Cohere API key configured
✅ PDF extraction support added

## Step-by-Step Initialization

### 1. Install Dependencies

```bash
cd "D:\PROJECT\HACKATHON 1\backend"
.venv\Scripts\activate
pip install -r requirements.txt
```

This will install PyPDF2 and all other required dependencies.

### 2. Ingest Your PDF Textbook

**Option A: Using the batch script (Easiest)**

```bash
cd "D:\PROJECT\HACKATHON 1\backend"
ingest_pdf.bat
```

**Option B: Using Python directly**

```bash
cd "D:\PROJECT\HACKATHON 1\backend"
.venv\Scripts\activate
python main.py --mode pdf --pdf-path "d:\Hackathon I_ Physical AI & Humanoid Robotics Textbook.pdf"
```

**What happens during ingestion:**
- Extracts text from all PDF pages
- Splits content into semantic chunks (400-700 words)
- Generates embeddings using Cohere
- Stores in Qdrant cloud vector database

**Expected output:**
```
INFO - Extracting text from PDF: d:\Hackathon I_ Physical AI & Humanoid Robotics Textbook.pdf
INFO - PDF has X pages
INFO - Created Y chunks from PDF
INFO - Generated Y embeddings
INFO - Stored Y embeddings in Qdrant
```

### 3. Start the Backend API

```bash
cd "D:\PROJECT\HACKATHON 1\backend"
.venv\Scripts\activate
set PORT=8001
python -m src.main
```

Backend will run on `http://localhost:8001`

### 4. Test the RAG Bot

**Option A: Interactive Chat Mode**

```bash
cd "D:\PROJECT\HACKATHON 1\backend"
.venv\Scripts\activate
python main.py --mode chat
```

Then ask questions like:
- "What is ROS 2?"
- "Explain NVIDIA Isaac Sim"
- "How do digital twins work?"
- "What are Vision-Language-Action systems?"

**Option B: Single Question Mode**

```bash
python main.py --mode ask --query "What is ROS 2?"
```

**Option C: Via Frontend Chat Widget**

1. Start frontend: `npm run frontend` (from project root)
2. Open `http://localhost:3000`
3. Click the floating chat button (bottom-right)
4. Ask questions about your textbook

### 5. Verify Integration

**Check backend health:**
```bash
curl http://localhost:8001/health
```

**Check RAG endpoint:**
```bash
curl http://localhost:8001/api/rag/health
```

**Test RAG query:**
```bash
curl -X POST http://localhost:8001/api/rag/query ^
  -H "Content-Type: application/json" ^
  -d "{\"question\":\"What is ROS 2?\",\"max_chunks\":5,\"threshold\":0.3}"
```

## Configuration Options

### Customize Chunking

```bash
python main.py --mode pdf ^
  --pdf-path "d:\Hackathon I_ Physical AI & Humanoid Robotics Textbook.pdf" ^
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

### Use Different Qdrant Collection

```bash
python main.py --mode pdf ^
  --pdf-path "d:\Hackathon I_ Physical AI & Humanoid Robotics Textbook.pdf" ^
  --collection my_textbook_collection
```

## Troubleshooting

### Issue: "PyPDF2 not installed"

**Solution:**
```bash
cd "D:\PROJECT\HACKATHON 1\backend"
.venv\Scripts\activate
pip install PyPDF2>=3.0.0
```

### Issue: "PDF file not found"

**Solution:** Check the file path. Use the full path with proper escaping:
```bash
python main.py --mode pdf --pdf-path "d:\Hackathon I_ Physical AI & Humanoid Robotics Textbook.pdf"
```

### Issue: "No text extracted from PDF"

**Solution:** Some PDFs are image-based. You may need OCR support. For now, ensure your PDF has selectable text.

### Issue: "Cohere API error"

**Solution:** Verify your API key in `.env`:
```
COHERE_API_KEY=your_cohere_api_key_here
```

### Issue: "Qdrant connection error"

**Solution:** Verify Qdrant credentials in `.env`:
```
QDRANT_URL=https://8326bd21-71bd-4306-9dfb-7719d5369ed2.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.f9vvic1mOncyz0WOS3AvLiqn7A7JAuZ15lGoyTOsfNs
```

### Issue: Port 8000 already in use

**Solution:** Use port 8001 or kill the process:
```bash
netstat -ano | findstr :8000
taskkill /PID <PID> /F
```

## Architecture

```
PDF Textbook
    ↓
PDF Extractor (PyPDF2)
    ↓
Text Chunker (400-700 words, 10% overlap)
    ↓
Embedder (Cohere embed-english-v3.0)
    ↓
Qdrant Vector Database (Cloud)
    ↓
RAG Bot (Cohere command-r-plus)
    ↓
User Interface (CLI or Web Widget)
```

## Files Created/Modified

- ✅ `backend/requirements.txt` - Added PyPDF2
- ✅ `backend/rag_bot/pdf_extractor.py` - PDF text extraction
- ✅ `backend/main.py` - Added PDF ingestion mode
- ✅ `backend/ingest_pdf.bat` - Easy PDF ingestion script

## Next Steps

1. **Run ingestion:** `cd backend && ingest_pdf.bat`
2. **Start backend:** `set PORT=8001 && python -m src.main`
3. **Test chat:** `python main.py --mode chat`
4. **Integrate with frontend:** Chat widget already configured
5. **Deploy to production:** Follow `DEPLOY_RAG_TO_PRODUCTION.md`

## Support

- **Logs:** Check `backend/pipeline.log` for detailed logs
- **API Docs:** `http://localhost:8001/docs`
- **Collection:** `docusaurus_embeddings` (default)

## Example Questions for Your Textbook

Based on the filename "Physical AI & Humanoid Robotics Textbook", try asking:

- "What are the key components of a humanoid robot?"
- "Explain physical AI and how it differs from traditional AI"
- "What sensors are used in humanoid robotics?"
- "How do humanoid robots achieve balance and locomotion?"
- "What are the applications of humanoid robotics?"
- "Explain the control systems used in humanoid robots"
- "What is the role of machine learning in physical AI?"

Enjoy your intelligent textbook assistant! 🤖📚
