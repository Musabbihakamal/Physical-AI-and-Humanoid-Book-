# Quick Start: RAG Bot with Physical AI Book

## 🚀 3-Step Setup

### 1. Ingest Your Book from Vercel
```bash
cd "D:\PROJECT\HACKATHON 1\backend"
ingest_vercel_book.bat
```
⏱️ Takes 2-5 minutes

### 2. Start Backend
```bash
cd "D:\PROJECT\HACKATHON 1\backend"
.venv\Scripts\activate
set PORT=8001
python -m src.main
```

### 3. Test It!

**Option A: CLI Chat**
```bash
python main.py --mode chat
```

**Option B: Frontend Widget**
```bash
cd "D:\PROJECT\HACKATHON 1"
npm run frontend
```
Open `http://localhost:3000` and click the chat button!

## 📖 Try These Questions

- "What is ROS 2?"
- "Explain NVIDIA Isaac Sim"
- "What are Vision-Language-Action systems?"
- "How do digital twins work in robotics?"

## 📚 Full Documentation

See `VERCEL_INTEGRATION_GUIDE.md` for complete details.

## ✅ Verify Setup

```bash
# Check backend health
curl http://localhost:8001/health

# Check RAG system
curl http://localhost:8001/api/rag/health
```

That's it! Your intelligent textbook assistant is ready! 🤖
