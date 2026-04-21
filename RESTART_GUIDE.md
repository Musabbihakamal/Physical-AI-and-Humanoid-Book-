# Complete Restart Guide

## Quick Start (After System Restart)

### Windows
```bash
cd "D:\PROJECT\BOOK WITH RAGBOT"
start-dev.bat
```

### Mac/Linux
```bash
cd "D:\PROJECT\BOOK WITH RAGBOT"
./start-dev.sh
```

Then open: **http://localhost:3000**

---

## Services Overview

| Service | Port | Purpose |
|---------|------|---------|
| Frontend (React) | 3000 | Web UI |
| Backend (FastAPI) | 8001 | APIs (RAG, Translator, Auth) |
| API Docs | 8001/docs | Swagger documentation |

---

## If Services Don't Start

### 1. Run Health Check
```bash
health-check.bat    # Windows
./health-check.sh   # Mac/Linux
```

### 2. Common Issues

**Port already in use:**
```bash
# Windows
netstat -ano | findstr :8001
wmic process where processid=<PID> delete

# Mac/Linux
lsof -i :8001
kill -9 <PID>
```

**Virtual environment not found:**
```bash
npm run setup
```

**Translation not working:**
- Check `backend/.env` has ANTHROPIC_API_KEY
- Check `frontend/.env` points to http://localhost:8001
- Restart services

**RAG Bot not responding:**
- Check QDRANT_URL and QDRANT_API_KEY in `backend/.env`
- Check COHERE_API_KEY is set
- Run: `cd backend/rag_bot && python index_content.py`

**Authentication failing:**
- Check OAuth credentials in `backend/.env`
- Verify redirect URIs match OAuth app settings

---

## Environment Variables

### Backend (`backend/.env`)
```
ANTHROPIC_API_KEY=sk-ant-...
COHERE_API_KEY=...
QDRANT_URL=https://...
QDRANT_API_KEY=...
SECRET_KEY=your-secret-key
```

### Frontend (`frontend/.env`)
```
REACT_APP_BACKEND_URL=http://localhost:8001
REACT_APP_RAG_ENABLED=true
```

---

## Test Commands

```bash
# Test translation
curl -X POST http://localhost:8001/api/translate -H "Content-Type: application/json" -d '{"text":"Hello","target_language":"es"}'

# Test RAG bot
curl -X POST http://localhost:8001/api/rag/query -H "Content-Type: application/json" -d '{"question":"What is ROS 2?"}'

# Test health
curl http://localhost:8001/health
```

---

## Files Created for Restart Fix

- `start-dev.bat` / `start-dev.sh` - Startup scripts
- `health-check.bat` / `health-check.sh` - Diagnostics
- This guide - Complete documentation

Your translation and RAG bot will work reliably after restart using the startup scripts.