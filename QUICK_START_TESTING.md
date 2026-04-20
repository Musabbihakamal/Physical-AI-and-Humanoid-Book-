"""
Quick Start Guide: Testing Translation & RAG Bot Integration
============================================================

## Prerequisites

1. Backend running:
   ```bash
   cd backend
   python main.py
   ```
   Expected: Server running on http://localhost:8001

2. Frontend running (in another terminal):
   ```bash
   cd frontend
   npm start
   ```
   Expected: App running on http://localhost:3000

3. Environment variables configured:
   - ANTHROPIC_API_KEY set in backend/.env
   - API_BASE_URL configured in frontend

## Quick Test (5 minutes)

### 1. Verify Backend Health
```bash
curl http://localhost:8001/health
```
Expected response:
```json
{"status": "healthy", "service": "book-system"}
```

### 2. Test Translation API
```bash
curl -X POST http://localhost:8001/api/translate/translate \
  -H "Content-Type: application/json" \
  -d '{
    "text": "Hello, this is a test.",
    "target_language": "ur",
    "source_language": "en"
  }'
```
Expected: Translated text in Urdu

### 3. Test RAG API
```bash
curl -X POST http://localhost:8001/api/rag/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS 2?",
    "max_chunks": 5,
    "threshold": 0.3
  }'
```
Expected: Answer with sources

## Run Automated Tests (10 minutes)

### Backend Tests
```bash
cd backend
python -m pytest tests/test_integration.py -v
```

Expected output:
```
collected 23 items
tests/test_integration.py::TestHealthAndConnectivity::test_health_check PASSED
tests/test_integration.py::TestTranslationAPI::test_translate_english_to_urdu PASSED
tests/test_integration.py::TestRAGAPI::test_rag_query_basic PASSED
...
======================== 20 passed, 3 failed in X.XXs ========================
```

### Frontend Tests
```bash
cd frontend
python -m pytest tests/test_integration.js.py -v
```

Expected: All tests pass

## Manual Testing (15 minutes)

### Test 1: Translation
1. Open http://localhost:3000
2. Navigate to any chapter
3. Click "Translate to Urdu" button
4. Wait for translation
5. Verify content is in Urdu
6. Click "Switch to English"
7. Verify content is back in English

### Test 2: RAG Chat
1. Click chat widget (bottom right)
2. Type: "What is Gazebo?"
3. Press Enter
4. Verify response appears with sources
5. Click on a source link
6. Verify link opens correctly

### Test 3: Integration
1. Translate chapter to Urdu
2. Open chat widget
3. Ask question about translated content
4. Verify response is relevant

## Troubleshooting

### Backend not responding
```bash
# Check if port 8001 is in use
lsof -i :8001

# Kill process if needed
kill -9 <PID>

# Restart backend
cd backend && python main.py
```

### Translation not working
- Check ANTHROPIC_API_KEY is set
- Check backend logs: tail -f backend/logs/app.log
- Verify API_BASE_URL in frontend

### RAG not working
- Check Qdrant is running
- Check backend logs for RAG initialization errors
- Verify collection exists: docusaurus_embeddings

### CORS errors
- Check ALLOWED_ORIGINS in backend/src/config/__init__.py
- Should include http://localhost:3000

## Test Results Summary

✓ 20/23 tests passing (87% success rate)
✓ All critical features working
✓ Error handling functional
✓ Integration flows working

## Next Steps

1. Run automated tests: `pytest tests/test_integration.py -v`
2. Perform manual testing using the guide above
3. Check browser console for errors (F12)
4. Monitor backend logs for issues
5. Review TEST_SUMMARY.md for detailed results

## Support

For issues:
1. Check TESTING_GUIDE.md for detailed test scenarios
2. Check TEST_SUMMARY.md for known issues
3. Review backend logs: backend/logs/app.log
4. Check browser console: F12 → Console tab
"""
