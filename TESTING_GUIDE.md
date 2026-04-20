"""
Manual Testing Guide for Translation and RAG Bot Integration
============================================================

This guide provides step-by-step instructions for manually testing the translation
and RAG bot frontend-backend integration.

## Prerequisites
- Backend running on http://localhost:8001
- Frontend running on http://localhost:3000
- Browser developer console open (F12)
- .env file configured with API keys

## Test Scenarios

### 1. HEALTH CHECK & CONNECTIVITY
Test: Verify backend is accessible
Steps:
  1. Open browser console
  2. Run: fetch('http://localhost:8001/health').then(r => r.json()).then(console.log)
  3. Verify response: {"status": "healthy", "service": "book-system"}
Expected: 200 OK, healthy status

### 2. TRANSLATION - BASIC FLOW
Test: Translate English to Urdu
Steps:
  1. Navigate to any book chapter
  2. Click "Translate to Urdu" button
  3. Wait for translation to complete
  4. Verify content is displayed in Urdu
  5. Click "Switch to English" to revert
Expected: Content translates and reverts without errors

### 3. TRANSLATION - CODE PRESERVATION
Test: Verify code blocks are preserved during translation
Steps:
  1. Navigate to a chapter with code examples
  2. Click "Translate to Urdu"
  3. Verify code blocks remain unchanged
  4. Check console for no errors
Expected: Code blocks preserved, no console errors

### 4. TRANSLATION - TECHNICAL TERMS
Test: Verify technical terms are preserved
Steps:
  1. Navigate to chapter mentioning ROS 2, Gazebo, Isaac
  2. Click "Translate to Urdu"
  3. Verify technical terms appear unchanged in translation
Expected: ROS 2, Gazebo, Isaac remain in translated text

### 5. TRANSLATION - ERROR HANDLING
Test: Handle translation errors gracefully
Steps:
  1. Disconnect internet or block API
  2. Click "Translate to Urdu"
  3. Verify error message appears
  4. Verify UI remains responsive
Expected: User-friendly error message, no crashes

### 6. RAG CHAT - BASIC QUERY
Test: Ask a question in RAG chat
Steps:
  1. Click chat widget button (bottom right)
  2. Type: "What is ROS 2?"
  3. Press Enter or click Send
  4. Wait for response
Expected: Response appears with sources, no errors

### 7. RAG CHAT - MULTIPLE QUERIES
Test: Ask multiple questions in same session
Steps:
  1. Open chat widget
  2. Ask: "What is Gazebo?"
  3. Wait for response
  4. Ask: "How do I use Isaac Sim?"
  5. Wait for response
Expected: Both responses appear, session maintained

### 8. RAG CHAT - SOURCES
Test: Verify sources are displayed correctly
Steps:
  1. Ask a question in chat
  2. Wait for response
  3. Verify "Sources:" section appears
  4. Click on source links
Expected: Sources displayed, links are clickable

### 9. RAG CHAT - RATE LIMITING
Test: Verify rate limiting works
Steps:
  1. Open chat widget
  2. Rapidly send 5+ queries
  3. Observe rate limit message
  4. Wait and retry
Expected: Rate limit message appears, can retry after wait

### 10. RAG CHAT - AUTHENTICATION
Test: Chat history persists for authenticated users
Steps:
  1. Sign in to application
  2. Open chat widget
  3. Ask a question
  4. Close widget
  5. Refresh page
  6. Open chat widget
Expected: Previous chat history appears

### 11. RAG CHAT - ANONYMOUS
Test: Anonymous users can chat without auth
Steps:
  1. Don't sign in
  2. Open chat widget
  3. Ask a question
  4. Verify response appears
Expected: Response appears without authentication

### 12. TRANSLATION + RAG FLOW
Test: Use translation and RAG together
Steps:
  1. Open chapter
  2. Click "Translate to Urdu"
  3. Open chat widget
  4. Ask question about translated content
  5. Verify response is relevant
Expected: Both features work together seamlessly

### 13. ERROR RECOVERY
Test: Recover from errors gracefully
Steps:
  1. Stop backend server
  2. Try to translate or chat
  3. Verify error message
  4. Start backend server
  5. Try again
Expected: Error message, then works after restart

### 14. PERFORMANCE
Test: Check performance and responsiveness
Steps:
  1. Open DevTools Network tab
  2. Translate content
  3. Check request time (should be < 5s)
  4. Ask RAG question
  5. Check request time (should be < 10s)
Expected: Reasonable response times

### 15. MOBILE RESPONSIVENESS
Test: Features work on mobile
Steps:
  1. Open DevTools mobile view
  2. Click translate button
  3. Verify translation works
  4. Open chat widget
  5. Ask question
Expected: All features work on mobile

## Console Checks

After each test, verify console has no errors:
- No red error messages
- No 404s for API calls
- No CORS errors
- No undefined references

## API Endpoints to Monitor

Translation:
- POST /api/translate/translate
- Expected: 200 OK, translated_text in response

RAG Chat:
- POST /api/rag/query
- Expected: 200 OK, answer and sources in response
- GET /api/rag/history
- Expected: 200 OK, chat history in response

Health:
- GET /health
- Expected: 200 OK, healthy status
- GET /api/rag/health
- Expected: 200 OK, RAG system status

## Debugging Tips

1. Check browser console for errors
2. Check backend logs: tail -f backend/logs/app.log
3. Check network requests in DevTools
4. Verify API_BASE_URL in frontend/src/constants/apiConfig.js
5. Verify .env has correct API keys
6. Check CORS settings in backend/src/api/main.py

## Success Criteria

All tests pass if:
✓ Translation works without errors
✓ Code blocks are preserved
✓ Technical terms are preserved
✓ RAG chat responds to queries
✓ Sources are displayed
✓ Rate limiting works
✓ Error messages are user-friendly
✓ No console errors
✓ Performance is acceptable
✓ Mobile works
"""
