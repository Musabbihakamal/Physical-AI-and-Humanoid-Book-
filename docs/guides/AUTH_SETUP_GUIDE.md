# Complete Authentication & RAG Chat History Setup Guide

## Overview

This guide will help you set up and test the complete authentication system with RAG chat history persistence for both Google and GitHub OAuth.

## Features Implemented

✅ **Authentication System:**
- Email/password registration and login
- Google OAuth integration
- GitHub OAuth integration
- JWT token-based authentication
- Refresh token support

✅ **RAG Chat History:**
- Automatic chat history saving for authenticated users
- Chat history loading on widget open
- Session continuity across page refreshes
- Delete chat history functionality
- Anonymous users can chat (no history saved)

✅ **User Experience:**
- Visual indicator when user is logged in
- Personalized welcome message
- Auth prompt for anonymous users
- User badge in chat header

---

## Step 1: Database Setup

The database tables are automatically created when you start the backend. Verify they exist:

```bash
# Check if tables are created
cd "D:\PROJECT\HACKATHON 1\backend"
python -c "from src.database.database import engine, Base; from src.models import user, user_profile, rag_session, token; Base.metadata.create_all(bind=engine); print('✓ Database tables created')"
```

**Required tables:**
- `users` - User accounts
- `user_profiles` - User preferences and settings
- `tokens` - Refresh tokens
- `rag_sessions` - Chat sessions
- `rag_queries` - Individual chat messages

---

## Step 2: Configure OAuth (Google & GitHub)

### Google OAuth Setup

1. **Go to Google Cloud Console**: https://console.cloud.google.com/
2. **Create a new project** or select existing
3. **Enable Google+ API**:
   - Go to "APIs & Services" → "Library"
   - Search for "Google+ API"
   - Click "Enable"
4. **Create OAuth credentials**:
   - Go to "APIs & Services" → "Credentials"
   - Click "Create Credentials" → "OAuth client ID"
   - Application type: "Web application"
   - Authorized redirect URIs:
     - `http://localhost:8000/api/auth/google/callback` (development)
     - `https://your-domain.com/api/auth/google/callback` (production)
5. **Copy credentials** and add to `.env`:

```bash
GOOGLE_CLIENT_ID=your_google_client_id_here
GOOGLE_CLIENT_SECRET=your_google_client_secret_here
GOOGLE_REDIRECT_URI=http://localhost:8000/api/auth/google/callback
```

### GitHub OAuth Setup

1. **Go to GitHub Settings**: https://github.com/settings/developers
2. **Click "New OAuth App"**
3. **Fill in details**:
   - Application name: "Physical AI Book"
   - Homepage URL: `http://localhost:3000`
   - Authorization callback URL: `http://localhost:8000/api/auth/github/callback`
4. **Copy credentials** and add to `.env`:

```bash
GITHUB_CLIENT_ID=your_github_client_id_here
GITHUB_CLIENT_SECRET=your_github_client_secret_here
GITHUB_REDIRECT_URI=http://localhost:8000/api/auth/github/callback
```

### Update Backend .env File

```bash
# Edit backend/.env
cd "D:\PROJECT\HACKATHON 1\backend"
notepad .env
```

Add these lines (replace with your actual credentials):

```env
# OAuth Configuration
GOOGLE_CLIENT_ID=your_actual_google_client_id
GOOGLE_CLIENT_SECRET=your_actual_google_client_secret
GITHUB_CLIENT_ID=your_actual_github_client_id
GITHUB_CLIENT_SECRET=your_actual_github_client_secret
GOOGLE_REDIRECT_URI=http://localhost:8000/api/auth/google/callback
GITHUB_REDIRECT_URI=http://localhost:8000/api/auth/github/callback

# Database (already configured)
DATABASE_URL=sqlite:///../data/book_agent_system.db

# Security
SECRET_KEY=your-secret-key-change-this-in-production
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30

# RAG Configuration (already configured)
COHERE_API_KEY=your_cohere_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_key
```

---

## Step 3: Start the Backend

```powershell
# Terminal 1 - Backend
cd "D:\PROJECT\HACKATHON 1"
python -m backend.src.main
```

**Expected output:**
```
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Creating database tables...
INFO:     Database tables created successfully
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

**Verify backend is running:**
```powershell
# Test health endpoint
curl http://localhost:8000/health

# Test RAG health
curl http://localhost:8000/api/rag/health
```

---

## Step 4: Start the Frontend

```powershell
# Terminal 2 - Frontend
cd "D:\PROJECT\HACKATHON 1"
npm run frontend
```

**Expected output:**
```
Compiled successfully!
You can now view the app in the browser.
  Local:            http://localhost:3000
```

---

## Step 5: Test Authentication Flow

### Test 1: Email/Password Registration

1. **Open browser**: http://localhost:3000
2. **Navigate to registration page** (if you have one)
3. **Or use API directly**:

```powershell
# Register a new user
curl -X POST http://localhost:8000/api/auth/register `
  -H "Content-Type: application/json" `
  -d '{
    "email": "test@example.com",
    "password": "SecurePass123!",
    "full_name": "Test User",
    "experience_level": "BEGINNER",
    "language_preference": "en"
  }'
```

**Expected response:**
```json
{
  "access_token": "eyJ0eXAiOiJKV1QiLCJhbGc...",
  "refresh_token": "eyJ0eXAiOiJKV1QiLCJhbGc...",
  "token_type": "bearer",
  "user_id": "uuid-here",
  "email": "test@example.com",
  "full_name": "Test User"
}
```

### Test 2: Email/Password Login

```powershell
curl -X POST http://localhost:8000/api/auth/login `
  -H "Content-Type: application/json" `
  -d '{
    "email": "test@example.com",
    "password": "SecurePass123!"
  }'
```

### Test 3: Google OAuth

1. **Open browser**: http://localhost:8000/api/auth/google
2. **You should be redirected to Google login**
3. **After login, you'll be redirected back** with tokens in URL
4. **Frontend should capture tokens** and store them

### Test 4: GitHub OAuth

1. **Open browser**: http://localhost:8000/api/auth/github
2. **You should be redirected to GitHub login**
3. **After login, you'll be redirected back** with tokens in URL
4. **Frontend should capture tokens** and store them

---

## Step 6: Test RAG Chat with Authentication

### Test Anonymous Chat (No History)

1. **Open**: http://localhost:3000
2. **Click chat button** (bottom-right)
3. **Ask a question**: "What is ROS 2?"
4. **Verify**: You get an answer
5. **Refresh page**: Chat history is lost (expected for anonymous users)

### Test Authenticated Chat (With History)

1. **Login** using any method (email/Google/GitHub)
2. **Click chat button**
3. **Ask questions**:
   - "What is ROS 2?"
   - "Explain Gazebo simulation"
   - "How does Isaac Sim work?"
4. **Verify**: 
   - User badge shows in chat header
   - Green indicator on chat button
5. **Refresh page**
6. **Open chat again**
7. **Verify**: Chat history is loaded automatically!

### Test Chat History API

```powershell
# Get chat history (replace TOKEN with your access token)
curl http://localhost:8000/api/rag/history `
  -H "Authorization: Bearer YOUR_ACCESS_TOKEN_HERE"
```

**Expected response:**
```json
{
  "session_id": "uuid-here",
  "history": [
    {
      "id": "query-id",
      "query": "What is ROS 2?",
      "response": "ROS 2 is...",
      "sources": [...],
      "created_at": "2026-04-12T..."
    }
  ],
  "total_queries": 3
}
```

### Test Delete Chat History

```powershell
# Delete chat history (replace SESSION_ID and TOKEN)
curl -X DELETE http://localhost:8000/api/rag/history/SESSION_ID `
  -H "Authorization: Bearer YOUR_ACCESS_TOKEN_HERE"
```

---

## Step 7: Verify Database

```powershell
# Check if chat history is being saved
cd "D:\PROJECT\HACKATHON 1\backend"
python -c "from src.database.database import SessionLocal; from src.models.rag_session import RAGSession, RAGQuery; db = SessionLocal(); sessions = db.query(RAGSession).count(); queries = db.query(RAGQuery).count(); print(f'Sessions: {sessions}, Queries: {queries}'); db.close()"
```

---

## Troubleshooting

### Issue 1: OAuth Not Working

**Symptoms**: OAuth redirect fails or shows "not configured"

**Solution**:
1. Check `.env` file has correct credentials
2. Verify redirect URIs match exactly in OAuth provider settings
3. Restart backend after changing `.env`

### Issue 2: Chat History Not Saving

**Symptoms**: Chat history disappears after refresh

**Solution**:
1. Verify user is logged in (check for user badge in chat header)
2. Check browser console for auth errors
3. Verify token is being sent in requests (Network tab → Headers)

### Issue 3: "Authorization header missing"

**Symptoms**: API returns 401 Unauthorized

**Solution**:
1. Check if `AuthContext` is properly wrapping the app
2. Verify token is stored in localStorage/context
3. Check if token is expired (login again)

### Issue 4: Database Errors

**Symptoms**: "Table doesn't exist" or similar

**Solution**:
```powershell
# Recreate database tables
cd "D:\PROJECT\HACKATHON 1\backend"
python -c "from src.database.database import engine, Base; from src.models import user, user_profile, rag_session, token; Base.metadata.drop_all(bind=engine); Base.metadata.create_all(bind=engine); print('✓ Database reset complete')"
```

---

## Testing Checklist

- [ ] Backend starts without errors
- [ ] Frontend starts without errors
- [ ] Can register new user with email/password
- [ ] Can login with email/password
- [ ] Google OAuth redirects correctly
- [ ] GitHub OAuth redirects correctly
- [ ] Anonymous users can chat (no history)
- [ ] Authenticated users can chat (with history)
- [ ] Chat history persists after page refresh
- [ ] Can delete chat history
- [ ] User badge shows when logged in
- [ ] Green indicator on chat button when logged in
- [ ] Auth prompt shows for anonymous users after 2+ messages

---

## Next Steps

1. **Deploy to production**: Update OAuth redirect URIs
2. **Add password reset**: Implement forgot password flow
3. **Add email verification**: Send verification emails
4. **Add user dashboard**: Show all chat sessions
5. **Add export chat**: Allow users to export their chat history

---

## API Endpoints Reference

### Authentication
- `POST /api/auth/register` - Register new user
- `POST /api/auth/login` - Login with email/password
- `POST /api/auth/oauth-login` - OAuth login (Google/GitHub)
- `GET /api/auth/google` - Initiate Google OAuth
- `GET /api/auth/github` - Initiate GitHub OAuth
- `GET /api/auth/google/callback` - Google OAuth callback
- `GET /api/auth/github/callback` - GitHub OAuth callback
- `POST /api/auth/refresh` - Refresh access token
- `POST /api/auth/logout` - Logout user
- `GET /api/auth/me` - Get current user profile
- `PUT /api/auth/me` - Update user profile

### RAG Chat
- `POST /api/rag/query` - Send chat message (auth optional)
- `GET /api/rag/history` - Get chat history (auth required)
- `DELETE /api/rag/history/{session_id}` - Delete chat session (auth required)
- `GET /api/rag/health` - Check RAG system health

---

## Security Notes

⚠️ **Important for Production:**

1. **Change SECRET_KEY** in `.env` to a strong random value
2. **Use HTTPS** for all OAuth redirects
3. **Set CORS origins** to specific domains (not `["*"]`)
4. **Enable rate limiting** on auth endpoints
5. **Add email verification** before allowing login
6. **Implement password strength requirements**
7. **Add CAPTCHA** to prevent bot registrations
8. **Monitor failed login attempts**
9. **Implement account lockout** after failed attempts
10. **Use environment-specific configs** (dev/staging/prod)

---

## Support

If you encounter issues:
1. Check the troubleshooting section above
2. Review backend logs for errors
3. Check browser console for frontend errors
4. Verify all environment variables are set correctly
5. Ensure database tables are created

**Backend logs location**: Console output where you ran `python -m backend.src.main`
**Frontend logs location**: Browser Developer Tools → Console
