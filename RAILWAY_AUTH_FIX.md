# Railway Authentication Fix Guide

## Problem
Authentication fails on Railway/Vercel with error: `connection to server at "localhost" (::1), port 5432 failed`

## Root Cause
The `DATABASE_URL` environment variable is not set on Railway, so the backend defaults to trying to connect to a local PostgreSQL server that doesn't exist.

## Solution

### Option 1: Use SQLite (Quickest - Recommended for Testing)

1. Go to your Railway dashboard: https://railway.app
2. Select your backend service
3. Go to **Variables** tab
4. Add this environment variable:
   ```
   DATABASE_URL=sqlite:///./book_agent_system.db
   ```
5. Click **Deploy** to redeploy with the new variable
6. Test authentication on your Vercel app

**Pros:** Works immediately, no setup needed
**Cons:** Data is stored in a file, not ideal for production with multiple instances

### Option 2: Use Railway's PostgreSQL Plugin (Better for Production)

1. Go to your Railway project dashboard
2. Click **+ New** button
3. Select **Database** → **PostgreSQL**
4. Railway will automatically:
   - Create a PostgreSQL database
   - Set the `DATABASE_URL` environment variable
5. Click **Deploy** to redeploy
6. Test authentication on your Vercel app

**Pros:** Production-ready, scalable, proper database
**Cons:** Requires Railway PostgreSQL plugin

## Verification

After deploying, test by:

1. Go to your Vercel app: https://your-app.vercel.app
2. Try to sign up or log in
3. Check Railway logs for any errors:
   - Railway Dashboard → Your Service → Logs tab
   - Look for "✅ Database connected" message

## What Changed in the Code

1. **Default DATABASE_URL** now uses SQLite instead of PostgreSQL
2. **Better error messages** - Auth endpoints now return 503 (Service Unavailable) with helpful message if database is down
3. **Improved logging** - Shows which database is being used and connection status

## Environment Variables Needed on Railway

Minimum required:
- `DATABASE_URL` - Set via Option 1 or 2 above

Optional but recommended:
- `SECRET_KEY` - For production security (currently using default)
- `GOOGLE_CLIENT_ID` / `GOOGLE_CLIENT_SECRET` - For Google OAuth
- `GITHUB_CLIENT_ID` / `GITHUB_CLIENT_SECRET` - For GitHub OAuth

## Testing Locally

To test the fix locally before deploying:

```bash
cd backend
source .venv/bin/activate  # or .venv\Scripts\activate on Windows
python -m uvicorn src.api.main:app --reload --port 8001
```

Then test auth endpoints:
```bash
# Sign up
curl -X POST http://localhost:8001/api/auth/register \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"test123","full_name":"Test User"}'

# Login
curl -X POST http://localhost:8001/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"test123"}'
```

## Still Having Issues?

1. Check Railway logs for the exact error
2. Verify `DATABASE_URL` is set in Railway Variables
3. Make sure the database is accessible (not in a private network)
4. Try Option 2 (PostgreSQL plugin) if Option 1 doesn't work
