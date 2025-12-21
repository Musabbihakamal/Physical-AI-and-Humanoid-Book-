# OAuth Setup Instructions

## Prerequisites

Before the Google and GitHub OAuth buttons will work, you need to:

1. **Start the backend server** on port 8000:
   ```bash
   cd backend
   python -m src.main
   ```

2. **Set up environment variables** for OAuth (in `backend/.env`):
   ```env
   # Google OAuth Configuration
   GOOGLE_CLIENT_ID=your_google_client_id
   GOOGLE_CLIENT_SECRET=your_google_client_secret
   GOOGLE_REDIRECT_URI=http://localhost:8000/api/auth/google/callback

   # GitHub OAuth Configuration
   GITHUB_CLIENT_ID=your_github_client_id
   GITHUB_CLIENT_SECRET=your_github_client_secret
   GITHUB_REDIRECT_URI=http://localhost:8000/api/auth/github/callback
   ```

## Backend Server Requirements

The OAuth functionality requires:
- Backend server running on port 8000 (default)
- Proper OAuth credentials configured in environment variables
- Internet connectivity to communicate with Google/GitHub APIs

## Troubleshooting Common Issues

### Issue: "Not Found" error when clicking OAuth buttons
**Cause:** Backend server is not running
**Solution:** Start the backend server with `python -m src.main`

### Issue: "Cannot connect to backend server"
**Cause:** Frontend cannot reach the backend
**Solution:** Ensure backend is running on port 8000 and check firewall settings

### Issue: "OAuth endpoints not found"
**Cause:** OAuth routes are not properly configured in the backend
**Solution:** Verify backend is running and OAuth functionality is enabled

### Issue: "Backend server is not accessible"
**Cause:** Network connectivity issues between frontend and backend
**Solution:** Check that both servers are running and can communicate

## Development Setup

For development, the frontend assumes the backend is running on `http://localhost:8000`. If you're running the backend on a different port, you may need to adjust the configuration.

## Production Setup

In production, ensure that:
- OAuth redirect URIs are configured correctly in Google/GitHub developer consoles
- Backend server is accessible from the frontend domain
- HTTPS is used for production environments
- Environment variables are properly set in the production environment

## Testing OAuth Locally

1. Make sure the backend server is running: `cd backend && python -m src.main`
2. In a separate terminal, start the frontend: `cd frontend && npm start`
3. Navigate to the sign-in or sign-up page
4. Click the Google or GitHub button
5. You should be redirected to the respective OAuth provider's login page

If you don't have OAuth credentials configured, you'll receive a clear error message directing you to set up the OAuth environment variables.