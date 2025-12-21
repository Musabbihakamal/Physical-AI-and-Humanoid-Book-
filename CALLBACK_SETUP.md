# OAuth Callback Configuration

To complete the OAuth flow and have users redirected back to your application after authentication, you need to properly configure the callback endpoints.

## Backend Routes

The following callback endpoints have been implemented in your backend:

- **Google Callback:** `/api/auth/google/callback`
- **GitHub Callback:** `/api/auth/github/callback`

## Required Configuration

### Google OAuth Callback Setup

1. **In Google Cloud Console:**
   - Go to your OAuth 2.0 Client ID
   - Under "Authorized redirect URIs", add:
     - `http://localhost:8000/api/auth/google/callback` (for development)
     - `https://yourdomain.com/api/auth/google/callback` (for production)

2. **Environment Variables:**
   ```env
   GOOGLE_REDIRECT_URI=http://localhost:8000/api/auth/google/callback
   ```

### GitHub OAuth Callback Setup

1. **In GitHub OAuth App Settings:**
   - Go to your OAuth App configuration
   - Set "Authorization callback URL" to:
     - `http://localhost:8000/api/auth/github/callback` (for development)
     - `https://yourdomain.com/api/auth/github/callback` (for production)

2. **Environment Variables:**
   ```env
   GITHUB_REDIRECT_URI=http://localhost:8000/api/auth/github/callback
   ```

## Complete OAuth Flow

1. **User clicks "Sign in with Google/GitHub"**
2. **Redirect to OAuth provider** (Google/GitHub login page)
3. **User authenticates** with OAuth provider
4. **OAuth provider redirects back** to your callback endpoint with authorization code
5. **Backend exchanges code** for access token
6. **Backend creates/updates user** in database
7. **Backend returns authentication tokens** to frontend
8. **User is logged in** and redirected to appropriate page

## Testing the Complete Flow

After configuring the callback URLs:

1. **Click Google/GitHub button** → Should redirect to provider login
2. **Complete login** on provider site
3. **Should redirect back** to your application
4. **User should be logged in** automatically

## Troubleshooting Callback Issues

**Problem:** After login on Google/GitHub, getting "redirect_uri_mismatch" error
**Solution:** Verify the callback URL exactly matches what you configured in Google/GitHub

**Problem:** Getting "Page not found" after OAuth provider login
**Solution:** Ensure your backend server is running and the callback endpoints are accessible

**Problem:** User authenticates but doesn't get logged into your app
**Solution:** Check backend logs for errors in the callback processing

## Security Considerations

- Callback URLs must use HTTPS in production
- State parameters are used to prevent CSRF attacks
- Access tokens are properly validated before creating user sessions
- OAuth provider verification ensures authentic user identity