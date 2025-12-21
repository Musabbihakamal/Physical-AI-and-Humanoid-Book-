# OAuth Setup Guide

This document explains how to configure Google and GitHub OAuth for the application.

## Environment Variables

You need to set up the following environment variables in your `.env` file:

### Backend Configuration

Add these to your backend `.env` file:

```env
# Google OAuth Configuration
GOOGLE_CLIENT_ID=your_google_client_id_here
GOOGLE_CLIENT_SECRET=your_google_client_secret_here
GOOGLE_REDIRECT_URI=http://localhost:8000/api/auth/google/callback

# GitHub OAuth Configuration
GITHUB_CLIENT_ID=your_github_client_id_here
GITHUB_CLIENT_SECRET=your_github_client_secret_here
GITHUB_REDIRECT_URI=http://localhost:8000/api/auth/github/callback
```

### Frontend Configuration

Add this to your frontend `.env` file:

```env
REACT_APP_BACKEND_URL=http://localhost:8000
```

## Setting up Google OAuth

1. Go to the [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project or select an existing one
3. Enable the Google+ API (or Google People API)
4. Go to "Credentials" and create an OAuth 2.0 Client ID
5. Set the authorized redirect URIs to:
   - `http://localhost:8000/api/auth/google/callback` (for development)
   - `https://yourdomain.com/api/auth/google/callback` (for production)
6. Download the credentials and add the Client ID and Secret to your `.env` file

## Setting up GitHub OAuth

1. Go to GitHub Settings → Developer settings → OAuth Apps
2. Click "New OAuth App"
3. Set the application name and homepage URL
4. Set the Authorization callback URL to:
   - `http://localhost:8000/api/auth/github/callback` (for development)
   - `https://yourdomain.com/api/auth/github/callback` (for production)
5. Add the Client ID and Secret to your `.env` file

## Testing the Configuration

After setting up the environment variables:

1. Restart your backend server
2. Visit the sign-in or sign-up page
3. Click the Google or GitHub buttons
4. You should be redirected to the respective OAuth provider's login page

## Troubleshooting

If the OAuth buttons still don't work:

1. **Check environment variables**: Make sure all required environment variables are set
2. **Verify URLs**: Ensure your redirect URLs match exactly what you configured in Google/GitHub
3. **Check backend logs**: Look for error messages in your backend server logs
4. **Browser console**: Check for JavaScript errors in the browser console
5. **Network tab**: Check if the API calls to `/api/auth/google` and `/api/auth/github` are working

## Development vs Production

For production, remember to:
- Change the redirect URLs to your production domain
- Update the `REACT_APP_BACKEND_URL` to your production backend URL
- Ensure your OAuth providers allow redirects to your production domain

## Common Issues

- **"redirect_uri_mismatch"**: This means the redirect URI configured in Google/GitHub doesn't match what your app is requesting
- **"access_denied"**: Usually caused by incorrect client credentials or scopes
- **CORS errors**: Make sure your backend allows requests from your frontend domain