# OAuth Setup Checklist

Follow this checklist to properly configure Google and GitHub OAuth for your application.

## Pre-requisites

- [ ] Backend server running and accessible
- [ ] Frontend application running and can communicate with backend
- [ ] Access to environment configuration files

## Step 1: Configure Backend Environment Variables

1. **Open your backend `.env` file**
2. **Add Google OAuth variables:**
   ```env
   GOOGLE_CLIENT_ID=your_actual_google_client_id
   GOOGLE_CLIENT_SECRET=your_actual_google_client_secret
   GOOGLE_REDIRECT_URI=http://localhost:8000/api/auth/google/callback
   ```

3. **Add GitHub OAuth variables:**
   ```env
   GITHUB_CLIENT_ID=your_actual_github_client_id
   GITHUB_CLIENT_SECRET=your_actual_github_client_secret
   GITHUB_REDIRECT_URI=http://localhost:8000/api/auth/github/callback
   ```

## Step 2: Configure Frontend Environment Variables

1. **Open your frontend `.env` file**
2. **Add backend URL:**
   ```env
   REACT_APP_BACKEND_URL=http://localhost:8000
   ```

## Step 3: Register Applications with OAuth Providers

### Google OAuth Setup
- [ ] Go to [Google Cloud Console](https://console.cloud.google.com/)
- [ ] Create a new project or select existing one
- [ ] Enable the Google People API
- [ ] Navigate to Credentials > Create Credentials > OAuth 2.0 Client IDs
- [ ] Set application type to "Web application"
- [ ] Add authorized redirect URIs:
  - `http://localhost:8000/api/auth/google/callback` (for development)
  - Your production URL (for production)
- [ ] Copy the Client ID and Client Secret to your backend `.env` file

### GitHub OAuth Setup
- [ ] Go to GitHub Settings > Developer settings > OAuth Apps
- [ ] Click "New OAuth App"
- [ ] Fill in application details
- [ ] Set Authorization callback URL to:
  - `http://localhost:8000/api/auth/github/callback` (for development)
  - Your production URL (for production)
- [ ] Copy the Client ID and Client Secret to your backend `.env` file

## Step 4: Restart Services

- [ ] Restart your backend server
- [ ] Restart your frontend development server
- [ ] Clear browser cache and cookies if needed

## Step 5: Test the Configuration

1. **Visit your sign-in page**
2. **Click the Google button**
   - [ ] You should be redirected to Google's login page
   - [ ] After login, you should be redirected back to your application
   - [ ] You should be logged in successfully

3. **Click the GitHub button**
   - [ ] You should be redirected to GitHub's login page
   - [ ] After login, you should be redirected back to your application
   - [ ] You should be logged in successfully

## Troubleshooting Checklist

If OAuth is not working:

- [ ] Verify all environment variables are set correctly
- [ ] Check that redirect URIs match exactly what's configured in Google/GitHub
- [ ] Verify backend server is running and accessible
- [ ] Check browser console for JavaScript errors
- [ ] Check backend server logs for error messages
- [ ] Verify CORS settings allow communication between frontend and backend
- [ ] Ensure OAuth provider apps are set to "enabled" status

## Common Error Messages

- `"redirect_uri_mismatch"` - Check your redirect URI configuration
- `"access_denied"` - Check your OAuth scopes and permissions
- `"OAuth is not configured"` - Missing environment variables
- `"Failed to initiate login"` - Network or server communication issue

## Production Deployment

For production, ensure you update:
- [ ] Redirect URIs to use your production domain
- [ ] Backend URL in frontend environment variables
- [ ] OAuth provider settings to allow your production domain
- [ ] SSL certificates for HTTPS communication