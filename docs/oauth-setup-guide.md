# Setting Up Google and GitHub OAuth Authentication

This guide explains how to configure Google and GitHub OAuth authentication for the Physical AI & Humanoid Robotics learning platform.

## Prerequisites

- A Google account for Google Cloud Console
- A GitHub account for GitHub Developer Settings
- Access to your deployed application's environment variables

## Google OAuth Setup

### Step 1: Create Google OAuth Credentials

1. Go to the [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project or select an existing one
3. Navigate to "APIs & Services" > "Credentials"
4. Click "Create Credentials" > "OAuth 2.0 Client IDs"
5. For "Application type", select "Web application"
6. Add the following authorized redirect URIs:
   - For local development: `http://localhost:8000/api/auth/google/callback`
   - For production: `https://yourdomain.com/api/auth/google/callback` (replace with your actual domain)
7. Click "Create"
8. Note down the "Client ID" and "Client Secret"

### Step 2: Configure Environment Variables

Update your `.env` file with the Google OAuth credentials:

```bash
GOOGLE_CLIENT_ID=your_google_client_id_here
GOOGLE_CLIENT_SECRET=your_google_client_secret_here
GOOGLE_REDIRECT_URI=http://localhost:8000/api/auth/google/callback
```

For production, update the redirect URI to match your domain:
```bash
GOOGLE_REDIRECT_URI=https://yourdomain.com/api/auth/google/callback
```

## GitHub OAuth Setup

### Step 1: Create GitHub OAuth App

1. Go to GitHub and navigate to Settings > Developer settings > OAuth Apps
2. Click "New OAuth App"
3. Fill in the application details:
   - Application name: Choose a descriptive name
   - Homepage URL: Your application's homepage (e.g., `https://yourdomain.com`)
   - Authorization callback URL:
     - For local development: `http://localhost:8000/api/auth/github/callback`
     - For production: `https://yourdomain.com/api/auth/github/callback`
4. Click "Register Application"
5. Note down the "Client ID" and "Client Secret"

### Step 2: Configure Environment Variables

Update your `.env` file with the GitHub OAuth credentials:

```bash
GITHUB_CLIENT_ID=your_github_client_id_here
GITHUB_CLIENT_SECRET=your_github_client_secret_here
GITHUB_REDIRECT_URI=http://localhost:8000/api/auth/github/callback
```

For production, update the redirect URI to match your domain:
```bash
GITHUB_REDIRECT_URI=https://yourdomain.com/api/auth/github/callback
```

## Testing the Configuration

1. Restart your backend server after updating environment variables
2. Navigate to your application's sign-in or sign-up page
3. Click the "Google" or "GitHub" buttons to test the OAuth flow
4. You should be redirected to the respective OAuth provider for authentication
5. After successful authentication, you should be redirected back to your application

## Troubleshooting

### Common Issues:

1. **"OAuth endpoints not found"**: Ensure your backend server is running and accessible
2. **"OAuth is not properly configured"**: Verify that all required environment variables are set
3. **"Failed to get access token"**: Check that your redirect URIs match exactly what you configured in Google/GitHub
4. **"Failed to get user info"**: Verify that the required scopes are granted

### Debug Steps:

1. Check your server logs for detailed error messages
2. Verify that your redirect URIs match exactly between your code and OAuth provider configuration
3. Ensure that the OAuth provider allows requests from your application's domain
4. Confirm that your OAuth credentials have not expired or been revoked

## Security Best Practices

- Never commit OAuth credentials to version control
- Use environment variables for storing sensitive information
- Regularly rotate your OAuth credentials
- Use HTTPS in production environments
- Validate and sanitize all user data received from OAuth providers

## Environment Variables Reference

| Variable | Description | Example |
|----------|-------------|---------|
| `GOOGLE_CLIENT_ID` | Google OAuth client ID | `1234567890-abcdefghijklmnopqrstuvwxyz.apps.googleusercontent.com` |
| `GOOGLE_CLIENT_SECRET` | Google OAuth client secret | `AbCdEfGhIjKlMnOpQrStUvWxYz` |
| `GITHUB_CLIENT_ID` | GitHub OAuth client ID | `a1b2c3d4e5f6g7h8i9j0` |
| `GITHUB_CLIENT_SECRET` | GitHub OAuth client secret | `a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6q7r8s9t0` |
| `GOOGLE_REDIRECT_URI` | Google OAuth callback URL | `http://localhost:8000/api/auth/google/callback` |
| `GITHUB_REDIRECT_URI` | GitHub OAuth callback URL | `http://localhost:8000/api/auth/github/callback` |