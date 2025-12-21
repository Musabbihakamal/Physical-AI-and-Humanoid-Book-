# OAuth Mock Implementation for Development

If you don't have OAuth configured yet, you can use these mock implementations for development/testing purposes.

## Frontend Mock Implementation

To enable mock OAuth functionality for development without actual Google/GitHub setup:

1. Create a mock OAuth handler in your frontend:

```javascript
// In your sign-in and sign-up pages, replace the OAuth functions with this mock version:

const handleGoogleLogin = async () => {
  try {
    setIsLoading(true);
    setError('');

    // Mock OAuth implementation for development
    console.log('Mock Google OAuth login initiated');

    // Simulate OAuth flow with mock data
    setTimeout(() => {
      // Mock successful login response
      const mockUserData = {
        email: 'mockuser@gmail.com',
        name: 'Mock User',
        provider: 'google'
      };

      // In a real app, you would process the actual OAuth response
      // For now, show an alert indicating what would happen
      alert('In a real implementation, this would redirect to Google for authentication.\n\n' +
            'Google OAuth is not configured. Please see OAUTH_SETUP.md for configuration instructions.');

      setIsLoading(false);
    }, 1000);
  } catch (err) {
    setError(err.message || 'Google login failed. Please try again.');
    setIsLoading(false);
  }
};

const handleGitHubLogin = async () => {
  try {
    setIsLoading(true);
    setError('');

    // Mock OAuth implementation for development
    console.log('Mock GitHub OAuth login initiated');

    // Simulate OAuth flow with mock data
    setTimeout(() => {
      // Mock successful login response
      const mockUserData = {
        email: 'mockuser@github.com',
        name: 'Mock User',
        provider: 'github'
      };

      // In a real app, you would process the actual OAuth response
      // For now, show an alert indicating what would happen
      alert('In a real implementation, this would redirect to GitHub for authentication.\n\n' +
            'GitHub OAuth is not configured. Please see OAUTH_SETUP.md for configuration instructions.');

      setIsLoading(false);
    }, 1000);
  } catch (err) {
    setError(err.message || 'GitHub login failed. Please try again.');
    setIsLoading(false);
  }
};
```

## Backend Mock Implementation

If you want to add mock OAuth endpoints to your backend, you can temporarily add these routes:

```python
# Add these mock routes to your auth_routes.py for development purposes

@router.get("/google/mock")
async def google_login_mock():
    """
    Mock Google OAuth endpoint for development without actual Google setup.
    """
    # Return a mock auth URL that simulates Google OAuth
    return {
        "auth_url": "/api/auth/google/callback?code=mock_code&state=mock_state",
        "message": "This is a mock Google OAuth endpoint. In production, this would redirect to Google."
    }


@router.get("/github/mock")
async def github_login_mock():
    """
    Mock GitHub OAuth endpoint for development without actual GitHub setup.
    """
    # Return a mock auth URL that simulates GitHub OAuth
    return {
        "auth_url": "/api/auth/github/callback?code=mock_code&state=mock_state",
        "message": "This is a mock GitHub OAuth endpoint. In production, this would redirect to GitHub."
    }
```

## Development Workflow

1. Use the mock implementation during development
2. Once you have OAuth configured, replace the mock functions with the real ones
3. The real OAuth functionality will then work seamlessly

## Testing OAuth Locally

To properly test OAuth functionality locally, you need:

1. Valid OAuth credentials from Google and GitHub
2. Correctly configured redirect URIs
3. A running backend server that can handle the OAuth callbacks

Without these, the OAuth buttons will show appropriate error messages to guide the user to the setup documentation.