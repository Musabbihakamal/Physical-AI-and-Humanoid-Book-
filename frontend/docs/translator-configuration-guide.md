# Translator Agent Configuration Guide

## Overview

This guide provides step-by-step instructions for configuring the translator agent to enable Urdu and other language translations in the Physical AI and Humanoid Robotics documentation system.

## Prerequisites

Before configuring the translator agent, ensure you have:

1. An API key from one of the supported providers:
   - [OpenAI](https://platform.openai.com/) (recommended)
   - [Anthropic Claude](https://www.anthropic.com/)
   - Or another translation API provider

2. Node.js and npm installed and running the application

## Configuration Steps

### Step 1: Obtain Your API Key

#### For OpenAI (Recommended)
1. Go to [OpenAI Platform](https://platform.openai.com/)
2. Create an account or log in to your existing account
3. Navigate to "API Keys" in the dashboard
4. Click "Create new secret key"
5. Copy the generated API key (save it securely as it will only be shown once)

#### For Anthropic Claude
1. Go to [Anthropic](https://www.anthropic.com/)
2. Create an account or log in to your existing account
3. Navigate to the API section
4. Generate a new API key
5. Copy the API key for use

### Step 2: Update Environment Variables

1. Navigate to the `frontend` directory in your project
2. Open or create the `.env` file
3. Add or update the following configuration:

```
# Translation API Configuration
REACT_APP_TRANSLATOR_API_KEY=your-actual-api-key-here
REACT_APP_TRANSLATOR_API_URL=https://api.openai.com/v1/chat/completions
```

For Anthropic Claude, use:
```
REACT_APP_TRANSLATOR_API_KEY=your-actual-api-key-here
REACT_APP_TRANSLATOR_API_URL=https://api.anthropic.com/v1/messages
```

### Step 3: Restart the Application

After updating the environment variables:

1. Stop the development server (Ctrl+C)
2. Restart the frontend server:
   ```bash
   cd frontend
   npm start
   ```

## API Configuration Options

### OpenAI Configuration
```
REACT_APP_TRANSLATOR_API_KEY=sk-...your-openai-api-key-here
REACT_APP_TRANSLATOR_API_URL=https://api.openai.com/v1/chat/completions
```

### Anthropic Claude Configuration
```
REACT_APP_TRANSLATOR_API_KEY=your-claude-api-key-here
REACT_APP_TRANSLATOR_API_URL=https://api.anthropic.com/v1/messages
```

### Generic Translation API Configuration
```
REACT_APP_TRANSLATOR_API_KEY=your-api-key-here
REACT_APP_TRANSLATOR_API_URL=https://your-translation-api.com/translate
```

## Testing the Configuration

1. Navigate to any documentation page in the application
2. Look for the "Translate to Urdu" button (it should appear on chapter pages)
3. The button should go through these states:
   - "Initializing..." (briefly)
   - ".Translate to Urdu" (ready state)
4. Click the button to test translation functionality
5. The button should show "Translating..." during the process
6. Content should appear translated to Urdu while preserving code blocks and technical terms

## Troubleshooting

### Common Issues

1. **Button shows "Initializing..." indefinitely**
   - Check your internet connection
   - Verify API key is correct
   - Check browser console (F12) for specific error messages

2. **"API key not configured" error**
   - Verify `.env` file is in the `frontend` directory
   - Ensure environment variables are properly formatted
   - Restart the development server after changes

3. **"Invalid API key" or "401 Unauthorized" error**
   - Double-check your API key for typos
   - Ensure your API key has proper permissions
   - Verify you're using the correct API endpoint URL

4. **"Rate limit exceeded" error**
   - Wait a few minutes before trying again
   - Check your API provider's rate limits
   - Consider upgrading your API plan if needed

### Verification Checklist

- [ ] API key is properly formatted and copied
- [ ] Environment variables are in the correct `.env` file
- [ ] Application server has been restarted after changes
- [ ] API provider account is in good standing
- [ ] Sufficient credits/tokens are available on your account

## Security Best Practices

1. **Never commit the `.env` file** to version control
2. **Keep your API key secure** and don't share it publicly
3. **Use environment-specific keys** when possible
4. **Monitor your API usage** to avoid unexpected charges
5. **Regenerate API keys periodically** for security

## Advanced Configuration

### Custom API Endpoints

If using a custom translation service, ensure your API endpoint supports:

- JSON request/response format
- Proper authentication headers
- Translation of text content
- Preservation of formatting when possible

### Multiple API Providers

You can configure different API providers by changing the `REACT_APP_TRANSLATOR_API_URL` value:

- OpenAI: `https://api.openai.com/v1/chat/completions`
- Anthropic: `https://api.anthropic.com/v1/messages`
- Azure OpenAI: `https://your-resource.openai.azure.com/openai/deployments/your-deployment/chat/completions?api-version=2023-03-15-preview`

## Support

If you continue to experience issues:

1. Check the browser console for specific error messages
2. Verify your API provider's status page
3. Consult the API provider's documentation
4. Ensure your account has sufficient credits
5. Check the browser console (F12) for specific error messages

For additional help, create an issue in the repository with specific error messages and your configuration details.