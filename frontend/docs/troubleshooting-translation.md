# Troubleshooting Translation Issues

This guide helps you resolve common issues with the translation feature in the Physical AI documentation.

## Common Translation Errors and Solutions

### 1. "Failed to initialize translator. Please check console for details." Error

**Problem:** The translation button shows "Failed to initialize translator" message.

**Solutions:**
1. **Check the browser console** (F12) for specific error details
2. **Verify dependencies** are installed: Run `npm install` in both root and frontend directories
3. **Check file paths** - ensure `frontend/src/agents/translator-agent.js` exists
4. **Verify axios is available** - check that axios is installed in your project
5. **Check for syntax errors** in the translator agent file
6. **Restart your development server** after making changes
7. **The system now includes a fallback translator** that will load if the main one fails, providing limited functionality with clear guidance

### 2. "API key not configured" Error

**Problem:** The translation button shows "API key not configured. Please set REACT_APP_TRANSLATOR_API_KEY in your environment variables."

**Solution:**
1. Create a `.env` file in the `frontend` directory
2. Add your API key configuration:

```env
REACT_APP_TRANSLATOR_API_KEY=your-api-key-here
REACT_APP_TRANSLATOR_API_URL=https://api.openai.com/v1/chat/completions
```

3. Restart your development server after creating the `.env` file

### 3. "Translator not ready" Error

**Problem:** The button shows "Initializing..." and doesn't become active.

**Solution:**
- Wait a few seconds for the translator to initialize (the system now has improved initialization with better error handling)
- Check that your environment variables are properly set
- Verify that you have internet connectivity

### 4. "Translation failed. Please check your API key is correct" Error

**Problem:** Translation fails with a 401 (unauthorized) error.

**Solution:**
1. Verify your API key is correct
2. Check that your API key has the necessary permissions
3. Make sure you're using the correct API endpoint URL

### 5. "Rate limit exceeded. Please try again later" Error

**Problem:** Translation fails with a 429 error.

**Solution:**
- Wait a few minutes before trying again
- Check your API provider's rate limits
- Consider upgrading your API plan if you frequently hit rate limits

### 6. Network Errors

**Problem:** Translation fails with network-related errors.

**Solution:**
- Check your internet connection
- Verify that your firewall or network isn't blocking API requests
- Try using a different network if possible

### 7. Content Extraction Issues

**Problem:** Translation fails or content doesn't appear properly translated.

**Solutions:**
- The system now handles complex HTML content including code blocks, technical terms, and formatting
- Content extraction has been improved with multiple fallback selectors
- If you have very large documents (>10,000 characters), try translating smaller sections
- The system preserves code blocks (```...```, `<code>...</code>`, `<pre>...</pre>`) and technical terms during translation

### 8. Timing Issues

**Problem:** Content appears blank or doesn't update after translation.

**Solution:**
- The system now has improved timing mechanisms to ensure content is ready before translation
- Multiple attempts are made to extract content with exponential backoff
- The system uses multiple CSS selectors to find the correct content area

## Dependency and Setup Verification

Before using the translation feature, ensure all dependencies are properly installed:

1. **Install dependencies in root directory:**
   ```bash
   npm install
   ```

2. **Install dependencies in frontend directory:**
   ```bash
   cd frontend
   npm install
   ```

3. **Verify axios is installed:**
   Check that `axios` appears in both `package.json` files (root and frontend)

4. **Verify the translator agent file exists:**
   Ensure `frontend/src/agents/translator-agent.js` exists and is properly formatted

## API Configuration Options

### OpenAI API Setup
```env
REACT_APP_TRANSLATOR_API_KEY=sk-...your-openai-api-key-here
REACT_APP_TRANSLATOR_API_URL=https://api.openai.com/v1/chat/completions
```

### Anthropic Claude API Setup
```env
REACT_APP_TRANSLATOR_API_KEY=your-claude-api-key-here
REACT_APP_TRANSLATOR_API_URL=https://api.anthropic.com/v1/messages
```

### Generic Translation API Setup
```env
REACT_APP_TRANSLATOR_API_KEY=your-api-key-here
REACT_APP_TRANSLATOR_API_URL=https://your-translation-api.com/translate
```

## Debugging Steps

1. **Check Browser Console:** Open browser developer tools (F12) and look for error messages in the Console tab
2. **Verify Environment Variables:** Ensure your `.env` file is properly configured and in the right location
3. **Test API Endpoint:** You can test your API configuration with a simple API call outside the application
4. **Check Network Tab:** In browser developer tools, check the Network tab for failed API requests

## Environment File Setup

Make sure your `.env` file is in the `frontend` directory and contains:

```env
# Translation API Configuration
REACT_APP_TRANSLATOR_API_KEY=your-api-key-here
REACT_APP_TRANSLATOR_API_URL=https://api.openai.com/v1/chat/completions

# For OpenAI, you can also use:
# REACT_APP_TRANSLATOR_API_URL=https://api.openai.com/v1/chat/completions

# For Anthropic Claude, you can use:
# REACT_APP_TRANSLATOR_API_URL=https://api.anthropic.com/v1/messages
```

## Restart After Changes

After making changes to environment variables:
1. Stop the development server (Ctrl+C)
2. Delete the build cache: `npm run build` (if needed)
3. Restart the development server: `npm start`

## Testing the Translation Feature

1. Navigate to any documentation chapter
2. Wait for the "Initializing..." message to disappear
3. Click the "Translate to Urdu" button
4. Wait for the "Translating..." message to complete
5. The content should appear in Urdu while preserving code blocks and technical terms

## Known Issues

- Very large chapters may exceed API token limits
- Complex HTML formatting may not translate perfectly
- Some technical terms may not translate as expected
- The translation process may take several seconds for large documents

## Support

If you continue to experience issues after following this guide:

1. Check that your API key has sufficient credits
2. Verify your API provider's status page for any ongoing issues
3. Consult the API provider's documentation for specific requirements
4. Ensure your account is in good standing with the API provider