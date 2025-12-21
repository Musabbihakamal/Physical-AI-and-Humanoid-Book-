# Translation Component Setup Guide

## Overview
The Physical AI and Humanoid Robotics documentation includes a translation component that allows users to translate content to Urdu and other languages. This guide explains how to properly configure the translation functionality.

## Required Environment Variables

To enable the translation feature, you need to set up the following environment variables in your `.env` file:

```env
# Translation API Configuration
REACT_APP_TRANSLATOR_API_KEY=your_translator_api_key_here
REACT_APP_TRANSLATOR_API_URL=https://api.openai.com/v1/chat/completions
```

## Setting Up Translation API

### Option 1: OpenAI API (Recommended)
1. Create an account at [OpenAI](https://platform.openai.com/)
2. Generate an API key in your OpenAI dashboard
3. Add the API key to your `.env` file:

```env
REACT_APP_TRANSLATOR_API_KEY=sk-...your-openai-api-key-here
REACT_APP_TRANSLATOR_API_URL=https://api.openai.com/v1/chat/completions
```

### Option 2: Anthropic Claude API
1. Create an account at [Anthropic](https://www.anthropic.com/)
2. Generate an API key
3. Add the API key to your `.env` file:

```env
REACT_APP_TRANSLATOR_API_KEY=your-claude-api-key-here
REACT_APP_TRANSLATOR_API_URL=https://api.anthropic.com/v1/messages
```

### Option 3: Generic Translation API
If you have a different translation API, you can configure it with:

```env
REACT_APP_TRANSLATOR_API_KEY=your-api-key-here
REACT_APP_TRANSLATOR_API_URL=https://your-translation-api.com/translate
```

## Creating the .env File

1. In your `frontend` directory, create a file named `.env`
2. Add your API configuration to this file:

```env
REACT_APP_TRANSLATOR_API_KEY=your-api-key-here
REACT_APP_TRANSLATOR_API_URL=https://api.openai.com/v1/chat/completions
```

**Important:** Do not commit the `.env` file to version control. It's already included in the `.gitignore` file.

## How the Translation Works

1. The translation component appears on every chapter page across all 4 modules
2. When a user clicks the "Translate to Urdu" button, the content is sent to the configured translation API
3. The system preserves code blocks, technical terms, and formatting during translation
4. Constitutional compliance checks ensure the translated content meets ethical standards
5. Users can switch back to English at any time

## Supported Languages

The system currently supports translation to:
- Urdu (ur)
- Hindi (hi)
- Bengali (bn)
- Arabic (ar)
- Pashto (ps)

## Troubleshooting

### Translation Button Not Appearing
- Ensure you're on a documentation page (not the homepage)
- Check that the DocItem theme is properly loaded

### Translation Fails
- Verify your API key is correct
- Check your API URL is properly formatted
- Ensure your API provider supports the target language
- Check browser console for specific error messages

### Content Not Preserved Properly
- Code blocks and technical terms are automatically preserved
- If formatting is lost, check if your API supports HTML format

For more detailed troubleshooting information, see the [Troubleshooting Translation Guide](./troubleshooting-translation.md).

## Development

To run the application with translation enabled:

```bash
# In the frontend directory
npm install
npm run build
npm start
```

Make sure your environment variables are set before starting the development server.