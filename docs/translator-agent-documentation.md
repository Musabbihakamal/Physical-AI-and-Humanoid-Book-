# Urdu Translator Agent Documentation

## Overview
The Urdu Translator Agent is a specialized AI agent designed to provide accurate translation services with a focus on Urdu language translation while adhering to constitutional principles and guidelines.

## Features
- Urdu translation with constitutional compliance
- Cultural sensitivity and context awareness
- Content filtering for constitutional adherence
- Multiple context types (legal, educational, religious, technical)
- Batch translation capabilities

## Installation

### Prerequisites
- Node.js 14+
- Access to translation API service

### Setup
1. Install dependencies:
```bash
npm install axios
```

2. Set up environment variables:
```bash
TRANSLATOR_API_KEY=your_api_key_here
```

## Usage

### Basic Translation
```javascript
import UrduTranslatorAgent from './src/agents/translator-agent.js';

const translator = new UrduTranslatorAgent({
  apiUrl: 'https://api.example.com/translate',
  apiKey: process.env.TRANSLATOR_API_KEY
});

// Translate text to Urdu
const result = await translator.translate('Hello, how are you?', 'ur');
console.log(result.translatedText);
```

### Translation with Context
```javascript
// Translate with legal context
const legalResult = await translator.translateWithContext(
  'Constitutional rights',
  'ur',
  { type: 'legal' }
);
console.log(legalResult.translatedText);
```

### Batch Translation
```javascript
const texts = ['Hello', 'How are you?', 'Goodbye'];
const batchResults = await translator.batchTranslate(texts, 'ur');
console.log(batchResults);
```

## Constitutional Compliance

The agent ensures all translations adhere to constitutional principles:

- **Respect for Diversity**: Translations maintain respect for diverse perspectives
- **Non-Discrimination**: Filters out discriminatory content
- **Truthfulness**: Maintains factual accuracy
- **Ethical Standards**: Adheres to ethical translation practices

## Context Types

### Legal Context
- Preserves legal terminology
- Maintains formal tone
- Includes constitutional references

### Educational Context
- Applies cultural sensitivity
- Ensures age-appropriate content
- Focuses on educational value

### Religious Context
- Shows cultural respect
- Properly handles sacred terms
- Preserves contextual meaning

### Technical Context
- Maintains terminology consistency
- Ensures formal accuracy
- Preserves technical jargon

## Configuration

The agent can be configured using the translator-config.json file or through constructor options.

## Error Handling

The agent implements comprehensive error handling:
- Validates input text
- Checks constitutional compliance
- Handles API errors gracefully
- Provides detailed error messages

## Security

- Content filtering to prevent discriminatory translations
- Constitutional compliance checks
- Secure API key handling
- Audit trail logging

## Performance

- Efficient batch processing
- Configurable rate limiting
- Timeout handling
- Retry mechanisms

## API Reference

### Methods

#### `translate(text, targetLang, sourceLang)`
Translates text between languages with constitutional compliance.

Parameters:
- `text`: Text to translate
- `targetLang`: Target language code (e.g., 'ur', 'en')
- `sourceLang`: Source language code (optional)

Returns: Promise<Object> with translation result

#### `batchTranslate(texts, targetLang, sourceLang)`
Translates multiple texts in batch.

#### `detectLanguage(text)`
Detects the language of provided text.

#### `getSupportedLanguages()`
Returns supported language codes.

#### `translateWithContext(text, targetLang, context)`
Translates text with context preservation.

## Examples

### Legal Translation
```javascript
const legalText = 'Fundamental rights of citizens';
const result = await translator.translateWithContext(legalText, 'ur', { type: 'legal' });
```

### Educational Translation
```javascript
const educationText = 'Importance of education in society';
const result = await translator.translateWithContext(educationText, 'ur', { type: 'educational' });
```

## Troubleshooting

- Ensure API key is properly configured
- Verify internet connectivity for API calls
- Check that target language is supported
- Ensure input text complies with constitutional principles

## Support

For issues or questions, please contact the development team.