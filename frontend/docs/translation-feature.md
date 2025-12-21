---
sidebar_position: 15
title: "Translation Feature"
---

# Translation Feature

## Overview

The Physical AI and Humanoid Book project includes a powerful translation feature that allows users to translate entire chapters from English to Urdu directly within the documentation pages.

## How It Works

The translation feature uses Google Translate API to provide real-time translation while preserving:

- **Code blocks** - Syntax highlighting and formatting maintained
- **Diagrams and images** - All visual elements preserved
- **Tables** - Structure and formatting kept intact
- **Technical terms** - Programming keywords and domain-specific terminology preserved
- **HTML elements** - All complex content elements maintained

## Using the Translation Feature

1. Navigate to any chapter in the documentation
2. Look for the **"Translate to Urdu"** button below the content
3. Click the button to start the translation process
4. The content will be translated while maintaining all formatting and structure
5. Use the **"Switch to English"** button to revert back to the original content

## Technical Implementation

The translation system:

- Extracts content from the current page
- Preserves all code blocks, images, diagrams, and special elements using placeholder replacement
- Sends the text content to Google Translate API
- Restores all preserved elements back to their original positions
- Displays the translated content with all formatting intact

## Benefits

- **No external redirects** - Translation happens entirely within the page
- **Full content preservation** - All complex elements remain intact
- **Easy toggling** - Switch between English and Urdu versions with one click
- **Fast performance** - Optimized for quick translation of large chapters

## Supported Content Types

The translation feature preserves and translates:

- Text content and paragraphs
- Code blocks (fenced, inline, and HTML code tags)
- Images and figure elements
- Tables (Markdown and HTML)
- Diagrams and SVG elements
- Technical terminology and programming keywords
- HTML divs and other complex elements