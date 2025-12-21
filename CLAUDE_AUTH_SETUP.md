# Claude API Authentication Setup

## Overview
This document explains the changes made to fix Claude API authentication in the backend translation service.

## Problem
The backend translation service was failing with "Translation failed. Not authenticated" because:
1. It was using direct HTTP requests instead of the official Anthropic SDK
2. API key validation and error handling were insufficient
3. The service wasn't properly initialized at startup

## Solution Implemented

### 1. Official Anthropic SDK Integration
- Updated the translation service to use the official `anthropic` Python SDK
- Replaced direct HTTP requests with proper SDK client initialization
- Added proper error handling for authentication failures

### 2. API Key Management
- The service now loads API keys from environment variables via the existing settings system
- Uses `CLAUDE_API_KEY` environment variable (loaded through pydantic-settings from .env file)
- Added proper validation to check if the API key is available before initialization

### 3. Startup Validation
- Added API key validation in `main.py` that runs at application startup
- Logs which API keys are available without exposing actual key values
- Provides warnings when no translation API keys are configured

### 4. Enhanced Error Handling
- Improved error messages for authentication failures
- Added specific handling for Claude API authentication errors
- Better logging for debugging authentication issues

### 5. Dependencies
- Added `anthropic==0.21.3` to both requirements files
- The `python-dotenv` dependency was already present for environment variable loading

## Required Environment Variables
Set these in your `.env` file:
- `CLAUDE_API_KEY` - Your Anthropic Claude API key
- `OPENAI_API_KEY` - Optional, for fallback OpenAI translation

## Setup
1. Install dependencies: `pip install -r requirements.txt`
2. Create `.env` file from `.env.example` template
3. Add your Claude API key to the `.env` file
4. Start the backend: `python -m src.main`

## Automatic Configuration
- No manual setup required beyond adding API keys to `.env` file
- The system automatically detects available API keys at startup
- Fails gracefully with clear error messages if keys are missing
- Uses the official Anthropic SDK for secure API communication