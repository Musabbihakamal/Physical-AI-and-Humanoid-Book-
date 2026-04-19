import React, { useState, useRef, useEffect } from 'react';
import styles from './RAGChatWidget.module.css';
import { API_BASE_URL } from '../../constants/apiConfig';
import { useAuth } from '../../contexts/AuthContext';

const RAGChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [sessionId, setSessionId] = useState(null);
  const [historyLoaded, setHistoryLoaded] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Safely get auth context - handle case where provider might not be available yet
  let user = null;
  let token = null;
  try {
    const auth = useAuth();
    user = auth.user;
    token = auth.token;
  } catch (e) {
    // AuthProvider not available yet, continue without auth
    console.log('Auth context not available, continuing without authentication');
  }

  // Check backend connection on component mount
  useEffect(() => {
    const checkBackend = async () => {
      try {
        const response = await fetch(`${API_BASE_URL}/health`);
        if (response.ok) {
          console.log('✅ Backend is accessible at:', API_BASE_URL);
        } else {
          console.warn('⚠️ Backend returned status:', response.status);
        }
      } catch (err) {
        console.error('❌ Backend not accessible at:', API_BASE_URL, err);
      }
    };
    checkBackend();
  }, []);

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Load chat history when user is authenticated and widget opens
  useEffect(() => {
    if (isOpen && user && token && !historyLoaded) {
      loadChatHistory();
    }
  }, [isOpen, user, token, historyLoaded]);

  // Add welcome message on first open (for non-authenticated or after history load)
  useEffect(() => {
    if (isOpen && messages.length === 0 && (!user || historyLoaded)) {
      const welcomeMessage = user
        ? `Hello ${user.full_name}! I'm your AI assistant for the Physical AI & Humanoid Robotics book. Your chat history has been loaded. Ask me anything!`
        : 'Hello! I\'m your AI assistant for the Physical AI & Humanoid Robotics book. Ask me anything about ROS 2, Gazebo, Isaac Sim, or any topic covered in the book!';

      setMessages([
        {
          type: 'bot',
          content: welcomeMessage,
          timestamp: new Date(),
        },
      ]);
    }
  }, [isOpen, messages.length, user, historyLoaded]);

  const loadChatHistory = async () => {
    try {
      const headers = {
        'Content-Type': 'application/json',
      };

      if (token) {
        headers['Authorization'] = `Bearer ${token}`;
      }

      const response = await fetch(`${API_BASE_URL}/api/rag/history?limit=50`, {
        method: 'GET',
        headers: headers,
      });

      if (response.ok) {
        const data = await response.json();

        if (data.history && data.history.length > 0) {
          // Convert history to message format
          const historyMessages = [];
          data.history.forEach((item) => {
            historyMessages.push({
              type: 'user',
              content: item.query,
              timestamp: new Date(item.created_at),
            });

            let botContent = item.response;
            if (item.sources && item.sources.length > 0) {
              botContent += '\n\n**Sources:**\n';
              item.sources.forEach((source, index) => {
                botContent += `${index + 1}. [${source.page_title}](${source.url})\n`;
              });
            }

            historyMessages.push({
              type: 'bot',
              content: botContent,
              sources: item.sources,
              timestamp: new Date(item.created_at),
            });
          });

          setMessages(historyMessages);
          setSessionId(data.session_id);
        }
      }
    } catch (err) {
      console.error('Failed to load chat history:', err);
    } finally {
      setHistoryLoaded(true);
    }
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      type: 'user',
      content: inputValue,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    const currentQuestion = inputValue;
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      console.log('🔵 RAG Query - Backend URL:', API_BASE_URL);
      console.log('🔵 RAG Query - Endpoint:', `${API_BASE_URL}/api/rag/query`);

      const headers = {
        'Content-Type': 'application/json',
      };

      // Add authentication token if user is logged in
      if (token) {
        headers['Authorization'] = `Bearer ${token}`;
      }

      const requestBody = {
        question: currentQuestion,
        max_chunks: 5,
        threshold: 0.3,
      };

      // Include session ID for conversation continuity
      if (sessionId) {
        requestBody.session_id = sessionId;
      }

      console.log('🔵 RAG Query - Request body:', requestBody);

      // Set up request timeout (30 seconds)
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 30000);

      const response = await fetch(`${API_BASE_URL}/api/rag/query`, {
        method: 'POST',
        headers: headers,
        body: JSON.stringify(requestBody),
        signal: controller.signal,
      });

      clearTimeout(timeoutId);
      console.log('🔵 RAG Query - Response status:', response.status);

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        const errorCode = errorData.error_code || 'unknown_error';
        const errorDetail = errorData.detail || `API error: ${response.status}`;

        console.error('🔴 RAG Query - Error response:', { status: response.status, errorCode, errorDetail });

        let userFriendlyError = 'Failed to get response. ';

        if (response.status === 429) {
          // Rate limit exceeded
          const retryAfter = errorData.retry_after || response.headers.get('Retry-After') || 60;
          const limit = errorData.limit || 'unknown';

          if (user) {
            userFriendlyError += `You've reached the rate limit (${limit} requests). Please wait ${retryAfter} seconds before trying again.`;
          } else {
            userFriendlyError += `Rate limit reached (${limit} requests). Sign in for higher limits or wait ${retryAfter} seconds.`;
          }

          // Set a timer to re-enable the input after retry period
          setTimeout(() => {
            setError(null);
          }, retryAfter * 1000);

        } else if (response.status === 401) {
          userFriendlyError += 'Please sign in to continue.';
        } else if (response.status === 500 && errorCode === 'rag_bot_init_failed') {
          userFriendlyError += 'The RAG system is not available. Please try again later.';
        } else if (response.status === 500 && errorCode === 'no_relevant_chunks') {
          userFriendlyError += 'No relevant information found for your question.';
        } else {
          userFriendlyError += 'Please check if the backend server is running.';
        }

        throw new Error(userFriendlyError);
      }

      const data = await response.json();
      console.log('🟢 RAG Query - Success:', data);

      // Save session ID for future queries
      if (data.session_id) {
        setSessionId(data.session_id);
      }

      // Format the bot response with sources
      let botContent = data.answer;

      if (data.sources && data.sources.length > 0) {
        botContent += '\n\n**Sources:**\n';
        data.sources.forEach((source, index) => {
          botContent += `${index + 1}. [${source.page_title}](${source.url})\n`;
        });
      }

      const botMessage = {
        type: 'bot',
        content: botContent,
        sources: data.sources,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, botMessage]);
    } catch (err) {
      console.error('RAG query error:', err);

      let errorMessage = err.message || 'Failed to get response. Please make sure the backend server is running.';
      if (err.name === 'AbortError') {
        errorMessage = 'Request timed out. The server took too long to respond. Please try again.';
      }

      setError(errorMessage);

      const errorMsg = {
        type: 'bot',
        content: `Sorry, I encountered an error: ${errorMessage}`,
        isError: true,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, errorMsg]);
    } finally {
      setIsLoading(false);
    }
  };

  const toggleWidget = () => {
    console.log('Toggle widget clicked, current state:', isOpen);
    setIsOpen(!isOpen);
  };

  const clearChat = async () => {
    if (user && token && sessionId) {
      // Delete chat history from server
      try {
        await fetch(`${API_BASE_URL}/api/rag/history/${sessionId}`, {
          method: 'DELETE',
          headers: {
            'Authorization': `Bearer ${token}`,
          },
        });
      } catch (err) {
        console.error('Failed to delete chat history:', err);
      }
    }

    setMessages([]);
    setSessionId(null);
    setHistoryLoaded(false);
    setError(null);
  };

  return (
    <>
      {/* Floating button */}
      {!isOpen && (
        <button
          className={styles.floatingButton}
          onClick={toggleWidget}
          aria-label="Open chat assistant"
          title="Ask questions about the book"
        >
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
          {user && <span className={styles.userIndicator}>●</span>}
        </button>
      )}

      {/* Chat widget */}
      {isOpen && (
        <div className={styles.chatWidget}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.headerContent}>
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
              </svg>
              <span>Book Assistant</span>
              {user && <span className={styles.userBadge}>{user.full_name}</span>}
            </div>
            <div className={styles.headerActions}>
              <button
                onClick={clearChat}
                className={styles.iconButton}
                aria-label="Clear chat"
                title="Clear chat history"
              >
                <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <polyline points="3 6 5 6 21 6" />
                  <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2" />
                </svg>
              </button>
              <button
                onClick={toggleWidget}
                className={styles.iconButton}
                aria-label="Close chat"
              >
                <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <line x1="18" y1="6" x2="6" y2="18" />
                  <line x1="6" y1="6" x2="18" y2="18" />
                </svg>
              </button>
            </div>
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.map((message, index) => (
              <div
                key={index}
                className={`${styles.message} ${
                  message.type === 'user' ? styles.userMessage : styles.botMessage
                } ${message.isError ? styles.errorMessage : ''}`}
              >
                <div className={styles.messageContent}>
                  {message.content.split('\n').map((line, i) => {
                    // Handle markdown links
                    const linkRegex = /\[([^\]]+)\]\(([^)]+)\)/g;
                    const parts = [];
                    let lastIndex = 0;
                    let match;

                    while ((match = linkRegex.exec(line)) !== null) {
                      if (match.index > lastIndex) {
                        parts.push(line.substring(lastIndex, match.index));
                      }
                      parts.push(
                        <a
                          key={`link-${i}-${match.index}`}
                          href={match[2]}
                          target="_blank"
                          rel="noopener noreferrer"
                          className={styles.sourceLink}
                        >
                          {match[1]}
                        </a>
                      );
                      lastIndex = match.index + match[0].length;
                    }

                    if (lastIndex < line.length) {
                      parts.push(line.substring(lastIndex));
                    }

                    return (
                      <React.Fragment key={i}>
                        {parts.length > 0 ? parts : line}
                        {i < message.content.split('\n').length - 1 && <br />}
                      </React.Fragment>
                    );
                  })}
                </div>
                <div className={styles.messageTime}>
                  {message.timestamp.toLocaleTimeString([], {
                    hour: '2-digit',
                    minute: '2-digit',
                  })}
                </div>
              </div>
            ))}

            {isLoading && (
              <div className={`${styles.message} ${styles.botMessage}`}>
                <div className={styles.messageContent}>
                  <div className={styles.typingIndicator}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Error display */}
          {error && (
            <div className={styles.errorBanner}>
              {error}
            </div>
          )}

          {/* Auth prompt for anonymous users */}
          {!user && messages.length > 2 && (
            <div className={styles.authPrompt}>
              <span>💡 Sign in to save your chat history!</span>
            </div>
          )}

          {/* Input form */}
          <form onSubmit={handleSubmit} className={styles.inputForm}>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask a question about the book..."
              className={styles.input}
              disabled={isLoading}
            />
            <button
              type="submit"
              className={styles.sendButton}
              disabled={isLoading || !inputValue.trim()}
              aria-label="Send message"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <line x1="22" y1="2" x2="11" y2="13" />
                <polygon points="22 2 15 22 11 13 2 9 22 2" />
              </svg>
            </button>
          </form>
        </div>
      )}
    </>
  );
};

export default RAGChatWidget;
