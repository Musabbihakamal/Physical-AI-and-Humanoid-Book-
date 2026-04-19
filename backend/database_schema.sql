-- Database Schema for RAG Chatbot System with User Profiles and Conversations

-- Users table for storing user profiles
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    username VARCHAR(100) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    first_name VARCHAR(100),
    last_name VARCHAR(100),
    profile_picture_url TEXT,
    bio TEXT,
    preferences JSONB DEFAULT '{}',
    is_verified BOOLEAN DEFAULT FALSE,
    is_active BOOLEAN DEFAULT TRUE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    last_login_at TIMESTAMP WITH TIME ZONE,
    password_reset_token VARCHAR(255),
    password_reset_expires_at TIMESTAMP WITH TIME ZONE,
    email_verification_token VARCHAR(255),
    email_verification_expires_at TIMESTAMP WITH TIME ZONE
);

-- Indexes for users table
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_username ON users(username);
CREATE INDEX idx_users_created_at ON users(created_at);

-- Conversations table for storing chat sessions
CREATE TABLE conversations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL,
    title VARCHAR(255) NOT NULL DEFAULT 'New Conversation',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    is_active BOOLEAN DEFAULT TRUE,
    metadata JSONB DEFAULT '{}',
    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
);

-- Indexes for conversations table
CREATE INDEX idx_conversations_user_id ON conversations(user_id);
CREATE INDEX idx_conversations_created_at ON conversations(created_at);
CREATE INDEX idx_conversations_user_active ON conversations(user_id, is_active);

-- Messages table for storing individual chat messages
CREATE TABLE messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    conversation_id UUID NOT NULL,
    user_id UUID NOT NULL,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
    content TEXT NOT NULL,
    sources JSONB, -- Stores references to book sections/chapters
    embeddings vector(1536), -- For semantic search (if using OpenAI embeddings)
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    token_count INTEGER DEFAULT 0,
    FOREIGN KEY (conversation_id) REFERENCES conversations(id) ON DELETE CASCADE,
    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
);

-- Indexes for messages table
CREATE INDEX idx_messages_conversation_id ON messages(conversation_id);
CREATE INDEX idx_messages_user_id ON messages(user_id);
CREATE INDEX idx_messages_role ON messages(role);
CREATE INDEX idx_messages_created_at ON messages(created_at);
CREATE INDEX idx_messages_conversation_time ON messages(conversation_id, created_at);
CREATE INDEX idx_messages_embeddings ON messages USING ivfflat (embeddings) WITH (lists = 100); -- For vector similarity search

-- Book content table for RAG system - stores the book content for retrieval
CREATE TABLE book_content (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter_id VARCHAR(50), -- e.g., "module-1/ch1-introduction"
    chapter_title VARCHAR(500),
    section_title VARCHAR(500),
    content TEXT,
    content_type VARCHAR(50) DEFAULT 'text', -- 'text', 'code', 'diagram', etc.
    page_number INTEGER,
    embeddings vector(1536), -- For semantic search
    token_count INTEGER DEFAULT 0,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    metadata JSONB DEFAULT '{}'
);

-- Indexes for book_content table
CREATE INDEX idx_book_content_chapter_id ON book_content(chapter_id);
CREATE INDEX idx_book_content_content_type ON book_content(content_type);
CREATE INDEX idx_book_content_embeddings ON book_content USING ivfflat (embeddings) WITH (lists = 100); -- For vector similarity search
CREATE INDEX idx_book_content_section_title ON book_content USING gin(to_tsvector('english', section_title));
CREATE INDEX idx_book_content_content ON book_content USING gin(to_tsvector('english', content));

-- Message sources table - links messages to specific book content
CREATE TABLE message_sources (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL,
    book_content_id UUID NOT NULL,
    relevance_score DECIMAL(3,2) DEFAULT 0.00, -- 0.00 to 1.00
    context_snippet TEXT, -- Excerpt from the book content
    reference_path VARCHAR(500), -- Path to the specific section
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (message_id) REFERENCES messages(id) ON DELETE CASCADE,
    FOREIGN KEY (book_content_id) REFERENCES book_content(id) ON DELETE CASCADE
);

-- Indexes for message_sources table
CREATE INDEX idx_message_sources_message_id ON message_sources(message_id);
CREATE INDEX idx_message_sources_book_content_id ON message_sources(book_content_id);
CREATE INDEX idx_message_sources_relevance_score ON message_sources(relevance_score);

-- User learning progress table - tracks user's progress through the book
CREATE TABLE user_learning_progress (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL,
    chapter_id VARCHAR(50) NOT NULL,
    section_id VARCHAR(100),
    progress_percentage DECIMAL(5,2) DEFAULT 0.00,
    time_spent_seconds INTEGER DEFAULT 0,
    last_accessed_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    completed_at TIMESTAMP WITH TIME ZONE,
    notes TEXT,
    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE,
    UNIQUE(user_id, chapter_id, section_id)
);

-- Indexes for user_learning_progress table
CREATE INDEX idx_user_learning_progress_user_id ON user_learning_progress(user_id);
CREATE INDEX idx_user_learning_progress_chapter_id ON user_learning_progress(chapter_id);
CREATE INDEX idx_user_learning_progress_user_chapter ON user_learning_progress(user_id, chapter_id);

-- User bookmarks table - allows users to save important sections
CREATE TABLE user_bookmarks (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL,
    book_content_id UUID NOT NULL,
    title VARCHAR(255),
    notes TEXT,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE,
    FOREIGN KEY (book_content_id) REFERENCES book_content(id) ON DELETE CASCADE
);

-- Indexes for user_bookmarks table
CREATE INDEX idx_user_bookmarks_user_id ON user_bookmarks(user_id);
CREATE INDEX idx_user_bookmarks_book_content_id ON user_bookmarks(book_content_id);

-- User feedback table - collects feedback on chatbot responses
CREATE TABLE user_feedback (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL,
    message_id UUID NOT NULL,
    conversation_id UUID NOT NULL,
    rating INTEGER CHECK (rating >= 1 AND rating <= 5), -- 1-5 star rating
    feedback_type VARCHAR(20) CHECK (feedback_type IN ('positive', 'negative', 'neutral')),
    comment TEXT,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE,
    FOREIGN KEY (message_id) REFERENCES messages(id) ON DELETE CASCADE,
    FOREIGN KEY (conversation_id) REFERENCES conversations(id) ON DELETE CASCADE
);

-- Indexes for user_feedback table
CREATE INDEX idx_user_feedback_user_id ON user_feedback(user_id);
CREATE INDEX idx_user_feedback_message_id ON user_feedback(message_id);
CREATE INDEX idx_user_feedback_rating ON user_feedback(rating);

-- Functions for updating timestamps
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ language 'plpgsql';

-- Triggers for updating 'updated_at' columns
CREATE TRIGGER update_users_updated_at BEFORE UPDATE ON users
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_conversations_updated_at BEFORE UPDATE ON conversations
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_messages_updated_at BEFORE UPDATE ON messages
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_book_content_updated_at BEFORE UPDATE ON book_content
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_user_bookmarks_updated_at BEFORE UPDATE ON user_bookmarks
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- RAG-specific indexes for efficient retrieval
-- Index for full-text search on book content
CREATE INDEX idx_book_content_fulltext ON book_content USING gin(to_tsvector('english', content || ' ' || section_title || ' ' || chapter_title));

-- Composite index for efficient conversation retrieval
CREATE INDEX idx_messages_conv_user_time ON messages(conversation_id, user_id, created_at DESC);