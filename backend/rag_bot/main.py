"""
RAG Bot main module - recreated from missing file
"""
import os
import logging
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
import cohere
from anthropic import Anthropic
from dotenv import load_dotenv

# Load environment variables from parent directory
load_dotenv('../.env')

logger = logging.getLogger(__name__)

# Default configuration
DEFAULT_CONFIG = {
    "qdrant_url": os.getenv("QDRANT_URL", "http://localhost:6333"),
    "qdrant_api_key": os.getenv("QDRANT_API_KEY"),
    "rag_cohere_model": "command",
    "cohere_api_key": os.getenv("COHERE_API_KEY"),
    "collection_name": "docusaurus_embeddings"
}

class RagBot:
    """RAG Bot for querying documentation"""

    def __init__(self, qdrant_url: str, cohere_model: str, collection_name: str = "docusaurus_embeddings"):
        self.qdrant_url = qdrant_url
        self.cohere_model = cohere_model
        self.collection_name = collection_name

        # Initialize clients
        try:
            self.qdrant_client = QdrantClient(
                url=qdrant_url,
                api_key=os.getenv("QDRANT_API_KEY")
            )
            logger.info(f"Connected to Qdrant at {qdrant_url}")
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {e}")
            raise

        try:
            self.cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
            logger.info("Connected to Cohere")
        except Exception as e:
            logger.error(f"Failed to connect to Cohere: {e}")
            raise

        try:
            import httpx
            # Create httpx client without proxy configuration
            http_client = httpx.Client()
            self.anthropic_client = Anthropic(
                api_key=os.getenv("ANTHROPIC_API_KEY"),
                http_client=http_client
            )
            logger.info("Connected to Anthropic")
        except Exception as e:
            logger.error(f"Failed to connect to Anthropic: {e}")
            raise

    def retrieve_relevant_chunks(self, query: str, limit: int = 5, threshold: float = 0.3) -> List[Dict[str, Any]]:
        """Retrieve relevant chunks from Qdrant"""
        try:
            # Generate embedding for query using Cohere
            response = self.cohere_client.embed(
                texts=[query],
                model="embed-english-v3.0",
                input_type="search_query"
            )
            query_embedding = response.embeddings[0]

            # Use query_points method for vector search
            search_result = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=limit,
                score_threshold=threshold
            )

            chunks = []
            for point in search_result.points:
                # Try multiple possible content field names
                content = (point.payload.get("chunk_text") or
                          point.payload.get("content") or
                          point.payload.get("text") or "")

                chunk_data = {
                    "content": content,
                    "url": point.payload.get("url", ""),
                    "page_title": point.payload.get("page_title", ""),
                    "section_title": point.payload.get("section_title", ""),
                    "score": point.score
                }

                # Debug logging
                logger.info(f"Chunk content length: {len(content)}, Score: {point.score:.3f}")
                logger.info(f"Content preview: {content[:100]}...")

                chunks.append(chunk_data)

            logger.info(f"Retrieved {len(chunks)} chunks for query: {query[:50]}...")
            return chunks

        except Exception as e:
            logger.error(f"Failed to retrieve chunks: {e}")
            return []

    def generate_response(self, query: str, chunks: List[Dict[str, Any]]) -> str:
        """Generate comprehensive, detailed response using Claude AI with enhanced fallback"""
        try:
            if not chunks:
                return "I couldn't find relevant information about your question in the Physical AI and Humanoid Robotics book. Please try rephrasing your question or ask about topics like ROS2, robot simulation, control systems, or humanoid locomotion."

            # Try Claude AI first (now that ANTHROPIC_API_KEY is available)
            try:
                logger.info(f"Using Claude AI for intelligent response generation: {query[:50]}...")

                # Prepare context from chunks
                context_parts = []
                sources = []

                for i, chunk in enumerate(chunks[:5], 1):  # Use top 5 chunks for better context
                    section_title = chunk.get('section_title', 'Unknown Section')
                    page_title = chunk.get('page_title', 'Unknown Page')
                    content = chunk.get('content', '').strip()
                    score = chunk.get('score', 0)

                    if content and len(content) > 50:  # Only include substantial content
                        context_parts.append(f"Source {i} - {section_title} ({page_title}) [Relevance: {score:.2f}]:\n{content}")
                        sources.append(f"• {section_title} - {page_title}")

                combined_context = "\n\n".join(context_parts)

                # Create comprehensive prompt for Claude
                prompt = f"""You are an expert Physical AI and Humanoid Robotics instructor. A student has asked: "{query}"

Based on the following content from the Physical AI and Humanoid Robotics book, provide a comprehensive, detailed, and educational answer:

{combined_context}

Please provide a response that:
1. **Directly answers the question** with technical accuracy
2. **Explains concepts thoroughly** with clear definitions
3. **Includes practical examples** and real-world applications
4. **Provides step-by-step guidance** when applicable
5. **Mentions related topics** the student should explore
6. **Uses proper technical terminology** while remaining accessible
7. **Includes safety considerations** when relevant
8. **Suggests next learning steps** or related chapters

Format your response with clear headings and bullet points for better readability. Make it comprehensive but well-structured."""

                # Generate response using Claude
                response = self.anthropic_client.messages.create(
                    model="claude-3-5-sonnet-20241022",
                    max_tokens=1500,
                    temperature=0.3,
                    messages=[{
                        "role": "user",
                        "content": prompt
                    }]
                )

                ai_response = response.content[0].text

                # Add sources section
                if sources:
                    sources_text = "\n\n📚 **Sources from the Book:**\n" + "\n".join(sources)
                    ai_response += sources_text

                # Add helpful footer
                ai_response += "\n\n💡 **Need more details?** Ask follow-up questions about specific aspects or check the referenced chapters for complete information!"

                logger.info(f"✅ Claude AI generated comprehensive response for: {query[:50]}...")
                return ai_response

            except Exception as api_error:
                logger.warning(f"Claude AI failed, using enhanced fallback: {api_error}")
                return self._generate_comprehensive_explanation(query, chunks)

        except Exception as e:
            logger.error(f"Failed to generate response: {str(e)}", exc_info=True)
            return "I encountered an error while processing your question. Please try asking again or rephrase your question."

    def _generate_comprehensive_explanation(self, query: str, chunks: List[Dict[str, Any]]) -> str:
        """Generate a comprehensive explanation from the chunks"""
        try:
            response_parts = []

            # Smart introduction
            intro = self._generate_smart_intro(query)
            response_parts.append(intro)

            # Combine and analyze all chunk content
            all_content = []
            sources_info = []
            has_substantial_content = False

            for chunk in chunks[:4]:  # Use top 4 chunks
                content = chunk.get('content', '').strip()
                section_title = chunk.get('section_title', 'Unknown Section')
                page_title = chunk.get('page_title', 'Unknown Page')
                score = chunk.get('score', 0)

                # Add to sources regardless of content
                sources_info.append({
                    'section': section_title,
                    'page': page_title,
                    'score': score
                })

                if content and len(content) > 30:
                    all_content.append(content)
                    has_substantial_content = True
                    logger.info(f"✅ Found substantial content: {len(content)} chars")
                else:
                    logger.warning(f"⚠️ Empty/short content for {section_title}: '{content[:50]}'")

            # If we have substantial content, generate full explanation
            if has_substantial_content:
                # Generate main explanation
                main_explanation = self._create_main_explanation(query, all_content)
                response_parts.append("## 📖 Explanation:")
                response_parts.append(main_explanation)
                response_parts.append("")

                # Extract and add key points
                key_points = self._extract_key_points(all_content)
                if key_points:
                    response_parts.append("## 🔑 Key Points:")
                    for point in key_points:
                        response_parts.append(f"• {point}")
                    response_parts.append("")

                # Add implementation details if found
                implementation_steps = self._extract_implementation_steps(all_content)
                if implementation_steps:
                    response_parts.append("## 🛠️ Implementation:")
                    for i, step in enumerate(implementation_steps, 1):
                        response_parts.append(f"{i}. {step}")
                    response_parts.append("")

                # Add examples if found
                examples = self._extract_examples(all_content)
                if examples:
                    response_parts.append("## 💡 Examples:")
                    for example in examples:
                        response_parts.append(f"• {example}")
                    response_parts.append("")

                # Add technical details section
                technical_details = self._extract_technical_details(all_content, query)
                if technical_details:
                    response_parts.append("## ⚙️ Technical Details:")
                    response_parts.append(technical_details)
                    response_parts.append("")

            else:
                # Fallback: Generate response based on metadata and query context
                logger.info("🔄 Using metadata-based response generation")
                response_parts.append("## 📖 Based on Available Information:")

                metadata_response = self._generate_metadata_based_response(query, sources_info)
                response_parts.append(metadata_response)
                response_parts.append("")

                # Add context-based explanation
                context_explanation = self._generate_context_based_explanation(query)
                if context_explanation:
                    response_parts.append("## 🎯 General Context:")
                    response_parts.append(context_explanation)
                    response_parts.append("")

            # Add sources
            response_parts.append("## 📚 Sources:")
            for source in sources_info:
                response_parts.append(f"• {source['section']} - {source['page']} (Relevance: {source['score']:.2f})")
            response_parts.append("")

            # Add helpful conclusion
            response_parts.append("## 🎯 Next Steps:")
            response_parts.append("• Check the referenced chapters for complete implementation details")
            response_parts.append("• Try more specific questions about particular aspects")
            response_parts.append("• Ask about related topics for broader understanding")

            return "\n".join(response_parts)

        except Exception as e:
            logger.error(f"Comprehensive explanation failed: {e}")
            return self._generate_simple_explanation(query, chunks)

    def _generate_metadata_based_response(self, query: str, sources_info: List[Dict]) -> str:
        """Generate response based on metadata when content is limited"""
        try:
            # Analyze the sources to provide context
            sections = [source['section'] for source in sources_info if source['section'] != 'Unknown Section']
            pages = [source['page'] for source in sources_info if source['page'] != 'Unknown Page']

            response_parts = []

            if sections:
                unique_sections = list(set(sections))
                response_parts.append(f"Your question about '{query}' relates to the following areas:")
                for section in unique_sections[:3]:
                    response_parts.append(f"• **{section}**")

            # Provide context-based information
            query_lower = query.lower()

            if any(term in query_lower for term in ['control', 'stability', 'humanoid']):
                response_parts.append("\n**Humanoid Control Systems** typically involve:")
                response_parts.append("• Balance and stability algorithms (ZMP, CoM control)")
                response_parts.append("• Joint trajectory planning and execution")
                response_parts.append("• Sensor fusion for state estimation")
                response_parts.append("• Real-time feedback control loops")

            elif any(term in query_lower for term in ['ros', 'ros2', 'node', 'communication']):
                response_parts.append("\n**ROS2 Systems** generally include:")
                response_parts.append("• Node-based architecture for modular design")
                response_parts.append("• Topic-based publish/subscribe communication")
                response_parts.append("• Service calls for request/response patterns")
                response_parts.append("• Parameter management and configuration")

            elif any(term in query_lower for term in ['simulation', 'gazebo', 'physics']):
                response_parts.append("\n**Robot Simulation** typically covers:")
                response_parts.append("• Physics engine configuration and parameters")
                response_parts.append("• Robot model description (URDF/SDF)")
                response_parts.append("• Sensor simulation and noise modeling")
                response_parts.append("• Environment and world setup")

            return "\n".join(response_parts)

        except Exception as e:
            logger.error(f"Metadata-based response failed: {e}")
            return f"The referenced chapters contain information about '{query}', but detailed content extraction failed."

    def _generate_context_based_explanation(self, query: str) -> str:
        """Generate general context explanation based on query"""
        query_lower = query.lower()

        if any(term in query_lower for term in ['control', 'pid', 'feedback']):
            return """Control systems in robotics involve mathematical models and algorithms that regulate robot behavior.
Key concepts include feedback loops, stability analysis, and real-time performance optimization.
Modern humanoid robots use advanced control strategies like Model Predictive Control (MPC) and adaptive control."""

        elif any(term in query_lower for term in ['humanoid', 'bipedal', 'walking']):
            return """Humanoid robotics focuses on creating robots with human-like form and movement capabilities.
This involves complex challenges in balance, locomotion, and dynamic stability.
Key areas include gait generation, zero moment point (ZMP) control, and whole-body motion planning."""

        elif any(term in query_lower for term in ['ros', 'ros2', 'middleware']):
            return """ROS2 is a robotics middleware that provides tools and libraries for building robot applications.
It offers a distributed computing framework with standardized communication patterns.
Core features include node management, message passing, and hardware abstraction."""

        elif any(term in query_lower for term in ['simulation', 'gazebo', 'virtual']):
            return """Robot simulation enables testing and development in virtual environments before real-world deployment.
This includes physics simulation, sensor modeling, and realistic environmental conditions.
Popular platforms include Gazebo, Unity, and specialized robotics simulators."""

        return ""

    def _create_main_explanation(self, query: str, content_list: List[str]) -> str:
        """Create the main explanation by intelligently combining content"""
        # Combine and clean content
        combined_content = " ".join(content_list)

        # Remove duplicates and clean up
        sentences = self._extract_unique_sentences(combined_content)

        # Create explanation based on query type
        query_lower = query.lower()

        if any(word in query_lower for word in ['what is', 'define', 'explain']):
            return self._create_clean_definition_explanation(query, sentences)
        elif any(word in query_lower for word in ['how to', 'how do', 'steps', 'implement']):
            return self._create_clean_how_to_explanation(query, sentences)
        elif any(word in query_lower for word in ['why', 'purpose', 'benefit']):
            return self._create_clean_why_explanation(query, sentences)
        else:
            return self._create_clean_general_explanation(query, sentences)

    def _extract_unique_sentences(self, content: str) -> List[str]:
        """Extract unique, meaningful sentences from content"""
        # Split into sentences and clean
        sentences = []
        for delimiter in ['. ', '.\n', '? ', '!\n']:
            content = content.replace(delimiter, '.|SPLIT|')

        raw_sentences = content.split('|SPLIT|')
        seen_sentences = set()

        for sentence in raw_sentences:
            sentence = sentence.strip()
            # Skip short, code-like, or repetitive sentences
            if (len(sentence) > 20 and len(sentence) < 300 and
                not sentence.startswith('def ') and
                not sentence.startswith('class ') and
                not sentence.startswith('import ') and
                not sentence.startswith('```') and
                sentence.count('_') < 5 and  # Skip variable-heavy lines
                sentence not in seen_sentences):

                sentences.append(sentence)
                seen_sentences.add(sentence)

        return sentences[:8]  # Limit to 8 unique sentences

    def _create_clean_definition_explanation(self, query: str, sentences: List[str]) -> str:
        """Create clean definition-focused explanation"""
        definition_sentences = []

        for sentence in sentences:
            if any(phrase in sentence.lower() for phrase in [' is ', ' are ', ' refers to', ' means', ' defines']):
                definition_sentences.append(sentence)

        if definition_sentences:
            explanation = ". ".join(definition_sentences[:3]) + "."
        else:
            # Use first few substantial sentences
            explanation = ". ".join(sentences[:3]) + "."

        return explanation + "\n\nThis concept is fundamental in robotics and essential for understanding advanced control systems."

    def _create_clean_how_to_explanation(self, query: str, sentences: List[str]) -> str:
        """Create clean process-focused explanation"""
        process_sentences = []

        for sentence in sentences:
            if any(word in sentence.lower() for word in ['implement', 'create', 'design', 'calculate', 'plan', 'control']):
                process_sentences.append(sentence)

        if process_sentences:
            explanation = ". ".join(process_sentences[:4]) + "."
        else:
            explanation = ". ".join(sentences[:4]) + "."

        return explanation + "\n\nFollowing these principles systematically will help ensure successful implementation."

    def _create_clean_why_explanation(self, query: str, sentences: List[str]) -> str:
        """Create clean purpose/benefit-focused explanation"""
        purpose_sentences = []

        for sentence in sentences:
            if any(phrase in sentence.lower() for phrase in ['because', 'purpose', 'important', 'essential', 'crucial', 'enables']):
                purpose_sentences.append(sentence)

        if purpose_sentences:
            explanation = ". ".join(purpose_sentences[:3]) + "."
        else:
            explanation = ". ".join(sentences[:3]) + "."

        return explanation + "\n\nUnderstanding these principles helps in making informed design decisions."

    def _create_clean_general_explanation(self, query: str, sentences: List[str]) -> str:
        """Create clean general comprehensive explanation"""
        # Select the most informative sentences
        informative_sentences = []

        for sentence in sentences:
            # Prioritize sentences with technical terms but avoid code
            if (any(term in sentence.lower() for term in ['control', 'system', 'model', 'algorithm', 'robot', 'balance']) and
                not sentence.startswith('def ') and '=' not in sentence):
                informative_sentences.append(sentence)

        if informative_sentences:
            explanation = ". ".join(informative_sentences[:4]) + "."
        else:
            explanation = ". ".join(sentences[:4]) + "."

        return explanation + "\n\nThis provides a solid foundation for understanding the technical concepts involved."

    def _extract_key_points(self, content_list: List[str]) -> List[str]:
        """Extract clean key points from content"""
        key_points = []
        seen_points = set()

        for content in content_list:
            lines = content.split('\n')
            for line in lines:
                line = line.strip()
                # Look for bullet points and important statements
                if ((line.startswith('•') or line.startswith('-') or
                     'important' in line.lower() or 'key' in line.lower() or
                     'essential' in line.lower() or 'must' in line.lower()) and
                    30 < len(line) < 120 and  # Reasonable length
                    line not in seen_points and
                    not line.startswith('def ') and
                    '=' not in line):  # Avoid code lines

                    clean_line = line.replace('•', '').replace('-', '').strip()
                    if clean_line and len(clean_line) > 20:
                        key_points.append(clean_line)
                        seen_points.add(line)

        return key_points[:4]  # Limit to 4 key points

    def _extract_technical_details(self, content_list: List[str], query: str) -> str:
        """Extract clean technical details relevant to the query"""
        technical_lines = []
        seen_lines = set()

        for content in content_list:
            lines = content.split('\n')
            for line in lines:
                line = line.strip()
                # Look for formulas, equations, or technical specifications
                if (('=' in line or ':' in line or 'ω' in line or 'ZMP' in line or 'CoM' in line) and
                    20 < len(line) < 100 and  # Reasonable length
                    line not in seen_lines and
                    not line.startswith('def ') and
                    not line.startswith('class ') and
                    not line.startswith('import ')):

                    technical_lines.append(line)
                    seen_lines.add(line)

        if technical_lines:
            return "\n".join(technical_lines[:4])  # Limit to 4 technical details

        return ""

    def _extract_examples(self, content_list: List[str]) -> List[str]:
        """Extract clean examples from content"""
        examples = []
        seen_examples = set()

        for content in content_list:
            sentences = content.split('.')
            for sentence in sentences:
                sentence = sentence.strip()
                if (any(phrase in sentence.lower() for phrase in ['example', 'for instance', 'such as']) and
                    30 < len(sentence) < 150 and
                    sentence not in seen_examples and
                    not sentence.startswith('def ') and
                    '=' not in sentence):

                    examples.append(sentence + '.')
                    seen_examples.add(sentence)

        return examples[:3]  # Limit to 3 examples

    def _generate_simple_explanation(self, query: str, chunks: List[Dict[str, Any]]) -> str:
        """Generate clean, educational fallback explanation"""
        try:
            response_parts = []

            # Smart introduction based on query
            intro = self._generate_smart_intro(query)
            response_parts.append(intro)

            # Extract clean educational content
            educational_content = []
            unique_sources = set()

            for chunk in chunks[:3]:  # Limit to top 3 chunks
                content = chunk.get('content', '').strip()
                section = chunk.get('section_title', 'Unknown Section')
                page = chunk.get('page_title', 'Unknown Page')
                score = chunk.get('score', 0)

                # Add to unique sources
                source_key = f"{section} - {page}"
                if source_key not in unique_sources and section != 'Unknown Section':
                    unique_sources.add(source_key)

                if content:
                    # Extract only educational sentences, no code
                    clean_sentences = self._extract_educational_sentences(content)
                    if clean_sentences:
                        educational_content.extend(clean_sentences[:2])  # Max 2 sentences per chunk

            # Generate explanation
            if educational_content:
                response_parts.append("## 📖 Explanation:")
                explanation = ". ".join(educational_content[:4]) + "."  # Max 4 sentences total
                response_parts.append(explanation)
                response_parts.append("")

                # Add context based on query
                context = self._get_topic_context(query)
                if context:
                    response_parts.append("## 🎯 Key Concepts:")
                    response_parts.append(context)
                    response_parts.append("")
            else:
                # Generate context-based response when no clean content available
                response_parts.append("## 📖 About Your Question:")
                context_response = self._generate_context_based_explanation(query)
                response_parts.append(context_response)
                response_parts.append("")

            # Add unique sources
            if unique_sources:
                response_parts.append("## 📚 Sources:")
                for source in list(unique_sources)[:3]:  # Max 3 unique sources
                    response_parts.append(f"• {source}")
                response_parts.append("")

            # Add helpful conclusion
            response_parts.append("## 💡 Next Steps:")
            response_parts.append("• Review the referenced chapters for detailed implementation")
            response_parts.append("• Ask more specific questions about particular aspects")
            response_parts.append("• Explore related topics for broader understanding")

            return "\n".join(response_parts)

        except Exception as e:
            logger.error(f"Simple explanation failed: {e}")
            return self._generate_basic_response(query)

    def _extract_educational_sentences(self, content: str) -> List[str]:
        """Extract only clean, educational sentences from content"""
        sentences = []

        # Split content into sentences
        for delimiter in ['. ', '.\n', '? ', '!\n']:
            content = content.replace(delimiter, '.|SPLIT|')

        raw_sentences = content.split('|SPLIT|')

        for sentence in raw_sentences:
            sentence = sentence.strip()

            # Very strict filtering - only educational content
            if (self._is_educational_sentence(sentence)):
                sentences.append(sentence)

        return sentences[:3]  # Max 3 sentences

    def _is_educational_sentence(self, sentence: str) -> bool:
        """Check if sentence is educational and not code"""
        if not sentence or len(sentence) < 25 or len(sentence) > 200:
            return False

        # Reject code-like content
        code_indicators = [
            'def ', 'class ', 'import ', 'from ', '=', '()', '[]', '{}',
            'self.', '__', 'np.', 'dt', 'zmp_', 'com_', 'gravity=',
            '**', '++', '--', '=>', 'function', 'var ', 'let ', 'const '
        ]

        if any(indicator in sentence for indicator in code_indicators):
            return False

        # Reject sentences with too many underscores or technical symbols
        if sentence.count('_') > 2 or sentence.count('#') > 0:
            return False

        # Accept educational content
        educational_indicators = [
            'is a', 'are', 'refers to', 'means', 'defines', 'concept',
            'system', 'control', 'robot', 'model', 'algorithm', 'method',
            'important', 'essential', 'fundamental', 'used for', 'enables'
        ]

        if any(indicator in sentence.lower() for indicator in educational_indicators):
            return True

        # Accept sentences that explain concepts
        if any(word in sentence.lower() for word in ['humanoid', 'balance', 'stability', 'locomotion', 'walking']):
            return True

        return False

    def _get_topic_context(self, query: str) -> str:
        """Get contextual information based on query topic"""
        query_lower = query.lower()

        if 'lipm' in query_lower or 'linear inverted pendulum' in query_lower:
            return """• LIMP simplifies humanoid dynamics to a point mass on a massless rod
• Assumes constant center of mass height for linearized motion equations
• Widely used in bipedal robot gait planning and balance control
• Forms the basis for Zero Moment Point (ZMP) control strategies"""

        elif any(term in query_lower for term in ['zmp', 'zero moment point']):
            return """• ZMP is the point where net ground reaction moment is zero
• Must remain within support polygon for stable balance
• Critical for humanoid robot stability analysis
• Used in real-time balance control systems"""

        elif any(term in query_lower for term in ['control', 'stability', 'balance']):
            return """• Balance control ensures robot stability during motion
• Combines sensor feedback with predictive algorithms
• Requires real-time processing for dynamic environments
• Essential for safe humanoid robot operation"""

        elif any(term in query_lower for term in ['humanoid', 'bipedal', 'walking']):
            return """• Humanoid robots mimic human form and movement
• Bipedal locomotion presents unique stability challenges
• Requires sophisticated control algorithms for balance
• Applications include service robots and human interaction"""

        return ""

    def _generate_basic_response(self, query: str) -> str:
        """Most basic fallback response"""
        return f"""## 📖 About: {query}

Based on the Physical AI and Humanoid Robotics book, your question relates to advanced robotics concepts.

The referenced chapters contain detailed information about this topic, including theoretical foundations, practical implementations, and real-world applications.

## 💡 Recommendation:
For comprehensive understanding, please refer to the complete chapters in the book or ask more specific questions about particular aspects of the topic.

## 🎯 Related Topics:
• Robot control systems and stability
• Humanoid locomotion and balance
• Mathematical modeling in robotics
• Real-time control algorithms"""

    def _generate_enhanced_fallback_response(self, query: str, chunks: List[Dict[str, Any]]) -> str:
        """Enhanced fallback response generation without API"""
        try:
            response_parts = []

            # Intelligent introduction based on query keywords
            intro = self._generate_smart_intro(query)
            response_parts.append(intro)

            # Process chunks intelligently
            processed_chunks = self._process_chunks_intelligently(chunks)

            # Add main content sections
            if processed_chunks['definitions']:
                response_parts.append("## 📖 Key Concepts:")
                for definition in processed_chunks['definitions']:
                    response_parts.append(f"• **{definition['term']}**: {definition['definition']}")
                response_parts.append("")

            if processed_chunks['procedures']:
                response_parts.append("## 🔧 Implementation Steps:")
                for i, step in enumerate(processed_chunks['procedures'], 1):
                    response_parts.append(f"{i}. {step}")
                response_parts.append("")

            if processed_chunks['examples']:
                response_parts.append("## 💡 Practical Examples:")
                for example in processed_chunks['examples']:
                    response_parts.append(f"• {example}")
                response_parts.append("")

            # Add detailed content from top chunks
            response_parts.append("## 📚 Detailed Information:")
            for i, chunk in enumerate(chunks[:3], 1):
                section_title = chunk.get('section_title', 'Unknown Section')
                page_title = chunk.get('page_title', 'Unknown Page')
                content = chunk.get('content', '').strip()

                if content:
                    # Clean and format content
                    formatted_content = self._format_content(content)
                    response_parts.append(f"### {i}. {section_title}")
                    response_parts.append(f"*From: {page_title}*")
                    response_parts.append(formatted_content)
                    response_parts.append("")

            # Add related topics
            related_topics = self._extract_related_topics(chunks, query)
            if related_topics:
                response_parts.append("## 🔗 Related Topics to Explore:")
                for topic in related_topics:
                    response_parts.append(f"• {topic}")
                response_parts.append("")

            # Add helpful conclusion
            response_parts.append("## 🎯 Next Steps:")
            response_parts.append("• Review the referenced chapters for complete details")
            response_parts.append("• Practice with hands-on exercises if available")
            response_parts.append("• Ask follow-up questions about specific aspects")

            if len(chunks) > 3:
                response_parts.append(f"\n📊 Found {len(chunks)} relevant sections total. Ask for more specific details if needed!")

            return "\n".join(response_parts)

        except Exception as e:
            logger.error(f"Enhanced fallback failed: {e}")
            return f"Based on the book content about '{query}', I found relevant information but encountered processing issues. Please try rephrasing your question."

    def _generate_smart_intro(self, query: str) -> str:
        """Generate contextual introduction based on query keywords"""
        query_lower = query.lower()

        if any(word in query_lower for word in ['ros', 'ros2', 'node', 'topic', 'service']):
            return "## 🤖 ROS2 Robotics Framework\n\nRegarding your question about ROS2 concepts:\n"
        elif any(word in query_lower for word in ['simulation', 'gazebo', 'unity', 'physics']):
            return "## 🎮 Robot Simulation Environment\n\nFor robot simulation and virtual environments:\n"
        elif any(word in query_lower for word in ['control', 'pid', 'feedback', 'controller']):
            return "## ⚙️ Robot Control Systems\n\nAbout robot control and feedback systems:\n"
        elif any(word in query_lower for word in ['humanoid', 'bipedal', 'walking', 'locomotion']):
            return "## 🚶 Humanoid Robotics & Locomotion\n\nRegarding humanoid robot movement and control:\n"
        elif any(word in query_lower for word in ['urdf', 'sdf', 'model', 'description']):
            return "## 📐 Robot Description & Modeling\n\nFor robot modeling and description formats:\n"
        else:
            return f"## 📖 Physical AI & Robotics Concepts\n\nRegarding your question about '{query}':\n"

    def _process_chunks_intelligently(self, chunks: List[Dict[str, Any]]) -> Dict[str, List]:
        """Extract definitions, procedures, and examples from chunks"""
        definitions = []
        procedures = []
        examples = []

        for chunk in chunks:
            content = chunk.get('content', '').strip()
            if not content:
                continue

            # Extract definitions (lines with "is a", "refers to", "means")
            lines = content.split('\n')
            for line in lines:
                line = line.strip()
                if any(phrase in line.lower() for phrase in [' is a ', ' refers to ', ' means ', ' defines ']):
                    if len(line) < 200:  # Keep definitions concise
                        # Try to extract term and definition
                        for phrase in [' is a ', ' refers to ', ' means ']:
                            if phrase in line.lower():
                                parts = line.split(phrase, 1)
                                if len(parts) == 2:
                                    term = parts[0].strip().replace('**', '').replace('*', '')
                                    definition = parts[1].strip()
                                    definitions.append({'term': term, 'definition': definition})
                                break

                # Extract procedures (numbered steps, bullet points)
                if any(pattern in line for pattern in ['1.', '2.', '3.', 'Step ', 'First', 'Next', 'Then', 'Finally']):
                    if len(line) < 300:
                        procedures.append(line)

                # Extract examples
                if any(phrase in line.lower() for phrase in ['for example', 'example:', 'such as', 'like ']):
                    if len(line) < 250:
                        examples.append(line)

        return {
            'definitions': definitions[:3],  # Limit to avoid overwhelming
            'procedures': procedures[:5],
            'examples': examples[:3]
        }

    def _format_content(self, content: str) -> str:
        """Format content for better readability"""
        # Clean up the content
        lines = content.split('\n')
        formatted_lines = []

        for line in lines:
            line = line.strip()
            if line:
                # Add bullet points for list items
                if line.startswith('-') or line.startswith('•'):
                    formatted_lines.append(line)
                elif any(line.startswith(str(i) + '.') for i in range(1, 10)):
                    formatted_lines.append(line)
                else:
                    formatted_lines.append(line)

        formatted_content = '\n'.join(formatted_lines)

        # Truncate if too long
        if len(formatted_content) > 400:
            formatted_content = formatted_content[:400] + "...\n\n*[Content truncated - see full chapter for complete details]*"

        return formatted_content

    def _extract_related_topics(self, chunks: List[Dict[str, Any]], query: str) -> List[str]:
        """Extract related topics from chunks"""
        related_topics = set()
        query_words = set(query.lower().split())

        for chunk in chunks:
            section_title = chunk.get('section_title', '')
            page_title = chunk.get('page_title', '')

            # Add section and page titles as related topics if they're different from query
            if section_title and not any(word in section_title.lower() for word in query_words):
                related_topics.add(section_title)

            if page_title and not any(word in page_title.lower() for word in query_words):
                related_topics.add(page_title)

        return list(related_topics)[:5]  # Limit to 5 related topics