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
                chunks.append({
                    "content": point.payload.get("content", ""),
                    "url": point.payload.get("url", ""),
                    "page_title": point.payload.get("page_title", ""),
                    "section_title": point.payload.get("section_title", ""),
                    "score": point.score
                })

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

            for chunk in chunks[:4]:  # Use top 4 chunks
                content = chunk.get('content', '').strip()
                if content and len(content) > 30:
                    all_content.append(content)
                    sources_info.append({
                        'section': chunk.get('section_title', 'Unknown Section'),
                        'page': chunk.get('page_title', 'Unknown Page'),
                        'score': chunk.get('score', 0)
                    })

            if not all_content:
                return "I found some references but couldn't extract detailed content. Please try a more specific question."

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

            # Add sources
            response_parts.append("## 📚 Sources:")
            for source in sources_info:
                response_parts.append(f"• {source['section']} - {source['page']} (Relevance: {source['score']:.2f})")
            response_parts.append("")

            # Add helpful conclusion
            response_parts.append("## 🎯 Next Steps:")
            response_parts.append("• Review the referenced chapters for complete implementation details")
            response_parts.append("• Try hands-on exercises to reinforce learning")
            response_parts.append("• Ask follow-up questions about specific aspects")

            return "\n".join(response_parts)

        except Exception as e:
            logger.error(f"Comprehensive explanation failed: {e}")
            return self._generate_simple_explanation(query, chunks)

    def _create_main_explanation(self, query: str, content_list: List[str]) -> str:
        """Create the main explanation by intelligently combining content"""
        # Combine all content
        combined_content = " ".join(content_list)

        # Create explanation based on query type
        query_lower = query.lower()

        if any(word in query_lower for word in ['what is', 'define', 'explain']):
            return self._create_definition_explanation(query, combined_content)
        elif any(word in query_lower for word in ['how to', 'how do', 'steps', 'implement']):
            return self._create_how_to_explanation(query, combined_content)
        elif any(word in query_lower for word in ['why', 'purpose', 'benefit']):
            return self._create_why_explanation(query, combined_content)
        else:
            return self._create_general_explanation(query, combined_content)

    def _create_definition_explanation(self, query: str, content: str) -> str:
        """Create definition-focused explanation"""
        # Extract key sentences that contain definitions
        sentences = content.split('.')
        definition_sentences = []

        for sentence in sentences:
            sentence = sentence.strip()
            if any(phrase in sentence.lower() for phrase in [' is ', ' are ', ' refers to', ' means', ' defines']):
                if len(sentence) > 20 and len(sentence) < 200:
                    definition_sentences.append(sentence + '.')

        if definition_sentences:
            explanation = " ".join(definition_sentences[:3])  # Top 3 definition sentences
        else:
            # Fallback: use first substantial paragraph
            paragraphs = content.split('\n\n')
            explanation = next((p.strip() for p in paragraphs if len(p.strip()) > 100), content[:300])

        return explanation + "\n\nThis concept is fundamental in robotics and plays a crucial role in system design and implementation."

    def _create_how_to_explanation(self, query: str, content: str) -> str:
        """Create process-focused explanation"""
        # Look for step-by-step content
        lines = content.split('\n')
        process_lines = []

        for line in lines:
            line = line.strip()
            if any(indicator in line.lower() for indicator in ['step', 'first', 'then', 'next', 'finally', '1.', '2.', '3.']):
                if len(line) > 10:
                    process_lines.append(line)

        if process_lines:
            explanation = "Here's the process:\n\n" + "\n".join(process_lines[:5])
        else:
            # Extract procedural content
            sentences = content.split('.')
            procedural_sentences = []

            for sentence in sentences:
                sentence = sentence.strip()
                if any(word in sentence.lower() for word in ['create', 'setup', 'configure', 'install', 'run', 'execute']):
                    if len(sentence) > 20:
                        procedural_sentences.append(sentence + '.')

            explanation = " ".join(procedural_sentences[:4]) if procedural_sentences else content[:400]

        return explanation + "\n\nFollowing these steps systematically will help ensure successful implementation."

    def _create_why_explanation(self, query: str, content: str) -> str:
        """Create purpose/benefit-focused explanation"""
        sentences = content.split('.')
        purpose_sentences = []

        for sentence in sentences:
            sentence = sentence.strip()
            if any(phrase in sentence.lower() for phrase in ['because', 'since', 'purpose', 'benefit', 'advantage', 'important', 'essential']):
                if len(sentence) > 20:
                    purpose_sentences.append(sentence + '.')

        if purpose_sentences:
            explanation = " ".join(purpose_sentences[:3])
        else:
            explanation = content[:400]

        return explanation + "\n\nUnderstanding these reasons helps in making informed design decisions."

    def _create_general_explanation(self, query: str, content: str) -> str:
        """Create general comprehensive explanation"""
        # Take the most substantial paragraphs
        paragraphs = content.split('\n\n')
        substantial_paragraphs = [p.strip() for p in paragraphs if len(p.strip()) > 50]

        if substantial_paragraphs:
            explanation = "\n\n".join(substantial_paragraphs[:2])
        else:
            explanation = content[:500]

        return explanation + "\n\nThis information provides a solid foundation for understanding the topic."

    def _extract_key_points(self, content_list: List[str]) -> List[str]:
        """Extract key points from content"""
        key_points = []

        for content in content_list:
            lines = content.split('\n')
            for line in lines:
                line = line.strip()
                # Look for bullet points, important statements
                if (line.startswith('•') or line.startswith('-') or
                    'important' in line.lower() or 'key' in line.lower() or
                    'essential' in line.lower() or 'critical' in line.lower()):
                    if 20 < len(line) < 150:
                        clean_line = line.replace('•', '').replace('-', '').strip()
                        key_points.append(clean_line)

        return key_points[:5]  # Limit to 5 key points

    def _extract_implementation_steps(self, content_list: List[str]) -> List[str]:
        """Extract implementation steps"""
        steps = []

        for content in content_list:
            lines = content.split('\n')
            for line in lines:
                line = line.strip()
                if (any(line.startswith(f"{i}.") for i in range(1, 10)) or
                    any(word in line.lower() for word in ['step', 'first', 'then', 'next', 'finally'])):
                    if 10 < len(line) < 200:
                        steps.append(line)

        return steps[:6]  # Limit to 6 steps

    def _extract_examples(self, content_list: List[str]) -> List[str]:
        """Extract examples from content"""
        examples = []

        for content in content_list:
            sentences = content.split('.')
            for sentence in sentences:
                sentence = sentence.strip()
                if any(phrase in sentence.lower() for phrase in ['example', 'for instance', 'such as', 'like']):
                    if 20 < len(sentence) < 180:
                        examples.append(sentence + '.')

        return examples[:4]  # Limit to 4 examples

    def _extract_technical_details(self, content_list: List[str], query: str) -> str:
        """Extract technical details relevant to the query"""
        technical_content = []

        for content in content_list:
            # Look for technical terms, code, specifications
            lines = content.split('\n')
            for line in lines:
                line = line.strip()
                if (any(term in line.lower() for term in ['parameter', 'config', 'setting', 'value', 'specification']) or
                    '=' in line or ':' in line or line.startswith('```')):
                    if len(line) > 10:
                        technical_content.append(line)

        if technical_content:
            return "\n".join(technical_content[:5])
        else:
            # Return most technical-sounding paragraph
            for content in content_list:
                if any(term in content.lower() for term in ['algorithm', 'method', 'function', 'class', 'module']):
                    return content[:300] + "..."

        return ""

    def _generate_simple_explanation(self, query: str, chunks: List[Dict[str, Any]]) -> str:
        """Simple fallback explanation"""
        response_parts = [f"## About: {query}\n"]

        for i, chunk in enumerate(chunks[:2], 1):
            content = chunk.get('content', '').strip()
            section = chunk.get('section_title', 'Unknown Section')

            if content:
                # Take first substantial sentence
                sentences = content.split('.')
                first_sentence = next((s.strip() + '.' for s in sentences if len(s.strip()) > 30), content[:200])

                response_parts.append(f"**{section}:**")
                response_parts.append(first_sentence)
                response_parts.append("")

        response_parts.append("💡 For more detailed information, please refer to the complete chapters in the book.")

        return "\n".join(response_parts)

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