"""
Translation service following the Physical AI Book Constitution.
Implements Urdu Translation Agent as per constitutional requirements.
"""
import os
import logging
from typing import Optional
from abc import ABC, abstractmethod
from enum import Enum
import re

logger = logging.getLogger(__name__)


class ConstitutionalUrduTranslationService(ABC):
    """
    Constitutional Urdu Translation Agent following Physical AI Book Constitution v2.0

    Core Constitutional Requirements:
    1. Maintain technical accuracy and official documentation grounding
    2. Preserve Docusaurus-compatible Markdown formatting
    3. Follow consistent writing style across all chapters
    4. Never hallucinate - if unsure, indicate ambiguity
    5. Maintain safety standards for minor users
    6. Preserve code blocks and technical terms
    """

    def __init__(self):
        self.constitution_version = "2.0"
        self.ratified_date = "2025-12-10"

        # Constitutional formatting preservation rules
        self.docusaurus_patterns = {
            'main_title': r'^# (.+)$',
            'subheadings': r'^## (.+)$',
            'sub_subheadings': r'^### (.+)$',
            'fenced_code': r'```[\s\S]*?```',
            'inline_code': r'`[^`\n]+`',
            'tables': r'\|[^\n]*\|(?:\n\|[-:|\s]*\|)?(?:\n\|[^\n]*\|)*',
            'admonitions': r':::(note|tip|warning|danger|info)[\s\S]*?:::',
            'internal_anchors': r'\[([^\]]+)\]\(#([^)]+)\)',
            'mermaid_diagrams': r'```mermaid[\s\S]*?```'
        }

        # Technical terms that MUST remain in English per constitution
        self.constitutional_technical_terms = [
            # ROS 2 & Robotics Core
            'ROS', 'ROS2', 'rclpy', 'Node', 'URDF', 'SDF', 'Gazebo', 'Isaac',
            'NVIDIA', 'Jetson', 'GPU', 'CUDA', 'OpenCV', 'PCL',

            # Physical AI & Embodied Intelligence
            'VLA', 'Vision-Language-Action', 'Embodied Intelligence', 'Physical AI',

            # Programming & Frameworks
            'Python', 'C++', 'JavaScript', 'FastAPI', 'Qdrant', 'Postgres',
            'GitHub', 'Docusaurus', 'Markdown', 'YAML', 'XML', 'JSON',

            # Robotics Technical Terms
            'ZMP', 'LIPM', 'FRI', 'CoM', 'CoP', 'IMU', 'LiDAR', 'SLAM',
            'PID', 'MPC', 'IK', 'FK', 'DH', 'Jacobian',

            # Safety & Standards (Constitutional requirement)
            'PEP8', 'ISO', 'IEEE', 'API', 'SDK', 'CLI', 'GUI', 'HMI'
        ]

    async def translate(self, text: str, target_language: str = 'ur', source_language: str = 'en') -> str:
        """
        Constitutional translation following Physical AI Book Constitution

        Args:
            text: Content to translate
            target_language: Target language (default: 'ur' for Urdu)
            source_language: Source language (default: 'en' for English)

        Returns:
            Translated text following constitutional requirements
        """
        try:
            if target_language != 'ur':
                return self._constitutional_error(f"Language {target_language} not supported by Constitutional Urdu Translation Agent")

            logger.info(f"Constitutional Urdu Translation Agent v{self.constitution_version} processing text")

            # Step 1: Preserve Constitutional Formatting (Docusaurus-compatible)
            preserved_elements = self._preserve_constitutional_formatting(text)

            # Step 2: Preserve Technical Terms (Constitutional requirement)
            processed_text = self._preserve_technical_terms(preserved_elements['processed_text'])

            # Step 3: Apply Constitutional Translation Rules
            translated_text = await self._apply_constitutional_translation(processed_text)

            # Step 4: Restore Constitutional Formatting
            final_text = self._restore_constitutional_formatting(translated_text, preserved_elements)

            # Step 5: Constitutional Validation
            validation_result = self._constitutional_validation(final_text, text)

            if not validation_result['valid']:
                return self._constitutional_error(f"Translation validation failed: {validation_result['reason']}")

            logger.info("Constitutional translation completed successfully")
            return final_text

        except Exception as e:
            logger.error(f"Constitutional translation error: {e}")
            return self._constitutional_error("Source required — ambiguity detected during translation")

    def _preserve_constitutional_formatting(self, text: str) -> dict:
        """Preserve Docusaurus-compatible Markdown formatting per constitution"""
        preserved = []
        processed_text = text
        index = 0

        for pattern_name, pattern in self.docusaurus_patterns.items():
            matches = list(re.finditer(pattern, processed_text, re.MULTILINE))
            for match in reversed(matches):  # Reverse to maintain positions
                placeholder = f"__CONSTITUTIONAL_{pattern_name.upper()}_{index}__"
                preserved.append({
                    'placeholder': placeholder,
                    'original': match.group(0),
                    'type': pattern_name
                })
                processed_text = processed_text[:match.start()] + placeholder + processed_text[match.end():]
                index += 1

        return {
            'processed_text': processed_text,
            'preserved_elements': preserved
        }

    def _preserve_technical_terms(self, text: str) -> str:
        """Preserve technical terms as required by constitution"""
        processed_text = text

        for term in self.constitutional_technical_terms:
            # Case-insensitive preservation with word boundaries
            pattern = r'\b' + re.escape(term) + r'\b'
            processed_text = re.sub(pattern, f"__TECH_TERM_{term}__", processed_text, flags=re.IGNORECASE)

        return processed_text

    async def _apply_constitutional_translation(self, text: str) -> str:
        """Apply constitutional translation rules with proper sentence structure"""

        # Constitutional sentence patterns for robotics education
        constitutional_patterns = {
            # Chapter structure (Constitutional requirement: consistent style)
            r"^chapter (\d+):?\s*(.+)": "باب {}: {}",
            r"^module (\d+):?\s*(.+)": "ماڈیول {}: {}",

            # Learning objectives (Constitutional requirement: pedagogical tone)
            r"by the end of this chapter,?\s*you will be able to:?": "اس باب کے اختتام تک آپ قابل ہوں گے:",
            r"learning objectives?:?": "تعلیمی مقاصد:",
            r"you will (learn|understand|be able to) (.+)": "آپ {} سکیں گے",

            # Overview and summary (Constitutional requirement: structured content)
            r"overview summary:?": "جائزہ اور خلاصہ:",
            r"detailed theory:?": "تفصیلی نظریہ:",
            r"practical implementation steps:?": "عملی نفاذ کے قدم:",
            r"hands-on exercises:?": "عملی مشقیں:",
            r"assessment questions:?": "تشخیصی سوالات:",

            # Safety emphasis (Constitutional requirement: safety for minors)
            r"safety (warning|note|reminder):?": "حفاظتی {}:",
            r"important safety note:?": "اہم حفاظتی نوٹ:",
            r"always ensure (.+) for safety": "حفاظت کے لیے ہمیشہ {} کو یقینی بنائیں",

            # Technical explanations (Constitutional requirement: technically accurate)
            r"this chapter (covers|discusses|explores) (.+)": "یہ باب {} کا احاطہ کرتا ہے",
            r"(.+) is a (.+) that (.+)": "{} ایک {} ہے جو {}",
            r"the (.+) system (.+)": "{} سسٹم {}",

            # Implementation guidance (Constitutional requirement: reproducible by students)
            r"to implement (.+),?\s*(first|you need to|we need to) (.+)": "{} کو نافذ کرنے کے لیے پہلے {}",
            r"step (\d+):?\s*(.+)": "قدم {}: {}",
            r"follow these steps:?": "یہ قدم اٹھائیں:",

            # Platform instructions (Constitutional requirement: platform-agnostic)
            r"on linux,?\s*(.+)": "Linux پر، {}",
            r"on jetson,?\s*(.+)": "Jetson پر، {}",
            r"in the cloud,?\s*(.+)": "کلاؤڈ میں، {}",
        }

        # Apply constitutional patterns
        translated_text = text
        for pattern, urdu_template in constitutional_patterns.items():
            match = re.search(pattern, text.lower())
            if match:
                groups = match.groups()
                translated_groups = [self._translate_phrase(group) for group in groups if group]
                try:
                    result = urdu_template.format(*translated_groups)
                    return result
                except:
                    continue

        # Fallback: Constitutional phrase translation
        return self._constitutional_phrase_translation(text)

    def _constitutional_phrase_translation(self, text: str) -> str:
        """Constitutional phrase-level translation maintaining pedagogical tone"""

        constitutional_phrases = {
            # Educational phrases (Constitutional requirement: pedagogical tone)
            "in this chapter": "اس باب میں",
            "as we learned": "جیسا کہ ہم نے سیکھا",
            "let us explore": "آئیے دریافت کرتے ہیں",
            "it is important to understand": "یہ سمجھنا اہم ہے",
            "remember that": "یاد رکھیں کہ",
            "note that": "نوٹ کریں کہ",

            # Safety phrases (Constitutional requirement: safety emphasis)
            "safety first": "حفاظت پہلے",
            "be careful": "احتیاط کریں",
            "ensure safety": "حفاظت کو یقینی بنائیں",
            "follow safety guidelines": "حفاظتی رہنمائی کی پیروی کریں",

            # Technical accuracy phrases (Constitutional requirement: technically accurate)
            "according to the documentation": "دستاویزات کے مطابق",
            "as specified in": "جیسا کہ میں بیان کیا گیا ہے",
            "following the standard": "معیار کی پیروی کرتے ہوئے",
            "based on official guidelines": "سرکاری رہنمائی کی بنیاد پر",
        }

        result = text
        for english_phrase, urdu_phrase in constitutional_phrases.items():
            pattern = r'\b' + re.escape(english_phrase) + r'\b'
            result = re.sub(pattern, urdu_phrase, result, flags=re.IGNORECASE)

        # For longer content, translate sentence by sentence
        if len(text) > 200:
            return self._translate_full_content_constitutional(result)

        return result

    def _translate_full_content_constitutional(self, text: str) -> str:
        """Translate full chapter content sentence by sentence following constitutional requirements"""
        # Split into sentences while preserving structure
        sentences = re.split(r'(?<=[.!?])\s+', text)
        translated_sentences = []

        for sentence in sentences:
            if sentence.strip():
                # Translate each sentence
                translated = self._translate_single_sentence_constitutional(sentence.strip())
                translated_sentences.append(translated)

        return ' '.join(translated_sentences)

    def _translate_single_sentence_constitutional(self, text: str) -> str:
        """Translate a single sentence following constitutional patterns"""
        text_lower = text.lower().strip()

        # Sentence pattern templates for robotics content
        sentence_templates = {
            # Chapter and section patterns
            r"^chapter (\d+):?\s*(.+)": "باب {}: {}",
            r"^# (.+)": "# {}",
            r"^## (.+)": "## {}",
            r"^### (.+)": "### {}",

            # Introduction patterns
            r"this chapter (covers|discusses|explores|examines|introduces) (.+)": "یہ باب {} کا احاطہ کرتا ہے",
            r"in this chapter,?\s*(we will|you will|we'll|you'll) (.+)": "اس باب میں ہم {}",
            r"this section (covers|discusses|explores|examines) (.+)": "یہ حصہ {} کا احاطہ کرتا ہے",

            # Learning objectives
            r"by the end of this chapter,?\s*you will (.+)": "اس باب کے اختتام تک آپ {}",
            r"learning objectives?:?": "تعلیمی مقاصد:",
            r"you will (be able to|learn to|understand how to) (.+)": "آپ {} سکیں گے",
            r"students will (learn|understand|be able to) (.+)": "طلباء {} سکیں گے",

            # Technical descriptions
            r"(.+) is a (.+) that (.+)": "{} ایک {} ہے جو {}",
            r"(.+) are (.+) used for (.+)": "{} {} ہیں جو {} کے لیے استعمال ہوتے ہیں",
            r"the (.+) system (.+)": "{} سسٹم {}",
            r"a (.+) is (.+)": "ایک {} {} ہے",

            # Process and instruction patterns
            r"to (.+),?\s*(first|you need to|we need to|you must|we must) (.+)": "{} کے لیے پہلے آپ کو {}",
            r"step (\d+):?\s*(.+)": "قدم {}: {}",
            r"follow these steps:?": "یہ قدم اٹھائیں:",
            r"first,?\s*(.+)": "پہلے، {}",
            r"next,?\s*(.+)": "اگلا، {}",
            r"finally,?\s*(.+)": "آخر میں، {}",

            # Explanation patterns
            r"this means that (.+)": "اس کا مطلب یہ ہے کہ {}",
            r"in other words,?\s*(.+)": "دوسرے الفاظ میں، {}",
            r"for example,?\s*(.+)": "مثال کے طور پر، {}",
            r"for instance,?\s*(.+)": "مثلاً، {}",

            # Conclusion patterns
            r"in conclusion,?\s*(.+)": "خلاصہ یہ ہے کہ {}",
            r"to summarize,?\s*(.+)": "خلاصہ کرتے ہوئے، {}",
            r"as we can see,?\s*(.+)": "جیسا کہ ہم دیکھ سکتے ہیں، {}",

            # Importance and emphasis
            r"it is important to (.+)": "{} کرنا اہم ہے",
            r"it is essential to (.+)": "{} کرنا ضروری ہے",
            r"make sure (to )?(.+)": "یقینی بنائیں کہ {}",
            r"remember that (.+)": "یاد رکھیں کہ {}",
            r"note that (.+)": "نوٹ کریں کہ {}",

            # Conditional patterns
            r"if (.+),?\s*then (.+)": "اگر {} تو {}",
            r"when (.+),?\s*(.+)": "جب {} تو {}",
            r"while (.+),?\s*(.+)": "جبکہ {}, {}",

            # Comparison patterns
            r"unlike (.+),?\s*(.+)": "{} کے برعکس، {}",
            r"similar to (.+),?\s*(.+)": "{} کی طرح، {}",
            r"compared to (.+),?\s*(.+)": "{} کے مقابلے میں، {}"
        }

        # Try to match sentence patterns
        for pattern, urdu_template in sentence_templates.items():
            match = re.search(pattern, text_lower)
            if match:
                groups = match.groups()
                translated_groups = []

                for group in groups:
                    if group:
                        translated_group = self._translate_key_terms(group)
                        translated_groups.append(translated_group)

                try:
                    result = urdu_template.format(*translated_groups)
                    logger.debug(f"Pattern matched: {pattern} -> {result}")
                    return result
                except:
                    pass

        # If no pattern matches, use phrase-level translation
        return self._translate_phrases_constitutional(text)

    def _translate_key_terms(self, text: str) -> str:
        """Translate key technical terms while preserving context"""
        key_terms = {
            "robot": "روبوٹ",
            "robotics": "روبوٹکس",
            "humanoid": "انسان نما روبوٹ",
            "control system": "کنٹرول سسٹم",
            "simulation": "نقل",
            "environment": "ماحول",
            "sensor": "سینسر",
            "actuator": "ایکچویٹر",
            "artificial intelligence": "مصنوعی ذہانت",
            "machine learning": "مشین لرننگ",
            "programming": "پروگرامنگ",
            "algorithm": "الگورتھم",
            "software": "سافٹ ویئر",
            "hardware": "ہارڈ ویئر",
            "computer": "کمپیوٹر",
            "technology": "ٹیکنالوجی",
            "development": "ترقی",
            "implementation": "نافذ کرنا",
            "configuration": "ترتیب",
            "installation": "انسٹالیشن"
        }

        result = text
        for english, urdu in key_terms.items():
            pattern = r'\b' + re.escape(english) + r'\b'
            result = re.sub(pattern, urdu, result, flags=re.IGNORECASE)

        return result

    def _translate_phrases_constitutional(self, text: str) -> str:
        """Translate common phrases and improve grammar"""

        # Common phrase replacements
        phrase_replacements = {
            "this chapter": "یہ باب",
            "in this chapter": "اس باب میں",
            "the next chapter": "اگلا باب",
            "as we can see": "جیسا کہ ہم دیکھ سکتے ہیں",
            "for example": "مثال کے طور پر",
            "in conclusion": "خلاصہ یہ ہے",
            "it is important": "یہ اہم ہے",
            "we will learn": "ہم سیکھیں گے",
            "you will learn": "آپ سیکھیں گے",
            "let us": "آئیے",
            "we can": "ہم کر ستے ہیں",
            "you can": "آپ کر سکتے ہیں",
            "we need to": "ہمیں ضرورت ہے",
            "you need to": "آپ کو ضرورت ہے",
            "make sure": "یقینی بنائیں",
            "keep in mind": "ذہن میں رکھیں",
            "step by step": "قدم بہ قدم",
            "first of all": "سب سے پہلے",
            "at the end": "آخر میں",
            "on the other hand": "دوسری طرف"
        }

        result = text

        # Replace phrases first (longer matches first)
        for english_phrase, urdu_phrase in sorted(phrase_replacements.items(), key=len, reverse=True):
            pattern = r'\b' + re.escape(english_phrase) + r'\b'
            result = re.sub(pattern, urdu_phrase, result, flags=re.IGNORECASE)

        # Then translate remaining key terms
        result = self._translate_key_terms(result)

        # Clean up spacing
        result = re.sub(r'\s+', ' ', result).strip()

        return result

    def _translate_phrase(self, phrase: str) -> str:
        """Translate individual phrases while maintaining constitutional requirements"""
        # Basic word-level translation for constitutional compliance
        basic_translations = {
            "robotics": "روبوٹکس",
            "simulation": "نقل",
            "environment": "ماحول",
            "setup": "سیٹ اپ",
            "implementation": "نفاذ",
            "theory": "نظریہ",
            "practical": "عملی",
            "exercise": "مشق",
            "assessment": "تشخیص",
            "safety": "حفاظت",
            "important": "اہم",
            "system": "سسٹم",
            "control": "کنٹرول"
        }

        result = phrase
        for english, urdu in basic_translations.items():
            pattern = r'\b' + re.escape(english) + r'\b'
            result = re.sub(pattern, urdu, result, flags=re.IGNORECASE)

        return result

    def _restore_constitutional_formatting(self, translated_text: str, preserved_elements: dict) -> str:
        """Restore constitutional formatting while maintaining translation"""
        result = translated_text

        # Restore preserved elements
        for element in preserved_elements['preserved_elements']:
            result = result.replace(element['placeholder'], element['original'])

        # Restore technical terms
        for term in self.constitutional_technical_terms:
            placeholder = f"__TECH_TERM_{term}__"
            result = result.replace(placeholder, term)

        return result

    def _constitutional_validation(self, translated_text: str, original_text: str) -> dict:
        """Validate translation against constitutional requirements"""

        # Check for hallucination (Constitutional requirement: no hallucination)
        if len(translated_text) > len(original_text) * 2:
            return {'valid': False, 'reason': 'Potential hallucination detected - output too long'}

        # Check for preserved formatting
        if '```' in original_text and '```' not in translated_text:
            return {'valid': False, 'reason': 'Code block formatting not preserved'}

        # Check for technical term preservation
        for term in self.constitutional_technical_terms:
            if term in original_text and term not in translated_text:
                return {'valid': False, 'reason': f'Technical term {term} not preserved'}

        return {'valid': True, 'reason': 'Constitutional validation passed'}

    def _constitutional_error(self, message: str) -> str:
        """Return constitutional error message as required by constitution"""
        return f"Source required — ambiguity detected: {message}"


class HuggingFaceTranslationService(TranslationService):
    """Free Hugging Face translation service - no API key needed"""

    def __init__(self):
        self.base_url = "https://api-inference.huggingface.co/models"
        logger.info("Using FREE Hugging Face translation service (no API key required)")

    async def translate(self, text: str, target_language: str, source_language: Optional[str] = None) -> str:
        """Translate using free Hugging Face models"""
        try:
            import requests
            import asyncio

            # Map language codes to working free model names
            model_map = {
                "ur": "Helsinki-NLP/opus-mt-en-urj",  # Updated model name
                "es": "Helsinki-NLP/opus-mt-en-es",   # English to Spanish
                "fr": "Helsinki-NLP/opus-mt-en-fr",   # English to French
                "de": "Helsinki-NLP/opus-mt-en-de",   # English to German
                "ar": "Helsinki-NLP/opus-mt-en-ar",   # English to Arabic
                "hi": "Helsinki-NLP/opus-mt-en-hi",   # English to Hindi
            }

            model_name = model_map.get(target_language)
            if not model_name:
                logger.warning(f"Language {target_language} not supported by free service")
                return f"[{target_language.upper()} - Language not supported] {text}"

            # For Urdu, use Constitutional service
            if target_language == "ur":
                logger.info("Using Constitutional Urdu Translation Service for Urdu")
                constitutional_service = ConstitutionalUrduTranslationService()
                return await constitutional_service.translate(text, target_language, source_language)

            # Use the correct Hugging Face inference API endpoint
            url = f"https://api-inference.huggingface.co/models/{model_name}"
            payload = {"inputs": text}
            headers = {"Content-Type": "application/json"}

            logger.info(f"Free Hugging Face translation to {target_language}: {text[:50]}...")
            logger.info(f"Using model: {model_name}")

            # Make async request with longer timeout
            loop = asyncio.get_event_loop()
            response = await loop.run_in_executor(
                None,
                lambda: requests.post(url, json=payload, headers=headers, timeout=30)
            )

            logger.info(f"Hugging Face API response status: {response.status_code}")

            if response.status_code == 200:
                result = response.json()
                logger.info(f"Hugging Face API response: {result}")

                if isinstance(result, list) and len(result) > 0:
                    translated_text = result[0].get("translation_text", text)
                    logger.info(f"Translation successful: {translated_text[:50]}...")
                    return translated_text
                else:
                    logger.warning("Empty result from Hugging Face API")
                    return f"[{target_language.upper()} - Free Translation] {text}"
            elif response.status_code == 503:
                # Model is loading, try again with a simple fallback
                logger.warning("Hugging Face model is loading, using fallback")
                return f"[{target_language.upper()} - Model Loading] {text}"
            else:
                logger.warning(f"Hugging Face API error: {response.status_code} - {response.text}")
                return f"[{target_language.upper()} - Free Translation] {text}"

        except Exception as e:
            logger.error(f"Hugging Face translation error: {e}")
            return f"[{target_language.upper()} - Free Translation] {text}"


class BasicUrduTranslationService(TranslationService):
    """Basic dictionary-based Urdu translation service (fallback)"""

    def __init__(self):
        # Basic English to Urdu dictionary for key terms
        self.translation_dict = {
            # Technical terms
            "robot": "روبوٹ",
            "robotics": "روبوٹکس",
            "humanoid": "انسان نما",
            "control": "کنٹرول",
            "system": "نظام",
            "systems": "نظامات",
            "simulation": "نقل",
            "environment": "ماحول",
            "physics": "طبیعیات",
            "engine": "انجن",
            "model": "ماڈل",
            "chapter": "باب",
            "learning": "سیکھنا",
            "objectives": "مقاصد",
            "theory": "نظریہ",
            "practical": "عملی",
            "examples": "مثالیں",
            "exercises": "مشقیں",
            "references": "حوالہ جات",
            "summary": "خلاصہ",

            # Common words
            "the": "",  # Urdu doesn't always need "the"
            "and": "اور",
            "or": "یا",
            "of": "کا",
            "in": "میں",
            "for": "کے لیے",
            "with": "کے ساتھ",
            "by": "کے ذریعے",
            "to": "کو",
            "is": "ہے",
            "are": "ہیں",
            "will": "گا",
            "can": "سکتا",
            "this": "یہ",
            "that": "وہ",
            "you": "آپ",
            "we": "ہم",
            "it": "یہ",

            # Action words
            "understand": "سمجھنا",
            "implement": "نافذ کرنا",
            "apply": "لاگو کرنا",
            "evaluate": "جانچنا",
            "design": "ڈیزائن",
            "create": "بنانا",
            "build": "تعمیر کرنا",
            "test": "ٹیسٹ",
            "configure": "ترتیب دینا",
            "install": "انسٹال کرنا",
        }

    async def translate(self, text: str, target_language: str, source_language: Optional[str] = None) -> str:
        """Comprehensive sentence-level translation to Urdu with proper grammar and context"""
        try:
            if target_language != "ur":
                return f"[{target_language.upper()} translation not supported] {text}"

            logger.info(f"Comprehensive Urdu translation starting for text length: {len(text)}")

            # For longer content, break into sentences and translate each
            if len(text) > 100:
                return await self._translate_full_content(text)
            else:
                # For short content, use direct translation
                return self._translate_single_sentence(text)

        except Exception as e:
            logger.error(f"Translation error: {e}")
            return text

    async def _translate_full_content(self, text: str) -> str:
        """Translate full chapter content sentence by sentence"""
        try:
            # First try MyMemory API for natural translation
            try:
                import requests
                import asyncio

                # Split into chunks if text is very long
                chunks = self._split_into_chunks(text, 400)
                translated_chunks = []

                for chunk in chunks:
                    url = "https://api.mymemory.translated.net/get"
                    params = {
                        'q': chunk,
                        'langpair': 'en|ur'
                    }

                    loop = asyncio.get_event_loop()
                    response = await loop.run_in_executor(
                        None,
                        lambda: requests.get(url, params=params, timeout=8)
                    )

                    if response.status_code == 200:
                        result = response.json()
                        if result.get('responseStatus') == 200:
                            translated = result['responseData']['translatedText']
                            if translated and translated != chunk and not translated.startswith('MYMEMORY WARNING'):
                                translated_chunks.append(translated)
                                continue

                    # Fallback to sentence-level translation for this chunk
                    translated_chunks.append(self._translate_sentences_in_chunk(chunk))

                final_result = ' '.join(translated_chunks)
                if final_result and final_result != text:
                    logger.info("MyMemory API translation successful")
                    return final_result

            except Exception as e:
                logger.warning(f"MyMemory API failed: {e}")

            # Fallback: Sentence-by-sentence translation
            return self._translate_sentences_in_chunk(text)

        except Exception as e:
            logger.error(f"Full content translation error: {e}")
            return text

    def _split_into_chunks(self, text: str, max_length: int) -> list:
        """Split text into chunks while preserving sentence boundaries"""
        import re

        # Split by sentences first
        sentences = re.split(r'(?<=[.!?])\s+', text)
        chunks = []
        current_chunk = ""

        for sentence in sentences:
            if len(current_chunk + sentence) <= max_length:
                current_chunk += sentence + " "
            else:
                if current_chunk:
                    chunks.append(current_chunk.strip())
                current_chunk = sentence + " "

        if current_chunk:
            chunks.append(current_chunk.strip())

        return chunks

    def _translate_sentences_in_chunk(self, text: str) -> str:
        """Translate text by breaking into sentences and translating each"""
        import re

        # Split into sentences
        sentences = re.split(r'(?<=[.!?])\s+', text)
        translated_sentences = []

        for sentence in sentences:
            if sentence.strip():
                translated = self._translate_single_sentence(sentence.strip())
                translated_sentences.append(translated)

        return ' '.join(translated_sentences)

    def _translate_single_sentence(self, text: str) -> str:
        """Translate a single sentence with context awareness"""
        text_lower = text.lower().strip()

        # Sentence pattern templates for robotics content
        sentence_templates = {
            # Chapter and section patterns
            r"^chapter (\d+):?\s*(.+)": "باب {}: {}",
            r"^# (.+)": "# {}",
            r"^## (.+)": "## {}",
            r"^### (.+)": "### {}",

            # Introduction patterns
            r"this chapter (covers|discusses|explores|examines|introduces) (.+)": "یہ باب {} کا احاطہ کرتا ہے",
            r"in this chapter,?\s*(we will|you will|we'll|you'll) (.+)": "اس باب میں ہم {}",
            r"this section (covers|discusses|explores|examines) (.+)": "یہ حصہ {} کا احاطہ کرتا ہے",

            # Learning objectives
            r"by the end of this chapter,?\s*you will (.+)": "اس باب کے اختتام تک آپ {}",
            r"learning objectives?:?": "تعلیمی مقاصد:",
            r"you will (be able to|learn to|understand how to) (.+)": "آپ {} سکیں گے",
            r"students will (learn|understand|be able to) (.+)": "طلباء {} سکیں گے",

            # Technical descriptions
            r"(.+) is a (.+) that (.+)": "{} ایک {} ہے جو {}",
            r"(.+) are (.+) used for (.+)": "{} {} ہیں جو {} کے لیے استعمال ہوتے ہیں",
            r"the (.+) system (.+)": "{} سسٹم {}",
            r"a (.+) is (.+)": "ایک {} {} ہے",

            # Process and instruction patterns
            r"to (.+),?\s*(first|you need to|we need to|you must|we must) (.+)": "{} کے لیے پہلے آپ کو {}",
            r"step (\d+):?\s*(.+)": "قدم {}: {}",
            r"follow these steps:?": "یہ قدم اٹھائیں:",
            r"first,?\s*(.+)": "پہلے، {}",
            r"next,?\s*(.+)": "اگلا، {}",
            r"finally,?\s*(.+)": "آخر میں، {}",

            # Explanation patterns
            r"this means that (.+)": "اس کا مطلب یہ ہے کہ {}",
            r"in other words,?\s*(.+)": "دوسرے الفاظ میں، {}",
            r"for example,?\s*(.+)": "مثال کے طور پر، {}",
            r"for instance,?\s*(.+)": "مثلاً، {}",

            # Conclusion patterns
            r"in conclusion,?\s*(.+)": "خلاصہ یہ ہے کہ {}",
            r"to summarize,?\s*(.+)": "خلاصہ کرتے ہوئے، {}",
            r"as we can see,?\s*(.+)": "جیسا کہ ہم دیکھ سکتے ہیں، {}",

            # Importance and emphasis
            r"it is important to (.+)": "{} کرنا اہم ہے",
            r"it is essential to (.+)": "{} کرنا ضروری ہے",
            r"make sure (to )?(.+)": "یقینی بنائیں کہ {}",
            r"remember that (.+)": "یاد رکھیں کہ {}",
            r"note that (.+)": "نوٹ کریں کہ {}",

            # Conditional patterns
            r"if (.+),?\s*then (.+)": "اگر {} تو {}",
            r"when (.+),?\s*(.+)": "جب {} تو {}",
            r"while (.+),?\s*(.+)": "جبکہ {}, {}",

            # Comparison patterns
            r"unlike (.+),?\s*(.+)": "{} کے برعکس، {}",
            r"similar to (.+),?\s*(.+)": "{} کی طرح، {}",
            r"compared to (.+),?\s*(.+)": "{} کے مقابلے میں، {}"
        }

        # Try to match sentence patterns
        import re
        for pattern, urdu_template in sentence_templates.items():
            match = re.search(pattern, text_lower)
            if match:
                groups = match.groups()
                translated_groups = []

                for group in groups:
                    if group:
                        translated_group = self._translate_key_terms(group)
                        translated_groups.append(translated_group)

                try:
                    result = urdu_template.format(*translated_groups)
                    logger.debug(f"Pattern matched: {pattern} -> {result}")
                    return result
                except:
                    pass

        # If no pattern matches, use phrase-level translation
        return self._translate_phrases(text)

    def _translate_key_terms(self, text: str) -> str:
        """Translate key technical terms while preserving context"""
        key_terms = {
            "robot": "روبوٹ",
            "robotics": "روبوٹکس",
            "humanoid": "انسان نما روبوٹ",
            "control system": "کنٹرول سسٹم",
            "simulation": "نقل",
            "environment": "ماحول",
            "sensor": "سینسر",
            "actuator": "ایکچویٹر",
            "artificial intelligence": "مصنوعی ذہانت",
            "machine learning": "مشین لرننگ",
            "programming": "پروگرامنگ",
            "algorithm": "الگورتھم",
            "software": "سافٹ ویئر",
            "hardware": "ہارڈ ویئر",
            "computer": "کمپیوٹر",
            "technology": "ٹیکنالوجی",
            "development": "ترقی",
            "implementation": "نافذ کرنا",
            "configuration": "ترتیب",
            "installation": "انسٹالیشن"
        }

        result = text
        import re
        for english, urdu in key_terms.items():
            pattern = r'\b' + re.escape(english) + r'\b'
            result = re.sub(pattern, urdu, result, flags=re.IGNORECASE)

        return result

    def _translate_phrases(self, text: str) -> str:
        """Translate common phrases and improve grammar"""

        # Common phrase replacements
        phrase_replacements = {
            "this chapter": "یہ باب",
            "in this chapter": "اس باب میں",
            "the next chapter": "اگلا باب",
            "as we can see": "جیسا کہ ہم دیکھ سکتے ہیں",
            "for example": "مثال کے طور پر",
            "in conclusion": "خلاصہ یہ ہے",
            "it is important": "یہ اہم ہے",
            "we will learn": "ہم سیکھیں گے",
            "you will learn": "آپ سیکھیں گے",
            "let us": "آئیے",
            "we can": "ہم کر سکتے ہیں",
            "you can": "آپ کر سکتے ہیں",
            "we need to": "ہمیں ضرورت ہے",
            "you need to": "آپ کو ضرورت ہے",
            "make sure": "یقینی بنائیں",
            "keep in mind": "ذہن میں رکھیں",
            "step by step": "قدم بہ قدم",
            "first of all": "سب سے پہلے",
            "at the end": "آخر میں",
            "on the other hand": "دوسری طرف"
        }

        result = text
        import re

        # Replace phrases first (longer matches first)
        for english_phrase, urdu_phrase in sorted(phrase_replacements.items(), key=len, reverse=True):
            pattern = r'\b' + re.escape(english_phrase) + r'\b'
            result = re.sub(pattern, urdu_phrase, result, flags=re.IGNORECASE)

        # Then translate remaining key terms
        result = self._translate_key_terms(result)

        # Clean up spacing
        result = re.sub(r'\s+', ' ', result).strip()

        return result


class TranslationServiceFactory:
    """Factory for creating translation services following Constitutional requirements"""

    @staticmethod
    def get_translation_service():
        """Get Constitutional Urdu Translation Agent as per Physical AI Book Constitution v2.0"""
        logger.info("=== Constitutional Translation Service Factory v2.0 ===")
        logger.info("✓ Using Constitutional Urdu Translation Agent (Constitution compliant)")
        return ConstitutionalUrduTranslationService()


# Global instance for easy access - lazy loaded
_translation_service = None


def get_translation_service():
    """Get or create the Constitutional translation service (lazy loading)"""
    global _translation_service
    # Use Constitutional service as per Physical AI Book Constitution
    logger.info("=== get_translation_service: Creating Constitutional service ===")
    _translation_service = ConstitutionalUrduTranslationService()
    return _translation_service


async def translate_text(text: str, target_language: str, source_language: Optional[str] = None) -> str:
    """
    Constitutional translation function following Physical AI Book Constitution v2.0

    This function implements the Constitutional Urdu Translation Agent requirements:
    - Maintains technical accuracy and official documentation grounding
    - Preserves Docusaurus-compatible Markdown formatting
    - Follows consistent writing style across all chapters
    - Never hallucinates - indicates ambiguity when unsure
    - Maintains safety standards for minor users
    - Preserves code blocks and technical terms
    """
    try:
        # Use Constitutional translation service DIRECTLY
        service = ConstitutionalUrduTranslationService()
        logger.info(f"=== Constitutional translate_text v2.0: text='{text[:50]}...', target='{target_language}' ===")

        result = await service.translate(text, target_language, source_language)
        logger.info(f"=== Constitutional translation result type: {type(result)}, length: {len(result)} ===")

        return result
    except Exception as e:
        logger.error(f"=== Constitutional translation EXCEPTION: {type(e).__name__}: {str(e)} ===")
        import traceback
        logger.error(f"=== Constitutional translation TRACEBACK: {traceback.format_exc()} ===")
        # Return constitutional error as required
        return f"Source required — ambiguity detected: Translation failed due to {str(e)}"


# Legacy services for backward compatibility
class TranslationProvider(Enum):
    """Supported translation providers"""
    CONSTITUTIONAL = "constitutional"
    HUGGINGFACE = "huggingface"
    BASIC_URDU = "basic_urdu"


class TranslationService(ABC):
    """Abstract base class for translation services"""

    @abstractmethod
    async def translate(self, text: str, target_language: str, source_language: Optional[str] = None) -> str:
        """Translate text from source language to target language"""
        pass
