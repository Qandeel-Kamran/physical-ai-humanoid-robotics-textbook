"""
Translator Subagent
This subagent handles translation of textbook content to different languages
"""
import asyncio
from typing import Dict, Any, List
import re

class TranslatorSubagent:
    """A subagent for translating textbook content to different languages"""

    def __init__(self):
        self.name = "Translator"
        self.description = "Translates textbook content to different languages with cultural adaptation"

        # Basic Urdu translation dictionary for technical terms
        self.urdu_translations = {
            # Core concepts
            "Physical AI": "جسمانی مصنوعی ذہانت",
            "Humanoid Robotics": "ہیومنوائڈ روبوٹس",
            "Artificial Intelligence": "مصنوعی ذہانت",
            "Robotics": "روبوٹکس",
            "Embodied Intelligence": "جسمانی ذہانت",

            # Technical terms
            "ROS": "روبوٹ آپریٹنگ سسٹم",
            "Robot Operating System": "روبوٹ آپریٹنگ سسٹم",
            "SLAM": "محل وقوع کا تعین اور نقشہ سازی",
            "VSLAM": "وژوئل SLAM",
            "IMU": "انرٹیل میزورمینٹ یونٹ",
            "LIDAR": "لائیزر انعکاس کا تجزیہ",
            "Computer Vision": "کمپیوٹر وژن",
            "Machine Learning": "مشین لرننگ",
            "Deep Learning": "گہری سیکھ",
            "Neural Network": "نیورل نیٹ ورک",
            "Control System": "کنٹرول سسٹم",
            "Actuator": "ایکچویٹر",
            "Sensor": "سینسر",
            "Algorithm": "الگورتھم",
            "Perception": "ادراک",
            "Manipulation": "ہاتھ سے کام لینا",
            "Locomotion": "چلنے کی صلاحیت",
            "Bipedal": "دو پاؤں والے",
            "Kinematics": "کنیمیٹکس",
            "Dynamics": "ڈائنا مکس",

            # Common words
            "and": "اور",
            "the": "کا/کی",
            "is": "ہے",
            "are": "ہیں",
            "to": "کو",
            "in": "میں",
            "on": "پر",
            "for": "کے لیے",
            "with": "کے ساتھ",
            "by": "کے ذریعے",
            "this": "یہ",
            "that": "وہ",
            "these": "یہ",
            "those": "وہ",
            "a": "ایک",
            "an": "ایک",
            "of": "کا",
            "as": "کے طور پر",
            "or": "یا",
            "but": "لیکن",
            "if": "اگر",
            "then": "پھر",
            "when": "جب",
            "where": "کہاں",
            "how": "کیسے",
            "what": "کیا",
            "why": "کیوں",
            "who": "کون",
            "which": "جس کا",
        }

    async def translate_content(self, content: str, target_language: str = "ur") -> Dict[str, Any]:
        """Translate content to target language"""
        if target_language.lower() == "ur" or target_language.lower() == "urdu":
            translated_content = self._translate_to_urdu(content)
        else:
            # For other languages, return original content with warning
            return {
                "original_content": content,
                "translated_content": content,
                "target_language": target_language,
                "status": "unsupported_language",
                "message": f"Language {target_language} not supported yet"
            }

        return {
            "original_content": content,
            "translated_content": translated_content,
            "target_language": target_language,
            "status": "success",
            "message": "Translation completed successfully"
        }

    def _translate_to_urdu(self, content: str) -> str:
        """Translate content to Urdu"""
        # First, try to translate technical terms and phrases
        translated = content

        # Sort keys by length (descending) to avoid partial replacements
        sorted_terms = sorted(self.urdu_translations.keys(), key=len, reverse=True)

        for term in sorted_terms:
            # Use word boundaries to avoid partial matches
            pattern = r'\b' + re.escape(term) + r'\b'
            translated = re.sub(pattern, self.urdu_translations[term], translated, flags=re.IGNORECASE)

        # For a more sophisticated implementation, we would use an actual translation API
        # This is a basic implementation using the dictionary

        # Handle some common sentence structures
        # This is a very basic approach - in reality, you'd want to use a proper NLP library
        # or translation API for better results

        return translated

    def _translate_paragraphs(self, content: str) -> str:
        """Translate content paragraph by paragraph"""
        paragraphs = content.split('\n\n')
        translated_paragraphs = []

        for paragraph in paragraphs:
            # Skip if it's a markdown header or code block
            if paragraph.strip().startswith('#') or paragraph.strip().startswith('```'):
                translated_paragraphs.append(paragraph)
            else:
                translated_paragraphs.append(self._translate_to_urdu(paragraph))

        return '\n\n'.join(translated_paragraphs)

    def get_supported_languages(self) -> List[str]:
        """Get list of supported languages"""
        return ["ur", "urdu"]  # Currently only Urdu is supported

# Example usage
async def main():
    translator = TranslatorSubagent()

    sample_content = """
    Physical AI represents a paradigm shift from traditional artificial intelligence approaches.
    Humanoid Robotics involves creating robots that mimic human form and behavior.
    ROS (Robot Operating System) is a middleware for robot control.
    """

    result = await translator.translate_content(sample_content, "ur")
    print(f"Status: {result['status']}")
    print(f"Original: {result['original_content']}")
    print(f"Translated: {result['translated_content']}")
    print(f"Supported languages: {translator.get_supported_languages()}")

if __name__ == "__main__":
    asyncio.run(main())