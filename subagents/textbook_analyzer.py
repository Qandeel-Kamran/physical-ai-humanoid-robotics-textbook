"""
Textbook Content Analyzer Subagent
This subagent analyzes textbook content and extracts key information
"""
import asyncio
from typing import Dict, List, Any
import re

class TextbookAnalyzerSubagent:
    """A subagent for analyzing textbook content and extracting structured information"""

    def __init__(self):
        self.name = "TextbookAnalyzer"
        self.description = "Analyzes textbook content and extracts key information like learning outcomes, concepts, and prerequisites"

    async def analyze_chapter(self, chapter_content: str) -> Dict[str, Any]:
        """Analyze a chapter and extract key information"""
        analysis = {
            "title": self._extract_title(chapter_content),
            "learning_outcomes": self._extract_learning_outcomes(chapter_content),
            "key_concepts": self._extract_key_concepts(chapter_content),
            "prerequisites": self._extract_prerequisites(chapter_content),
            "word_count": len(chapter_content.split()),
            "complexity_score": self._calculate_complexity(chapter_content),
            "summary": self._generate_summary(chapter_content)
        }

        return analysis

    def _extract_title(self, content: str) -> str:
        """Extract chapter title from content"""
        # Look for markdown title or first heading
        lines = content.split('\n')
        for line in lines[:10]:  # Check first 10 lines
            if line.strip().startswith('# '):
                return line.strip('# ').strip()
            elif line.strip().startswith('title:'):
                return line.strip('title:').strip()
        return "Untitled Chapter"

    def _extract_learning_outcomes(self, content: str) -> List[str]:
        """Extract learning outcomes from content"""
        outcomes = []

        # Look for common patterns
        patterns = [
            r'learning[ _-]?outcomes?:?\s*\n((?:\s*[-*]\s*[^\n]+\n?)+)',
            r'objectives?:?\s*\n((?:\s*[-*]\s*[^\n]+\n?)+)',
            r'what[ \w]*will[ \w]*learn[ \w]*:?\s*\n((?:\s*[-*]\s*[^\n]+\n?)+)',
        ]

        for pattern in patterns:
            matches = re.findall(pattern, content, re.IGNORECASE)
            for match in matches:
                items = re.findall(r'[-*]\s*([^\n]+)', match)
                outcomes.extend([item.strip() for item in items if item.strip()])

        return outcomes[:5]  # Return max 5 outcomes

    def _extract_key_concepts(self, content: str) -> List[str]:
        """Extract key concepts from content"""
        concepts = []

        # Look for bolded text, technical terms, etc.
        bold_patterns = [
            r'\*\*([^\*]+)\*\*',  # **text**
            r'__([^\_]+)__'       # __text__
        ]

        for pattern in bold_patterns:
            matches = re.findall(pattern, content)
            concepts.extend([match.strip() for match in matches if len(match.strip()) > 3])

        # Remove duplicates while preserving order
        unique_concepts = []
        for concept in concepts:
            if concept.lower() not in [uc.lower() for uc in unique_concepts] and concept not in unique_concepts:
                unique_concepts.append(concept)

        return unique_concepts[:10]  # Return max 10 concepts

    def _extract_prerequisites(self, content: str) -> List[str]:
        """Extract prerequisites from content"""
        prerequisites = []

        patterns = [
            r'prerequisites?:?\s*\n((?:\s*[-*]\s*[^\n]+\n?)+)',
            r'before[ \w]*reading[ \w]*:?\s*\n((?:\s*[-*]\s*[^\n]+\n?)+)',
            r'assumes[ \w]*knowledge[ \w]*of:?\s*\n((?:\s*[-*]\s*[^\n]+\n?)+)',
        ]

        for pattern in patterns:
            matches = re.findall(pattern, content, re.IGNORECASE)
            for match in matches:
                items = re.findall(r'[-*]\s*([^\n]+)', match)
                prerequisites.extend([item.strip() for item in items if item.strip()])

        return prerequisites

    def _calculate_complexity(self, content: str) -> float:
        """Calculate a complexity score based on various factors"""
        words = content.split()
        avg_word_length = sum(len(word) for word in words) / len(words) if words else 0

        sentences = re.split(r'[.!?]+', content)
        avg_sentence_length = sum(len(sentence.split()) for sentence in sentences) / len(sentences) if sentences else 0

        # Complexity score: higher for longer words and sentences
        complexity = (avg_word_length * 0.3) + (avg_sentence_length * 0.1)

        # Normalize to 0-1 scale
        return min(1.0, complexity / 10.0)

    def _generate_summary(self, content: str) -> str:
        """Generate a brief summary of the content"""
        # Simple approach: take the first few sentences
        sentences = re.split(r'[.!?]+', content)
        summary_sentences = []
        char_count = 0

        for sentence in sentences:
            if char_count < 200:  # Limit to ~200 characters
                summary_sentences.append(sentence.strip())
                char_count += len(sentence)
            else:
                break

        return '. '.join(summary_sentences).strip() + '.'

# Example usage
async def main():
    analyzer = TextbookAnalyzerSubagent()

    sample_content = """
    # Introduction to Physical AI

    Physical AI represents a paradigm shift from traditional artificial intelligence approaches, emphasizing the importance of physical embodiment in creating intelligent systems.

    Learning Outcomes:
    - Define Physical AI and distinguish it from traditional AI approaches
    - Describe the evolution of humanoid robotics from early concepts to current systems
    - Identify key challenges in developing human-like robotic systems

    Before reading this chapter, students should have a basic understanding of robotics and artificial intelligence concepts.
    """

    analysis = await analyzer.analyze_chapter(sample_content)
    print(f"Analysis: {analysis}")

if __name__ == "__main__":
    asyncio.run(main())