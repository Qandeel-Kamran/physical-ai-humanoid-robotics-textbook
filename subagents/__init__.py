"""
Main Subagents Controller for Physical AI & Humanoid Robotics Textbook
Coordinates all subagents for reusable intelligence
"""
from .textbook_analyzer import TextbookAnalyzerSubagent
from .content_personalizer import ContentPersonalizerSubagent
from .translator import TranslatorSubagent

class SubagentsController:
    """Main controller for managing all textbook subagents"""

    def __init__(self):
        self.analyzer = TextbookAnalyzerSubagent()
        self.personalizer = ContentPersonalizerSubagent()
        self.translator = TranslatorSubagent()

        self.subagents = {
            'analyzer': self.analyzer,
            'personalizer': self.personalizer,
            'translator': self.translator
        }

    async def process_content(self, content: str, operations: list, **kwargs):
        """
        Process content using specified operations
        operations: list of operations to perform (e.g., ['analyze', 'personalize', 'translate'])
        """
        result = {'original_content': content}

        # Perform operations in sequence
        current_content = content

        for operation in operations:
            if operation == 'analyze':
                analysis = await self.analyzer.analyze_chapter(current_content)
                result['analysis'] = analysis
            elif operation == 'personalize':
                user_profile = kwargs.get('user_profile', {})
                personalization = await self.personalizer.personalize_content(current_content, user_profile)
                result['personalization'] = personalization
                current_content = personalization['personalized_content']
            elif operation == 'translate':
                target_language = kwargs.get('target_language', 'ur')
                translation = await self.translator.translate_content(current_content, target_language)
                result['translation'] = translation
                current_content = translation['translated_content']

        result['final_content'] = current_content
        return result

    def get_available_subagents(self):
        """Get list of available subagents"""
        return list(self.subagents.keys())

    def get_subagent_description(self, subagent_name: str):
        """Get description of a specific subagent"""
        if subagent_name in self.subagents:
            return self.subagents[subagent_name].description
        return None

# Example usage
async def main():
    controller = SubagentsController()

    sample_content = """
    # Introduction to Physical AI

    Physical AI represents a paradigm shift from traditional artificial intelligence approaches, emphasizing the importance of physical embodiment in creating intelligent systems.

    Learning Outcomes:
    - Define Physical AI and distinguish it from traditional AI approaches
    - Describe the evolution of humanoid robotics from early concepts to current systems
    - Identify key challenges in developing human-like robotic systems
    """

    user_profile = {
        "software_experience": "beginner",
        "ai_ml_experience": "novice",
        "robotics_background": "none"
    }

    # Process content with multiple operations
    result = await controller.process_content(
        sample_content,
        operations=['analyze', 'personalize', 'translate'],
        user_profile=user_profile,
        target_language='ur'
    )

    print(f"Available subagents: {controller.get_available_subagents()}")
    print(f"Analysis: {result.get('analysis', {}).get('learning_outcomes', [])}")
    print(f"Personalization: {result.get('personalization', {}).get('experience_level', 'N/A')}")
    print(f"Translation status: {result.get('translation', {}).get('status', 'N/A')}")

if __name__ == "__main__":
    import asyncio
    asyncio.run(main())