"""
Content Personalizer Subagent
This subagent adapts textbook content based on user background and preferences
"""
import asyncio
from typing import Dict, Any, List
from enum import Enum

class ExperienceLevel(Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

class ContentPersonalizerSubagent:
    """A subagent for personalizing textbook content based on user profile"""

    def __init__(self):
        self.name = "ContentPersonalizer"
        self.description = "Adapts textbook content based on user background and preferences"

    async def personalize_content(self,
                                original_content: str,
                                user_profile: Dict[str, Any]) -> Dict[str, Any]:
        """Personalize content based on user profile"""

        # Determine user experience level
        experience_level = self._determine_experience_level(user_profile)

        # Apply personalization based on experience level
        personalized_content = self._apply_personalization(
            original_content,
            experience_level,
            user_profile
        )

        return {
            "original_content": original_content,
            "personalized_content": personalized_content,
            "experience_level": experience_level.value,
            "modifications": self._get_modifications(original_content, personalized_content)
        }

    def _determine_experience_level(self, user_profile: Dict[str, Any]) -> ExperienceLevel:
        """Determine user experience level from profile"""
        # Default to intermediate
        level_scores = {
            ExperienceLevel.BEGINNER: 0,
            ExperienceLevel.INTERMEDIATE: 0,
            ExperienceLevel.ADVANCED: 0
        }

        # Score based on user background
        if user_profile.get('software_experience', '').lower() in ['beginner', 'novice', 'entry-level']:
            level_scores[ExperienceLevel.BEGINNER] += 2
        elif user_profile.get('software_experience', '').lower() in ['intermediate', 'mid-level']:
            level_scores[ExperienceLevel.INTERMEDIATE] += 2
        elif user_profile.get('software_experience', '').lower() in ['advanced', 'expert', 'senior']:
            level_scores[ExperienceLevel.ADVANCED] += 2

        if user_profile.get('ai_ml_experience', '').lower() in ['beginner', 'novice']:
            level_scores[ExperienceLevel.BEGINNER] += 1
        elif user_profile.get('ai_ml_experience', '').lower() in ['intermediate']:
            level_scores[ExperienceLevel.INTERMEDIATE] += 1
        elif user_profile.get('ai_ml_experience', '').lower() in ['advanced', 'expert']:
            level_scores[ExperienceLevel.ADVANCED] += 1

        # Return the level with highest score
        return max(level_scores, key=level_scores.get)

    def _apply_personalization(self, content: str, level: ExperienceLevel, user_profile: Dict[str, Any]) -> str:
        """Apply personalization to content based on experience level"""
        if level == ExperienceLevel.BEGINNER:
            return self._personalize_for_beginner(content, user_profile)
        elif level == ExperienceLevel.ADVANCED:
            return self._personalize_for_advanced(content, user_profile)
        else:  # INTERMEDIATE
            return self._personalize_for_intermediate(content, user_profile)

    def _personalize_for_beginner(self, content: str, user_profile: Dict[str, Any]) -> str:
        """Add explanations and context for beginners"""
        import re

        # Add more explanations for technical terms
        content = re.sub(r'\b(ROS|AI|ML|SLAM|VSLAM|IMU|LIDAR|SLAM)\b',
                        r'**\1** (*\1: explanation of this term for beginners*)', content)

        # Add more context and background
        content = content.replace('Physical AI', 'Physical AI (AI systems that interact with the physical world through sensors and actuators)')
        content = content.replace('Humanoid Robotics', 'Humanoid Robotics (robots designed to look and act like humans)')

        # Add more examples and analogies
        content += "\n\n> **Beginner Tip**: Think of this concept like [simple analogy related to everyday experiences]."

        return content

    def _personalize_for_intermediate(self, content: str, user_profile: Dict[str, Any]) -> str:
        """Standard content for intermediate users"""
        # For intermediate users, provide balanced content
        return content

    def _personalize_for_advanced(self, content: str, user_profile: Dict[str, Any]) -> str:
        """Add depth and advanced concepts for advanced users"""
        import re

        # Add more technical depth
        content += "\n\n> **Advanced Note**: This concept connects to [advanced topic/implementation detail]. For more details, see [research paper or advanced resource]."

        # Add references to more complex implementations
        content = re.sub(r'(control system)', r'control system (e.g., model-predictive control, adaptive control)', content)
        content = re.sub(r'(algorithm)', r'algorithm (with considerations for computational complexity, real-time constraints)', content)

        return content

    def _get_modifications(self, original: str, personalized: str) -> List[str]:
        """Get list of modifications made to content"""
        modifications = []

        if len(personalized) > len(original) * 1.1:  # If content is significantly longer
            modifications.append("Added explanations and context")
        elif len(personalized) < len(original) * 0.9:  # If content is significantly shorter
            modifications.append("Simplified content for readability")

        if "**Beginner Tip**" in personalized:
            modifications.append("Added beginner tips and analogies")
        if "**Advanced Note**" in personalized:
            modifications.append("Added advanced notes and depth")

        return modifications

# Example usage
async def main():
    personalizer = ContentPersonalizerSubagent()

    sample_content = """
    Physical AI represents a paradigm shift from traditional artificial intelligence approaches.
    Humanoid Robotics involves creating robots that mimic human form and behavior.
    ROS (Robot Operating System) is a middleware for robot control.
    """

    user_profile = {
        "software_experience": "beginner",
        "ai_ml_experience": "novice",
        "robotics_background": "none"
    }

    result = await personalizer.personalize_content(sample_content, user_profile)
    print(f"Experience Level: {result['experience_level']}")
    print(f"Modifications: {result['modifications']}")
    print(f"Personalized Content: {result['personalized_content']}")

if __name__ == "__main__":
    asyncio.run(main())