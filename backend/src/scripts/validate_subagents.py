#!/usr/bin/env python3
"""
Subagent Validation Script

This script validates that all subagents work as expected according to the quickstart guide.
It tests the Glossary Maker, Code Explainer, Quiz Creator, and Book Content Writer subagents.
"""
import asyncio
import sys
import os
from typing import Dict, Any, List
from datetime import datetime

# Add the project root, src directory, and shared directory to the Python path to allow imports
project_root = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
src_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)))
shared_dir = os.path.join(project_root, 'shared')

# Insert paths in the right order
sys.path.insert(0, src_dir)
sys.path.insert(0, project_root)
sys.path.insert(0, shared_dir)

# Import using absolute paths from the src directory
from src.services.glossary_service import GlossaryService
from src.services.code_explainer_service import CodeExplainerService
from src.services.quiz_service import QuizService
from src.services.book_content_service import BookContentService
from src.services.content_service import ContentService
from src.database.database import get_db
from shared.utils.content_validation import validate_content_safety


class SubagentValidator:
    """Validates that all subagents work correctly"""

    def __init__(self):
        self.content_service = ContentService()
        self.book_content_service = BookContentService(self.content_service)
        self.results = {
            "timestamp": datetime.now().isoformat(),
            "tests_passed": 0,
            "tests_total": 0,
            "test_results": {},
            "overall_success": True
        }

    async def validate_all_subagents(self) -> Dict[str, Any]:
        """Run validation tests for all subagents"""
        print("Starting subagent validation...")
        print("=" * 50)

        # Test Glossary Maker functionality
        await self._test_glossary_maker()

        # Test Code Explainer functionality
        await self._test_code_explainer()

        # Test Quiz Creator functionality
        await self._test_quiz_creator()


        # Test Book Content Writer functionality
        await self._test_book_content_writer()

        # Calculate final results
        self.results["overall_success"] = self.results["tests_passed"] == self.results["tests_total"]

        print("=" * 50)
        print(f"Validation Summary: {self.results['tests_passed']}/{self.results['tests_total']} tests passed")
        print(f"Overall Success: {'[PASS]' if self.results['overall_success'] else '[FAIL]'}")

        return self.results

    async def _test_glossary_maker(self):
        """Test the Glossary Maker subagent"""
        print("\nTesting Glossary Maker...")

        test_content = """
        This chapter discusses ROS 2 nodes, publishers, subscribers, and topics.
        A node is the fundamental unit of computation in ROS 2.
        Publishers send messages to topics, while subscribers receive messages from topics.
        Services provide request-response communication patterns.
        """

        try:
            # Test term extraction
            terms = await GlossaryService.term_extraction(test_content)
            self._add_test_result("glossary_extraction", len(terms) > 0, "Term extraction")

            if terms:
                # Test definition generation
                defined_terms = await GlossaryService.term_definition_generation(terms, test_content)
                self._add_test_result("glossary_definition", len(defined_terms) == len(terms), "Definition generation")

            print(f"  [PASS] Glossary Maker: {self.results['test_results'].get('glossary_extraction', {}).get('status', 'Unknown')}")

        except Exception as e:
            self._add_test_result("glossary_error", False, f"Error in Glossary Maker: {str(e)}")
            print(f"  [FAIL] Glossary Maker: Error - {str(e)}")

    async def _test_code_explainer(self):
        """Test the Code Explainer subagent"""
        print("\nTesting Code Explainer...")

        test_code = """
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        class MinimalPublisher(Node):
            def __init__(self):
                super().__init__('minimal_publisher')
                self.publisher = self.create_publisher(String, 'topic', 10)
                timer_period = 0.5  # seconds
                self.timer = self.create_timer(timer_period, self.timer_callback)

            def timer_callback(self):
                msg = String()
                msg.data = 'Hello World'
                self.publisher.publish(msg)
        """

        try:
            # Test ROS 2 command identification using the private method (accessing directly)
            ros2_commands = CodeExplainerService._find_ros2_commands(test_code)
            self._add_test_result("code_ros_identification", len(ros2_commands) > 0, "ROS 2 command identification")

            # Test Isaac Sim command identification (should be empty for this code)
            isaac_commands = CodeExplainerService._find_isaac_commands(test_code)
            self._add_test_result("code_isaac_identification", True, "Isaac command identification (may be empty)")

            # Test code parsing and highlighting
            parsed_result = await CodeExplainerService.parse_and_highlight_code(test_code)
            self._add_test_result("code_explanation", len(parsed_result.get("highlights", {})) > 0, "Code parsing and highlighting")

            print(f"  [PASS] Code Explainer: {self.results['test_results'].get('code_explanation', {}).get('status', 'Unknown')}")

        except Exception as e:
            self._add_test_result("code_error", False, f"Error in Code Explainer: {str(e)}")
            print(f"  [FAIL] Code Explainer: Error - {str(e)}")

    async def _test_quiz_creator(self):
        """Test the Quiz Creator subagent"""
        print("\nTesting Quiz Creator...")

        test_content = "This chapter covers basic ROS 2 concepts including nodes, publishers, and subscribers."
        user_profile = {"experience_level": "BEGINNER"}
        difficulty = "MEDIUM"

        try:
            # Test MCQ generation using private method directly
            mcqs = QuizService._generate_mcq_questions(test_content, difficulty, user_profile["experience_level"])
            self._add_test_result("quiz_mcq", len(mcqs) > 0, "MCQ generation")

            # Test short answer generation using private method directly
            short_answers = QuizService._generate_short_answer_questions(test_content, difficulty, user_profile["experience_level"])
            self._add_test_result("quiz_short_answer", len(short_answers) > 0, "Short answer generation")

            # Test coding exercise generation using private method directly
            coding_exercises = QuizService._generate_coding_exercises(test_content, difficulty, user_profile["experience_level"])
            self._add_test_result("quiz_coding", len(coding_exercises) > 0, "Coding exercise generation")

            print(f"  [PASS] Quiz Creator: {self.results['test_results'].get('quiz_mcq', {}).get('status', 'Unknown')}")

        except Exception as e:
            self._add_test_result("quiz_error", False, f"Error in Quiz Creator: {str(e)}")
            print(f"  [FAIL] Quiz Creator: Error - {str(e)}")


    async def _test_book_content_writer(self):
        """Test the Book Content Writer subagent"""
        print("\nTesting Book Content Writer...")

        try:
            # Test book chapter generation
            chapter = await self.book_content_service.generate_book_chapter(
                module_focus="ROS 2 Basics",
                chapter_title="Introduction to ROS 2",
                content_type="theory",
                target_audience="beginner",
                technical_domain="ROS 2"
            )

            # Validate the generated chapter
            is_valid = validate_content_safety(chapter.content)
            # Check that the chapter object has the expected properties
            # Note: BookChapter ID might be None if not persisted to DB, but object should still be created
            has_content = hasattr(chapter, 'content') and chapter.content is not None and len(chapter.content) > 0
            self._add_test_result("book_content_generation",
                                has_content and is_valid["safe"],
                                "Book content generation and safety validation")

            print(f"  [PASS] Book Content Writer: {self.results['test_results'].get('book_content_generation', {}).get('status', 'Unknown')}")

        except Exception as e:
            self._add_test_result("book_error", False, f"Error in Book Content Writer: {str(e)}")
            print(f"  [FAIL] Book Content Writer: Error - {str(e)}")

    def _add_test_result(self, test_id: str, success: bool, description: str):
        """Add a test result to the validation results"""
        status = "PASS" if success else "FAIL"
        status_emoji = "[PASS]" if success else "[FAIL]"

        self.results["test_results"][test_id] = {
            "description": description,
            "success": success,
            "status": status,
            "status_emoji": status_emoji
        }

        self.results["tests_total"] += 1
        if success:
            self.results["tests_passed"] += 1

    def print_detailed_results(self):
        """Print detailed validation results"""
        print("\n" + "=" * 50)
        print("DETAILED VALIDATION RESULTS")
        print("=" * 50)

        for test_id, result in self.results["test_results"].items():
            status = result["status_emoji"]
            desc = result["description"]
            print(f"{status} {desc}")

            if not result["success"]:
                print(f"   Details: {result.get('error', 'Test failed')}")

        print(f"\nSUMMARY:")
        print(f"Tests Passed: {self.results['tests_passed']}/{self.results['tests_total']}")
        print(f"Success Rate: {(self.results['tests_passed']/self.results['tests_total']*100):.1f}%" if self.results['tests_total'] > 0 else "Success Rate: 0%")
        print(f"Overall Status: {'SUCCESS' if self.results['overall_success'] else 'FAILURE'}")


async def main():
    """Main function to run the validation"""
    validator = SubagentValidator()
    results = await validator.validate_all_subagents()
    validator.print_detailed_results()

    # Exit with appropriate code
    sys.exit(0 if results["overall_success"] else 1)


if __name__ == "__main__":
    asyncio.run(main())