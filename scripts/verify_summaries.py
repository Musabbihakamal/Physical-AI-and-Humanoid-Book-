#!/usr/bin/env python3
"""
Script to verify that chapters have end-of-chapter summaries.
"""

import os
from pathlib import Path

def verify_end_of_chapter_summaries():
    """Verify that key chapters have end-of-chapter summaries."""
    frontend_docs = Path("frontend/docs")

    # Check the specific files we updated
    updated_files = [
        "frontend/docs/physical-ai/module-1/ch1-1-ros2-fundamentals-architecture.md",
        "frontend/docs/physical-ai/module-1/ch1-2-python-cpp-integration-ai-agents.md",
        "frontend/docs/physical-ai/module-1/ch1-3-robot-description-control.md",
        "frontend/docs/physical-ai/module-1/ch1-4-advanced-ros2-concepts.md",
        "frontend/docs/physical-ai/module-2/ch2-1-gazebo-harmonic-next-generation-simulation.md",
        "frontend/docs/physical-ai/module-1-ros2.md",
        "frontend/docs/physical-ai/module-2-digital-twin.md",
        "frontend/docs/physical-ai/module-3/ch1-nvidia-isaac-sim-foundations.md"
    ]

    verification_results = []

    for file_path in updated_files:
        full_path = Path(file_path)
        if full_path.exists():
            with open(full_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Check if there's a summary section near the end
            lines = content.split('\n')
            summary_positions = []
            for i, line in enumerate(lines):
                if line.strip() == '## Summary':
                    summary_positions.append(i)

            # Check if there's a summary after the main content (not just at the beginning)
            has_end_summary = False
            for pos in summary_positions:
                # Look at content after the summary to see if it's followed by references/quiz/exercises
                content_after = '\n'.join(lines[pos+1:pos+30])  # Check next 30 lines after summary
                if any(keyword in content_after.lower() for keyword in ['references', 'quiz', 'exercises']):
                    has_end_summary = True
                    break

            verification_results.append((file_path, has_end_summary))
        else:
            verification_results.append((file_path, False))

    return verification_results

def main():
    results = verify_end_of_chapter_summaries()

    print("Verification Results for End-of-Chapter Summaries:")
    print("="*50)

    all_good = True
    for file_path, has_summary in results:
        status = "[OK]" if has_summary else "[MISSING]"
        print(f"{status} {file_path}")
        if not has_summary:
            all_good = False

    print("="*50)
    if all_good:
        print("[OK] All checked chapters have proper end-of-chapter summaries!")
    else:
        print("[ERROR] Some chapters are missing end-of-chapter summaries.")

if __name__ == "__main__":
    main()