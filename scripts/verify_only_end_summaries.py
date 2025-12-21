#!/usr/bin/env python3
"""
Script to verify that chapters have only end-of-chapter summaries (no introductory summaries).
"""

import os
from pathlib import Path

def verify_only_end_summaries():
    """Verify that chapters have only end-of-chapter summaries, not introductory ones."""
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

            lines = content.split('\n')

            # Check if there's a summary in the first 20 lines (introductory summary)
            has_intro_summary = False
            for i, line in enumerate(lines[:20]):  # Check first 20 lines
                if line.strip() == '## Summary':
                    has_intro_summary = True
                    break

            # Check if there's a summary near the end (after main content like exercises/quiz/references)
            has_end_summary = False
            content_lower = content.lower()
            if any(keyword in content_lower for keyword in ['exercises', 'quiz', 'references']):
                # If the file has exercises/quiz/references, look for summary before these
                summary_positions = []
                for i, line in enumerate(lines):
                    if line.strip() == '## Summary':
                        summary_positions.append(i)

                for pos in summary_positions:
                    # Check if this summary comes after major content sections
                    content_after_summary = '\n'.join(lines[pos+1:pos+20])
                    if any(keyword in content_after_summary.lower() for keyword in ['exercises', 'quiz', 'references']):
                        has_end_summary = True
                        break

            verification_results.append((file_path, not has_intro_summary, has_end_summary))
        else:
            verification_results.append((file_path, False, False))

    return verification_results

def main():
    results = verify_only_end_summaries()

    print("Verification Results for Summary Placement:")
    print("="*60)
    print("Format: [OK/ISSUE] File Path - (No Intro Summary, Has End Summary)")
    print("="*60)

    all_good = True
    for file_path, no_intro, has_end in results:
        status = "[OK]" if (no_intro and has_end) else "[ISSUE]"
        print(f"{status} {file_path} - ({'No Intro' if no_intro else 'Intro'}, {'End' if has_end else 'No End'})")
        if not (no_intro and has_end):
            all_good = False

    print("="*60)
    if all_good:
        print("[OK] All checked chapters have only end-of-chapter summaries!")
    else:
        print("[ISSUE] Some chapters still have introductory summaries or missing end summaries.")

if __name__ == "__main__":
    main()