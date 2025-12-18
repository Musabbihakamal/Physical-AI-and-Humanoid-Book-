#!/usr/bin/env python3
"""
Script to find chapters that are missing end-of-chapter summaries.
"""

import os
from pathlib import Path

def find_chapters_without_end_summaries():
    """Find all markdown files that don't have an end-of-chapter summary."""
    frontend_docs = Path("frontend/docs")
    missing_summary_files = []

    # Find all markdown files in the docs directory
    for md_file in frontend_docs.rglob("*.md"):
        if "agent-guides" not in str(md_file):  # Skip agent guides for now
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Check if there's a summary section at the end
            # Look for ## Summary after the main content
            lines = content.split('\n')

            # Find all "## Summary" sections
            summary_positions = []
            for i, line in enumerate(lines):
                if line.strip() == '## Summary':
                    summary_positions.append(i)

            # If there's only one summary and it's near the beginning (likely the intro summary),
            # then there's no end-of-chapter summary
            has_end_summary = False

            if len(summary_positions) > 1:
                # If there are multiple summaries, there might be one at the end
                # Check if any summary is after the main content sections
                for pos in summary_positions:
                    # Look for the content after this summary position
                    # to see if it's followed by other sections like Exercises, Quiz, etc.
                    content_after_summary = '\n'.join(lines[pos+1:pos+20])  # Check next 20 lines
                    if any(keyword in content_after_summary.lower() for keyword in ['exercises', 'quiz', 'references', '## exerc', '## quiz', '## refer']):
                        has_end_summary = True
                        break
            elif len(summary_positions) == 1:
                # Check if the summary is at the beginning (likely intro summary)
                if summary_positions[0] < 20:  # If summary is in first 20 lines
                    # Check if there are sections like Exercises, Quiz, or References after main content
                    content_lower = content.lower()
                    if not any(keyword in content_lower for keyword in ['## exerc', '## quiz', '## refer']):
                        # If there are no exercise/quiz/references sections, this might be fine
                        # But if there ARE these sections, and only one summary at the beginning,
                        # then we're missing an end summary
                        if any(keyword in content_lower for keyword in ['exercises', 'quiz', 'references']):
                            has_end_summary = False
                        else:
                            has_end_summary = True  # No ending sections, so single summary might be sufficient
                    else:
                        has_end_summary = False  # Has ending sections but only one summary at beginning

            if not has_end_summary and any(keyword in content.lower() for keyword in ['exercises', 'quiz', 'references']):
                missing_summary_files.append(md_file)

    return missing_summary_files

def main():
    missing_summaries = find_chapters_without_end_summaries()

    print(f"Found {len(missing_summaries)} chapters without end-of-chapter summaries:")
    for file_path in missing_summaries:
        print(f"  - {file_path}")

    # Write to a file for reference
    with open("missing_summaries.txt", "w") as f:
        for file_path in missing_summaries:
            f.write(f"{file_path}\n")

    print(f"\nList of files saved to missing_summaries.txt")

if __name__ == "__main__":
    main()