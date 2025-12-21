#!/usr/bin/env python3
"""
Script to find all chapters that have introductory summaries (in the first 20 lines).
"""

import os
from pathlib import Path

def find_chapters_with_intro_summaries():
    """Find all markdown files that have a summary in the first 20 lines."""
    frontend_docs = Path("frontend/docs")
    intro_summary_files = []

    # Find all markdown files in the docs directory
    for md_file in frontend_docs.rglob("*.md"):
        # Skip agent guides and other non-chapter files
        if "agent-guides" not in str(md_file) and "deployment-guide" not in str(md_file):
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

            lines = content.split('\n')

            # Check if there's a summary in the first 20 lines (after the frontmatter)
            has_intro_summary = False
            for i, line in enumerate(lines[:25]):  # Check first 25 lines to account for frontmatter
                if line.strip() == '## Summary':
                    # If it's in the first 20 lines after content starts, it's an intro summary
                    if i < 25:  # Allow for frontmatter
                        has_intro_summary = True
                        break

            if has_intro_summary:
                intro_summary_files.append(str(md_file))

    return intro_summary_files

def main():
    intro_summaries = find_chapters_with_intro_summaries()

    print(f"Found {len(intro_summaries)} chapters with introductory summaries:")
    for file_path in intro_summaries:
        print(f"  - {file_path}")

    # Write to a file for reference
    with open("intro_summaries.txt", "w") as f:
        for file_path in intro_summaries:
            f.write(f"{file_path}\n")

    print(f"\nList of files saved to intro_summaries.txt")

if __name__ == "__main__":
    main()