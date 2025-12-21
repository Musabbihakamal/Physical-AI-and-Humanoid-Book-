#!/usr/bin/env python3
"""
Final script to verify that all chapters have only end-of-chapter summaries.
"""

import os
from pathlib import Path

def verify_all_chapters():
    """Verify that all chapters have only end-of-chapter summaries."""
    frontend_docs = Path("frontend/docs")
    all_md_files = list(frontend_docs.rglob("*.md"))

    # Filter out non-chapter files
    chapter_files = []
    for file_path in all_md_files:
        if "agent-guides" not in str(file_path) and "deployment-guide" not in str(file_path):
            chapter_files.append(file_path)

    results = []

    for file_path in chapter_files:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        lines = content.split('\n')

        # Find all summary sections
        summary_positions = []
        for i, line in enumerate(lines):
            if line.strip() == '## Summary':
                summary_positions.append(i)

        # Check if there's a summary in the first 25 lines (introductory)
        has_intro_summary = False
        for pos in summary_positions:
            if pos < 25:  # If summary is in first 25 lines, it's introductory
                has_intro_summary = True
                break

        # Check if there's a summary near the end (after main content)
        has_end_summary = False
        content_lower = content.lower()

        # Look for indicators of end content (exercises, quiz, references)
        has_end_indicators = any(keyword in content_lower for keyword in ['## exercises', '## quiz', '## references', 'exercises', 'quiz', 'references'])

        if has_end_indicators:
            # If the file has end indicators, check if there's a summary before them
            for pos in summary_positions:
                # Look at content after this summary position
                content_after = '\n'.join(lines[pos+1:pos+50])  # Check 50 lines after summary
                if any(keyword in content_after.lower() for keyword in ['exercises', 'quiz', 'references']):
                    has_end_summary = True
                    break

        # For files without clear end indicators, look for summaries that are not in the first 25 lines
        if not has_end_indicators and not has_intro_summary:
            for pos in summary_positions:
                if pos >= 25:  # Summary is not in the first 25 lines
                    # Check if it's reasonably close to the end
                    if pos > len(lines) * 0.7:  # Summary is in the last 30% of the file
                        has_end_summary = True
                        break

        results.append((str(file_path), not has_intro_summary, has_end_summary))

    return results

def main():
    print("Final Verification: Checking all chapters for summary placement...")
    print("="*80)

    results = verify_all_chapters()

    # Separate results
    good_files = []
    bad_files = []

    for file_path, no_intro, has_end in results:
        if no_intro and has_end:
            good_files.append((file_path, no_intro, has_end))
        else:
            bad_files.append((file_path, no_intro, has_end))

    print(f"Files with correct summary placement (no intro, has end): {len(good_files)}")
    print(f"Files with incorrect summary placement: {len(bad_files)}")
    print()

    if bad_files:
        print("Files with issues:")
        for file_path, no_intro, has_end in bad_files:
            issue = []
            if no_intro and not has_end:
                issue.append("No end summary")
            elif not no_intro and has_end:
                issue.append("Still has intro summary")
            elif not no_intro and not has_end:
                issue.append("Has intro summary and no end summary")
            else:
                issue.append("Unknown issue")
            print(f"  - {file_path}: {', '.join(issue)}")
    else:
        print("✅ All chapters have correct summary placement!")
        print("  - No introductory summaries at the beginning")
        print("  - End-of-chapter summaries present where appropriate")

    print("="*80)

    if len(bad_files) == 0:
        print("[SUCCESS] All chapters follow the required format with summaries at the end only!")
    else:
        print(f"[ISSUE] Issues found: {len(bad_files)} files need correction")

if __name__ == "__main__":
    main()