#!/usr/bin/env python3
"""
Script to automatically remove introductory summaries from all chapters.
"""

import os
from pathlib import Path

def remove_introductory_summaries():
    """Remove introductory summaries from all chapters."""
    # Read the list of files with intro summaries from our previous script
    with open("intro_summaries.txt", "r") as f:
        files_to_process = [line.strip() for line in f.readlines() if line.strip()]

    results = []

    for file_path in files_to_process:
        full_path = Path(file_path)
        if full_path.exists():
            try:
                with open(full_path, 'r', encoding='utf-8') as f:
                    content = f.read()

                lines = content.split('\n')

                # Find the first '## Summary' line in the first 25 lines
                intro_summary_start = -1
                intro_summary_end = -1

                for i, line in enumerate(lines[:25]):
                    if line.strip() == '## Summary':
                        intro_summary_start = i
                        # Find where the summary content ends (next section or empty lines)
                        j = i + 1
                        while j < len(lines) and j < i + 15:  # Look ahead up to 15 lines
                            if lines[j].strip().startswith('## ') and not lines[j].strip().startswith('## Summary'):
                                # Found next section header
                                intro_summary_end = j
                                break
                            elif lines[j].strip() == '' and j > i + 1:
                                # Found empty line after summary content
                                # Check if the next non-empty line is a new section
                                k = j + 1
                                while k < len(lines) and k < i + 15:
                                    if lines[k].strip() != '':
                                        if lines[k].strip().startswith('## ') and not lines[k].strip().startswith('## Summary'):
                                            intro_summary_end = j
                                            break
                                        else:
                                            break  # Not a new section, continue looking
                                    k += 1
                                if intro_summary_end != -1:
                                    break
                            j += 1

                        if intro_summary_end == -1:
                            # If we couldn't find a clear end, use a reasonable end point
                            intro_summary_end = min(i + 5, len(lines))

                        break

                if intro_summary_start != -1 and intro_summary_end != -1:
                    # Remove the introductory summary section
                    new_lines = lines[:intro_summary_start] + lines[intro_summary_end:]
                    new_content = '\n'.join(new_lines)

                    with open(full_path, 'w', encoding='utf-8') as f:
                        f.write(new_content)

                    results.append((file_path, True, f"Removed intro summary at line {intro_summary_start+1}"))
                else:
                    results.append((file_path, False, "No intro summary found or couldn't locate it properly"))

            except Exception as e:
                results.append((file_path, False, f"Error processing file: {str(e)}"))
        else:
            results.append((file_path, False, "File does not exist"))

    return results

def main():
    print("Removing introductory summaries from all chapters...")
    results = remove_introductory_summaries()

    print(f"\nProcessing complete. Results:")
    print("="*70)

    success_count = 0
    fail_count = 0

    for file_path, success, message in results:
        status = "[SUCCESS]" if success else "[FAILED]"
        print(f"{status} {file_path}")
        if message:
            print(f"         {message}")

        if success:
            success_count += 1
        else:
            fail_count += 1

    print("="*70)
    print(f"Summary: {success_count} files processed successfully, {fail_count} failed")

    if fail_count == 0:
        print("[OK] All introductory summaries have been removed!")
    else:
        print(f"[WARNING] {fail_count} files had issues during processing.")

if __name__ == "__main__":
    main()