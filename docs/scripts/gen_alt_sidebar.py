#! /usr/bin/python

"""
Generates a file _sidebar.md for each SUMMARY.md in the immediate folders (e.g. /en/SUMMARY.md)
This is stored in github. Can be called at build time to create/deploy for docsify builds.
Note, we don't "support" docsify - this is experimental
"""
import os
import re


def modify_summary_links(directory):
    # Iterate over all immediate subdirectories
    for subdir in next(os.walk(directory))[1]:
        summary_md_path = os.path.join(directory, subdir, 'SUMMARY.md')
        # Check if SUMMARY.md exists in the subdirectory
        if os.path.isfile(summary_md_path):
            # print(f"Processing: {summary_md_path}")
            # Read the SUMMARY.md content
            with open(summary_md_path, 'r', encoding='utf-8') as f:
                summary_content = f.read()

            # Modify the URLs to start with '/'
            modified_content = re.sub(r'\]\((?!/)', '](/', summary_content)

            # Add generated disclaimer
            disclaimer = "<!-- GENERATED CONTENT: DO NOT EDIT -->\n"
            modified_content = disclaimer + modified_content
            # Create the output file path
            output_path = os.path.join(directory, subdir, '_sidebar.md')

            # Write the modified content to the output file
            with open(output_path, 'w', encoding='utf-8') as f:
                f.write(modified_content)

            # print(f"Modified SUMMARY.md saved as {output_path}")


if __name__ == "__main__":
    # Specify the directory to search for SUMMARY.md files
    specified_directory = "."

    # Call the function to modify SUMMARY.md links in all subdirectories
    modify_summary_links(specified_directory)
