#!/usr/bin/env python3

import json
import re

def remove_comments(json_like):
    """Remove C-style comments from a JSON-like string."""
    pattern = r'//.*?$|/\*.*?\*/'
    return re.sub(pattern, '', json_like, flags=re.DOTALL | re.MULTILINE)

def remove_empty_lines(text):
    """Remove empty lines from a text string while keeping line breaks."""
    return "\n".join([line for line in text.splitlines() if line.strip()])

def json_to_dict(path_to_json):
    with open(path_to_json, 'r') as file:
        content = file.read()

        content_no_comments = remove_comments(content)
        # Return parsed JSON
        return json.loads(content_no_comments)

# Function to load template and perform substitutions
def process_template(template_path, substitutions):
    # Read the template content
    with open(template_path, 'r') as file:
        content = file.read()
    # Perform substitutions
    for key, value in substitutions.items():
        content = content.replace(key, value)

    content = remove_comments(content)
    content = remove_empty_lines(content)
    
    return content
