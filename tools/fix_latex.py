#!/usr/bin/env python3
"""Convert LaTeX notation from \(...\) to $...$ and \[...\] to $$...$$"""

import re

def convert_latex(content):
    # Convert inline math \(...\) to $...$
    content = re.sub(r'\\\(([^)]+?)\\\)', r'$\1$', content)
    
    # Convert display math \[...\] to $$...$$
    # Handle multiline by using DOTALL flag
    content = re.sub(r'\\\[\s*\n?(.*?)\n?\s*\\\]', r'$$\1$$', content, flags=re.DOTALL)
    
    # Remove \boxed{} commands - just keep the content
    content = re.sub(r'\\boxed\{([^}]+)\}', r'\1', content)
    
    # Clean up text commands
    content = content.replace(r'\ \text{', ' ')
    
    return content

if __name__ == '__main__':
    with open('modelling_complete.md', 'r') as f:
        content = f.read()
    
    converted = convert_latex(content)
    
    with open('modelling_complete.md', 'w') as f:
        f.write(converted)
    
    print("LaTeX formatting converted successfully!")
