import re
import os

file_path = 'rPi controller/controller.py'

if not os.path.exists(file_path):
    print("File not found")
    exit(1)

with open(file_path, 'r') as f:
    content = f.read()

# Regex to find except Exception followed by pass, capturing indentation
# (?m) enables multiline mode for ^
# ^(\s*) captures indentation
pattern = r'^(\s*)except Exception:\s*\n\s*pass'

def replace_func(match):
    indent = match.group(1)
    # Use the same indentation for the new block
    # indent + except ...
    # indent + 4 spaces + logging...
    return f'{indent}except Exception as e:\n{indent}    logging.warning(f"Ignored error: {{e}}")'

new_content, count = re.subn(pattern, replace_func, content, flags=re.MULTILINE)

print(f"Replaced {count} occurrences.")

with open(file_path, 'w') as f:
    f.write(new_content)
