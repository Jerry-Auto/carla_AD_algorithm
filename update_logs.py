import re
import sys

file_path = sys.argv[1]
with open(file_path, 'r') as f:
    content = f.read()

# 1. Handle the ostringstream blocks I just added
def simplify_oss(match):
    indent = match.group(1)
    level = match.group(3)
    # Extract the parts from oss << ...
    oss_content = match.group(2)
    parts = re.findall(r'<<\s*(.*?)\s*(?=<<|$)', oss_content)
    # Clean up parts (remove std::endl, etc.)
    parts = [p.strip() for p in parts if p.strip() != 'std::endl']
    return f'{indent}log({level}, {", ".join(parts)});'

# Pattern for the ostringstream blocks
oss_pattern = re.compile(r'(\s+){\s+std::ostringstream oss;\s+oss\s+(.*?);\s+log\((.*?), oss\.str\(\)\);\s+}', re.DOTALL)
content = oss_pattern.sub(simplify_oss, content)

# Pattern for the if(_enable_log) { std::ostringstream oss; ... } blocks
oss_if_pattern = re.compile(r'(\s+)if\s*\(_enable_log\)\s*{\s+std::ostringstream oss;\s+oss\s+(.*?);\s+log\((.*?), oss\.str\(\)\);\s+}', re.DOTALL)
content = oss_if_pattern.sub(simplify_oss, content)

# 2. Handle simple log(msg, level) -> log(level, msg)
def swap_log(match):
    msg = match.group(1)
    level = match.group(2)
    return f'log({level}, {msg})'

content = re.sub(r'log\(([^,]+),\s*("[A-Z]+")\)', swap_log, content)

# 3. Handle log(msg) -> log("INFO", msg)
# We need to be careful not to match log(level, msg) that we just swapped
# So we only match if there's no comma inside the parens (simplified)
def add_info(match):
    args = match.group(1)
    if ',' not in args:
        return f'log("INFO", {args})'
    return match.group(0)

content = re.sub(r'log\(([^)]+)\)', add_info, content)

with open(file_path, 'w') as f:
    f.write(content)
