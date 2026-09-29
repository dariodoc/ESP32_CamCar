import os

for filename in os.listdir('src'):
    if filename.endswith('.cpp') or filename.endswith('.h'):
        content = open(f'src/{filename}', 'r', encoding='utf-8', errors='ignore').read()
        lines = content.split('\n')
        for i, line in enumerate(lines):
            if 'while' in line:
                print(f'{filename}:{i+1}: {line.strip()}')
