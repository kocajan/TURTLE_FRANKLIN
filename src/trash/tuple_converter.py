data = []
with open('excel_data.txt', 'r') as f:
    for line in f:
        left, right = map(int, line.strip().split())
        data.append((left, right))

print(data)