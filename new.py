from itertools import permutations, combinations

a = ["a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z"]
b = []
for i in a:
  b.append(i.capitalize())

a.extend(b)
for i in range(10):
  a.append(str(i))
print(a)
result = combinations(a, 8)
for i in result:
  print(i)