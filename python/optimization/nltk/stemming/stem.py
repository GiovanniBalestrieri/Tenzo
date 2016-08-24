from nltk.stem import PorterStemmer
from nltk.tokenize import word_tokenize

ps = PorterStemmer()

ex = ["python","pythoner","pythons","pythoning","pythonly"]

for w in ex:
	print(ps.stem(w))


print("Testing Stemming in a sentence")

test = "It is very important to be pythonly while you are pythoning with python. All pythoners have pythonic hands"

words = word_tokenize(test)

for w in words:
	print(ps.stem(w))
