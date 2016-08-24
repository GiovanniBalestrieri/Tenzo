from nltk.corpus import stopwords
from nltk.tokenize import word_tokenize

sent = "Hey come va? Io bene tu. Qui fa caldo e siamo in estate. Quando lo leggerai probabilmente fara' freddo."

sent1 = "Where do we get if go ahead? I don't know my friend, probably we would end up in a paradise"


stop_words = set(stopwords.words("english"))

words = word_tokenize(sent1)
print(words)

filtered_sentence = []

for w in words:
	if w not in stop_words:
		filtered_sentence.append(w)

print filtered_sentence


