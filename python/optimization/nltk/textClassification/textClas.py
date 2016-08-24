import nltk
import random
from nltk.corpus import movie_reviews

# List of tuples
documents=[(list(movie_reviews.words(fileid)),category)
	for category in movie_reviews.categories()
	for fileid in movie_reviews.fileids(category)]


#for category in movie_reviews.categories():
#	for fileid in movie_reviews.fileids(category):
#		documents.append(list(movie_reviews.words(fileid)),category)


random.shuffle(documents)


# Prints all the words in the reviews collection and its label pos/neg
#print(documents[2])


words = []
# Converto to lower case
for w in movie_reviews.words():
	words.append(w.lower())


# convert to nltk frequency distribution
all_words = nltk.FreqDist(words)
print(all_words.most_common(15))

print(all_words["stupid"])

